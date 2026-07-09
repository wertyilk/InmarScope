#include "sdr/rtl_tcp_source.h"

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
using socket_t = SOCKET;
#define CLOSESOCK closesocket
inline void shutSock(socket_t s) { ::shutdown(s, SD_BOTH); }
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
using socket_t = int;
#define INVALID_SOCKET (-1)
#define CLOSESOCK ::close
inline void shutSock(socket_t s) { ::shutdown(s, SHUT_RDWR); }
#endif

#include <cstdio>
#include <cstring>

#if defined(_WIN32)
namespace {
struct WsaInit { WsaInit() { WSADATA d; WSAStartup(MAKEWORD(2, 2), &d); } };
static WsaInit g_wsa;
}
#define INVALID_SOCK (~(uintptr_t)0)
#else
#define INVALID_SOCK (~(uintptr_t)0)
#endif

RtlTcpSource::~RtlTcpSource() { stop(); }

bool RtlTcpSource::sendCmd(uint8_t opcode, const void* data, int dataLen)
{
    std::lock_guard<std::mutex> lk(sockMtx_);
    if (sock_ == INVALID_SOCK) return false;
    uint8_t buf[5] = {opcode};
    if (data && dataLen > 0 && dataLen <= 4)
        std::memcpy(buf + 1, data, dataLen);
    int total = 1 + (data ? dataLen : 0);
    int sent = 0;
    while (sent < total)
    {
        int n = ::send((socket_t)sock_, (const char*)(buf + sent), total - sent, 0);
        if (n <= 0) return false;
        sent += n;
    }
    return true;
}

void RtlTcpSource::setCenterFreq(double hz)
{
    centerFreq_ = hz;
    uint32_t f = (uint32_t)hz;
    sendCmd(0x03, &f, 4);
}

void RtlTcpSource::setSampleRate(double hz)
{
    sampleRate_ = hz;
    uint32_t r = (uint32_t)hz;
    sendCmd(0x04, &r, 4);
}

void RtlTcpSource::setGain(double db)
{
    gainDb_ = db;
    if (db < 0.0)
    {
        uint32_t mode = 0;
        sendCmd(0x02, &mode, 4);
    }
    else
    {
        uint32_t mode = 1;
        sendCmd(0x02, &mode, 4);
        uint32_t val = (uint32_t)(db * 10.0);
        sendCmd(0x05, &val, 4);
    }
}

void RtlTcpSource::setBiasTee(bool on)
{
    biasTee_ = on;
    uint32_t v = on ? 1 : 0;
    sendCmd(0x0d, &v, 4);
}

void RtlTcpSource::setPpm(double ppm)
{
    ppm_ = ppm;
    int16_t v = (int16_t)ppm;
    sendCmd(0x06, &v, 2);
}

bool RtlTcpSource::start(int, SdrSampleCb cb, std::string& err)
{
    if (running_.load()) stop();

    struct addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    struct addrinfo* ai = nullptr;
    char portStr[16];
    std::snprintf(portStr, sizeof(portStr), "%u", (unsigned)port_);
    int rc = getaddrinfo(host_.c_str(), portStr, &hints, &ai);
    if (rc != 0 || !ai)
    {
        err = "RTL-TCP: getaddrinfo(" + host_ + ":" + portStr + ") failed";
        if (ai) freeaddrinfo(ai);
        return false;
    }

    socket_t s = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (s == INVALID_SOCKET)
    {
        err = "RTL-TCP: socket() failed";
        freeaddrinfo(ai);
        return false;
    }
    if (::connect(s, ai->ai_addr, (int)ai->ai_addrlen) != 0)
    {
        err = "RTL-TCP: connect() failed";
        CLOSESOCK(s);
        freeaddrinfo(ai);
        return false;
    }
    freeaddrinfo(ai);

    int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));

    // Read dongle info string.
    char info[256]{};
    int infoPos = 0;
    while (infoPos < (int)sizeof(info) - 1)
    {
        int n = ::recv(s, info + infoPos, 1, 0);
        if (n <= 0) { err = "RTL-TCP: no dongle info"; CLOSESOCK(s); return false; }
        if (info[infoPos] == '\n') break;
        infoPos++;
    }
    info[infoPos] = 0;

    {
        std::lock_guard<std::mutex> lk(sockMtx_);
        sock_ = (uintptr_t)s;
    }

    cb_ = cb;
    running_.store(true);

    // Send initial configuration before starting the recv thread.
    setCenterFreq(centerFreq_);
    setSampleRate(sampleRate_);
    setGain(gainDb_);
    setPpm(ppm_);
    setBiasTee(biasTee_);

    rbuf_.resize(65536);
    fbuf_.resize(65536);
    thread_ = std::thread(&RtlTcpSource::recvLoop, this);
    return true;
}

void RtlTcpSource::stop()
{
    running_.store(false);

    socket_t localSock = INVALID_SOCKET;
    {
        std::lock_guard<std::mutex> lk(sockMtx_);
        if (sock_ != INVALID_SOCK)
        {
            localSock = (socket_t)sock_;
            sock_ = INVALID_SOCK;
        }
    }

    if (localSock != INVALID_SOCKET)
    {
        shutSock(localSock);
        CLOSESOCK(localSock);
    }

    if (thread_.joinable())
        thread_.join();

    cb_ = nullptr;
}

void RtlTcpSource::recvLoop()
{
    socket_t s;
    {
        std::lock_guard<std::mutex> lk(sockMtx_);
        s = (sock_ != INVALID_SOCK) ? (socket_t)sock_ : INVALID_SOCKET;
    }
    if (s == INVALID_SOCKET) return;

    dcOffRe_ = dcOffIm_ = 0.0f;

    while (running_.load())
    {
        int n = ::recv(s, (char*)rbuf_.data(), (int)rbuf_.size(), 0);
        if (n <= 0)
        {
            if (running_.load())
            {
#if defined(_WIN32)
                int e = WSAGetLastError();
                if (e == WSAEWOULDBLOCK || e == WSAEINTR) continue;
#else
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
#endif
            }
            break;
        }
        int nSamp = n / 2;
        if (nSamp * 2 > (int)fbuf_.size())
            fbuf_.resize(nSamp * 2);
        for (int i = 0; i < nSamp; ++i)
        {
            float re = ((int)rbuf_[i * 2]     - 128) / 128.0f;
            float im = ((int)rbuf_[i * 2 + 1] - 128) / 128.0f;
            dcOffRe_ = dcOffRe_ * (1.0f - dcRate_) + re * dcRate_;
            dcOffIm_ = dcOffIm_ * (1.0f - dcRate_) + im * dcRate_;
            fbuf_[i * 2]     = re - dcOffRe_;
            fbuf_[i * 2 + 1] = im - dcOffIm_;
        }
        if (cb_)
            cb_(fbuf_.data(), nSamp);
    }
    running_.store(false);
}
