// Headless A/B benchmark: run the real Decoder on known channels of a
// baseband WAV capture and report lock quality metrics per channel.
//
//   decode_bench <capture.wav> <seconds> <chan> [chan ...]
//     chan = <baud>@<offsetHz>   e.g. 600@-478000  10500@419450  egc@-10000
//
// All channels decode in a single pass over the file (same as the app's
// decoder_manager), so a run is directly comparable across DSP changes.
#include "decode/decoder.h"
#include "decode/message_log.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

static uint32_t rd32(const unsigned char* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t rd16(const unsigned char* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

struct Chan
{
    int baud;
    double offHz;
    std::unique_ptr<MessageLog> log, suLog;
    std::unique_ptr<Decoder> dec;
    uint64_t lockedBlocks = 0, blocks = 0;
    double firstLockSec = -1.0;
    double ebnoSum = 0.0; // averaged over locked blocks only
    uint64_t ebnoN = 0;
};

int main(int argc, char** argv)
{
    if (argc < 4)
    {
        std::fprintf(stderr, "usage: %s <capture.wav> <seconds> <baud>@<offsetHz> ...\n", argv[0]);
        return 2;
    }
    const char* path = argv[1];
    double seconds = atof(argv[2]);

    std::ifstream f(path, std::ios::binary);
    if (!f) { std::fprintf(stderr, "cannot open %s\n", path); return 1; }
    unsigned char hdr[12];
    f.read((char*)hdr, 12);
    int channels = 2, bits = 16;
    double Fs = 0;
    uint64_t dataOff = 0, dataSz = 0;
    while (f)
    {
        unsigned char ch[8];
        f.read((char*)ch, 8);
        if (!f) break;
        uint32_t sz = rd32(ch + 4);
        if (!std::memcmp(ch, "fmt ", 4))
        {
            std::vector<unsigned char> fmt(sz);
            f.read((char*)fmt.data(), sz);
            channels = rd16(fmt.data() + 2);
            Fs = rd32(fmt.data() + 4);
            bits = rd16(fmt.data() + 14);
            if (sz & 1) f.seekg(1, std::ios::cur);
        }
        else if (!std::memcmp(ch, "data", 4)) { dataOff = (uint64_t)f.tellg(); dataSz = sz; break; }
        else f.seekg(sz + (sz & 1), std::ios::cur);
    }
    if (!Fs || !dataOff) { std::fprintf(stderr, "bad wav\n"); return 1; }
    std::fprintf(stderr, "file: %d ch %d-bit %.0f Hz\n", channels, bits, Fs);

    std::vector<Chan> chans;
    for (int a = 3; a < argc; ++a)
    {
        const char* at = std::strchr(argv[a], '@');
        if (!at) { std::fprintf(stderr, "bad chan spec %s\n", argv[a]); return 2; }
        Chan c;
        c.baud = std::strncmp(argv[a], "egc", 3) == 0 ? kEgcBaud : atoi(argv[a]);
        c.offHz = atof(at + 1);
        chans.push_back(std::move(c));
    }
    for (size_t i = 0; i < chans.size(); ++i)
    {
        Chan& c = chans[i];
        c.log = std::make_unique<MessageLog>();
        c.suLog = std::make_unique<MessageLog>();
        c.dec = std::make_unique<Decoder>(Fs, 0.0, c.offHz, c.baud, (int)i + 1,
                                          c.log.get(), c.suLog.get(), nullptr,
                                          nullptr, nullptr);
    }

    const int frameBytes = channels * (bits / 8);
    uint64_t nFrames = dataSz / frameBytes;
    uint64_t maxFrames = (uint64_t)(seconds * Fs);
    if (seconds > 0 && maxFrames < nFrames) nFrames = maxFrames;

    f.seekg((std::streamoff)dataOff, std::ios::beg);
    const int B = 65536;
    std::vector<unsigned char> raw((size_t)B * frameBytes);
    std::vector<double> iq((size_t)B * 2);
    uint64_t done = 0;
    while (done < nFrames)
    {
        int want = (int)std::min<uint64_t>(B, nFrames - done);
        f.read((char*)raw.data(), (std::streamsize)want * frameBytes);
        int got = (int)(f.gcount() / frameBytes);
        if (got <= 0) break;
        for (int i = 0; i < got; ++i)
        {
            const unsigned char* p = raw.data() + (size_t)i * frameBytes;
            if (bits == 8)
            {
                iq[i * 2] = ((int)p[0] - 128) * (1.0 / 128.0);
                iq[i * 2 + 1] = (channels == 2) ? ((int)p[1] - 128) * (1.0 / 128.0) : 0.0;
            }
            else
            {
                iq[i * 2] = (int16_t)rd16(p) * (1.0 / 32768.0);
                iq[i * 2 + 1] = (channels == 2) ? (int16_t)rd16(p + 2) * (1.0 / 32768.0) : 0.0;
            }
        }
        double tNow = (double)done / Fs;
        for (Chan& c : chans)
        {
            c.dec->process(iq.data(), got);
            ++c.blocks;
            if (c.dec->locked())
            {
                ++c.lockedBlocks;
                if (c.firstLockSec < 0) c.firstLockSec = tNow;
                double e = c.dec->ebno();
                if (std::isfinite(e)) { c.ebnoSum += e; ++c.ebnoN; }
            }
        }
        done += got;
        std::fprintf(stderr, "\r%.1f / %.1f s", (double)done / Fs, (double)nFrames / Fs);
    }
    std::fprintf(stderr, "\n");

    std::printf("%9s %10s %6s %8s %8s %8s %8s %8s %8s %8s\n",
                "baud", "off_Hz", "lock%", "tlock_s", "ebno_avg", "ebno_end", "mse_end", "msgs", "SUs", "voice");
    for (Chan& c : chans)
    {
        if (c.baud == kEgcBaud)
        {
            std::printf("%9s %10.0f %6.1f %8.1f %8s %8s %8d %8llu %8d\n",
                        "egc", c.offHz,
                        100.0 * c.lockedBlocks / std::max<uint64_t>(1, c.blocks),
                        c.firstLockSec, "-", "-", c.dec->egcBer(),
                        (unsigned long long)c.dec->msgCount(), c.dec->egcFrames());
            continue;
        }
        std::printf("%9d %10.0f %6.1f %8.1f %8.1f %8.1f %8.3f %8llu %8llu %8llu\n",
                    c.baud, c.offHz,
                    100.0 * c.lockedBlocks / std::max<uint64_t>(1, c.blocks),
                    c.firstLockSec,
                    c.ebnoN ? c.ebnoSum / c.ebnoN : 0.0,
                    c.dec->ebno(), c.dec->mse(),
                    (unsigned long long)c.log->count(),
                    (unsigned long long)c.suLog->count(),
                    (unsigned long long)c.dec->voiceFrames());
    }
    return 0;
}
