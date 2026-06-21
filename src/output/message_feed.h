// Decoded-message output feed. Emits ACARS and EGC messages as newline-
// delimited JSON (JAERO JSONdump / inmarsat-sniffer compatible) and/or the
// JAERO text format, to a file and/or UDP endpoint. Thread-safe.
#pragma once

#include "decode/message_log.h"

#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>

class MessageFeed
{
public:
    enum Format { JSON = 0, JAERO_TEXT = 1 };

    ~MessageFeed();

    // Configuration (safe to call live).
    void setFileEnabled(bool on, const std::string& path);
    void setUdpEnabled(bool on, const std::string& host, int port);
    // SBS / BaseStation (port 30003 style) position feed over UDP. Emits one
    // MSG,3 line per ACARS message that carries an ADS-C position.
    void setSbsEnabled(bool on, const std::string& host, int port);
    void setFormat(int fmt) { format_ = fmt; }
    void setStationId(const std::string& s) { station_ = s; }

    bool enabled() const { return fileEnabled_ || udpEnabled_ || sbsEnabled_; }
    uint64_t sent() const { return sent_; }
    uint64_t sbsSent() const { return sbsSent_; }

    void feedAcars(const DecodedMessage& m);
    void feedEgc(const EgcMessage& m);

private:
    void emit(const std::string& line);
    void emitSbs(const DecodedMessage& m);
    void openFileIfNeeded();
    void closeFile();
    void ensureUdp();
    void closeUdp();
    void ensureSbs();
    void closeSbs();

    std::mutex mtx_;
    bool fileEnabled_ = false;
    std::string filePath_;
    std::FILE* file_ = nullptr;

    bool udpEnabled_ = false;
    std::string udpHost_;
    int udpPort_ = 0;
    uintptr_t sock_ = ~(uintptr_t)0;
    void* addr_ = nullptr; // sockaddr_in*

    bool sbsEnabled_ = false;
    std::string sbsHost_;
    int sbsPort_ = 0;
    uintptr_t sbsSock_ = ~(uintptr_t)0;
    void* sbsAddr_ = nullptr; // sockaddr_in*
    uint64_t sbsSent_ = 0;

    int format_ = JSON;
    std::string station_;
    uint64_t sent_ = 0;
};
