// Minimal PCM WAV file writer (used to save decoded 8400 voice calls).
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

class WavWriter
{
public:
    WavWriter() = default;
    ~WavWriter() { close(); }

    WavWriter(const WavWriter&) = delete;
    WavWriter& operator=(const WavWriter&) = delete;

    // Create/overwrite a 16-bit PCM WAV. Returns false if the file can't open.
    bool open(const std::string& path, int sampleRate = 8000, int channels = 1);

    // Append interleaved 16-bit samples (count = frames * channels).
    void write(const int16_t* data, int count);

    // Finalize the header and close. Safe to call multiple times.
    void close();

    bool isOpen() const { return f_ != nullptr; }
    uint32_t dataBytes() const { return dataBytes_; }

private:
    std::FILE* f_ = nullptr;
    uint32_t dataBytes_ = 0;
    int sampleRate_ = 8000;
    int channels_ = 1;
};
