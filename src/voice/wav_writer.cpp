#include "voice/wav_writer.h"

#include <cstring>

namespace
{
void put32(std::FILE* f, uint32_t v)
{
    uint8_t b[4] = {(uint8_t)(v), (uint8_t)(v >> 8), (uint8_t)(v >> 16),
                    (uint8_t)(v >> 24)};
    std::fwrite(b, 1, 4, f);
}
void put16(std::FILE* f, uint16_t v)
{
    uint8_t b[2] = {(uint8_t)(v), (uint8_t)(v >> 8)};
    std::fwrite(b, 1, 2, f);
}
} // namespace

bool WavWriter::open(const std::string& path, int sampleRate, int channels)
{
    close();
    f_ = std::fopen(path.c_str(), "wb");
    if (!f_)
        return false;
    sampleRate_ = sampleRate;
    channels_ = channels;
    dataBytes_ = 0;

    const uint32_t byteRate = (uint32_t)(sampleRate_ * channels_ * 2);
    const uint16_t blockAlign = (uint16_t)(channels_ * 2);

    std::fwrite("RIFF", 1, 4, f_);
    put32(f_, 36); // RIFF chunk size (patched on close)
    std::fwrite("WAVE", 1, 4, f_);
    std::fwrite("fmt ", 1, 4, f_);
    put32(f_, 16);                  // fmt chunk size
    put16(f_, 1);                   // PCM
    put16(f_, (uint16_t)channels_); // channels
    put32(f_, (uint32_t)sampleRate_);
    put32(f_, byteRate);
    put16(f_, blockAlign);
    put16(f_, 16); // bits per sample
    std::fwrite("data", 1, 4, f_);
    put32(f_, 0); // data chunk size (patched on close)
    return true;
}

void WavWriter::write(const int16_t* data, int count)
{
    if (!f_ || count <= 0)
        return;
    std::fwrite(data, sizeof(int16_t), (size_t)count, f_);
    dataBytes_ += (uint32_t)count * sizeof(int16_t);
}

void WavWriter::close()
{
    if (!f_)
        return;
    // Patch the RIFF size (offset 4) and data size (offset 40).
    std::fseek(f_, 4, SEEK_SET);
    put32(f_, 36 + dataBytes_);
    std::fseek(f_, 40, SEEK_SET);
    put32(f_, dataBytes_);
    std::fclose(f_);
    f_ = nullptr;
}
