#include "voice/wav_writer.h"

#include <cstring>

// miniaudio header-only: the implementation is compiled in audio_output.cpp.
// Here we only need the declarations (ma_encoder, etc.).
#define MA_NO_DECODING
#define MA_NO_RESOURCE_MANAGER
#define MA_NO_ENGINE
#define MA_NO_GENERATION
#include "miniaudio.h"

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
    sampleRate_ = sampleRate;
    channels_ = channels;

    if (fmt_ == RecordFormat::OGG)
    {
        ma_encoder* enc = new ma_encoder{};
        ma_encoder_config cfg = ma_encoder_config_init(ma_encoding_format_vorbis,
                                                        ma_format_s16, (ma_uint32)channels,
                                                        (ma_uint32)sampleRate);
        ma_result r = ma_encoder_init_file(path.c_str(), &cfg, enc);
        if (r != MA_SUCCESS)
        {
            delete enc;
            return false;
        }
        ogg_ = enc;
        isOpen_ = true;
        return true;
    }

    // WAV path
    f_ = std::fopen(path.c_str(), "wb");
    if (!f_)
        return false;
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
    isOpen_ = true;
    return true;
}

void WavWriter::write(const int16_t* data, int count)
{
    if (!isOpen_ || count <= 0)
        return;

    if (fmt_ == RecordFormat::OGG)
    {
        if (ogg_)
        {
            int frames = count / channels_;
            ma_encoder_write_pcm_frames(ogg_, data, (ma_uint64)frames, nullptr);
        }
        return;
    }

    if (f_)
    {
        std::fwrite(data, sizeof(int16_t), (size_t)count, f_);
        dataBytes_ += (uint32_t)count * sizeof(int16_t);
    }
}

void WavWriter::close()
{
    if (!isOpen_)
        return;
    if (fmt_ == RecordFormat::OGG)
        closeOgg();
    else
        closeWav();
    isOpen_ = false;
}

void WavWriter::closeWav()
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

void WavWriter::closeOgg()
{
    if (ogg_)
    {
        ma_encoder_uninit(ogg_);
        delete ogg_;
        ogg_ = nullptr;
    }
}
