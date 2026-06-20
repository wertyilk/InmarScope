#include "gui/waterfall.h"

#if defined(_WIN32)
#include <windows.h>
#endif
#include <GL/gl.h>

#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

#include <algorithm>
#include <cmath>

Waterfall::~Waterfall()
{
    if (tex_)
        glDeleteTextures(1, &tex_);
}

void Waterfall::ensureTexture(int bins)
{
    if (tex_ && bins == texW_)
        return;

    if (tex_)
    {
        glDeleteTextures(1, &tex_);
        tex_ = 0;
    }

    texW_ = bins;
    writePtr_ = 0;
    filled_ = false;

    std::vector<uint8_t> zeros((size_t)texW_ * texH_ * 4, 0);
    glGenTextures(1, &tex_);
    glBindTexture(GL_TEXTURE_2D, tex_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texW_, texH_, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, zeros.data());
}

void Waterfall::colormap(float t, uint8_t& r, uint8_t& g, uint8_t& b)
{
    // Smooth black -> blue -> cyan -> green -> yellow -> red -> white.
    t = std::clamp(t, 0.0f, 1.0f);
    static const float stops[7][3] = {
        {0.00f, 0.00f, 0.00f}, // black
        {0.00f, 0.00f, 0.55f}, // dark blue
        {0.00f, 0.55f, 0.90f}, // cyan
        {0.00f, 0.85f, 0.20f}, // green
        {0.95f, 0.90f, 0.00f}, // yellow
        {0.95f, 0.20f, 0.00f}, // red
        {1.00f, 1.00f, 1.00f}, // white
    };
    float x = t * 6.0f;
    int i = (int)x;
    if (i > 5) i = 5;
    float f = x - i;
    float rr = stops[i][0] + (stops[i + 1][0] - stops[i][0]) * f;
    float gg = stops[i][1] + (stops[i + 1][1] - stops[i][1]) * f;
    float bb = stops[i][2] + (stops[i + 1][2] - stops[i][2]) * f;
    r = (uint8_t)std::lround(rr * 255.0f);
    g = (uint8_t)std::lround(gg * 255.0f);
    b = (uint8_t)std::lround(bb * 255.0f);
}

void Waterfall::addRow(const float* db, int n, float dbMin, float dbMax)
{
    if (n <= 0)
        return;
    ensureTexture(n);

    rowBuf_.resize((size_t)texW_ * 4);
    float span = (dbMax > dbMin) ? (dbMax - dbMin) : 1.0f;
    for (int i = 0; i < texW_; ++i)
    {
        float t = (db[i] - dbMin) / span;
        uint8_t r, g, b;
        colormap(t, r, g, b);
        rowBuf_[i * 4 + 0] = r;
        rowBuf_[i * 4 + 1] = g;
        rowBuf_[i * 4 + 2] = b;
        rowBuf_[i * 4 + 3] = 255;
    }

    // Newest row goes to the (decremented) write pointer so memory index
    // increasing == older, which maps cleanly to a top-to-bottom display.
    writePtr_ = (writePtr_ - 1 + texH_) % texH_;
    glBindTexture(GL_TEXTURE_2D, tex_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, writePtr_, texW_, 1, GL_RGBA,
                    GL_UNSIGNED_BYTE, rowBuf_.data());
    if (writePtr_ == 0)
        filled_ = true;
}

void Waterfall::draw(const ImVec2& size, float uMin, float uMax)
{
    if (!tex_ || size.x <= 1.0f || size.y <= 1.0f)
    {
        ImGui::Dummy(size);
        return;
    }

    if (uMax <= uMin)
    {
        uMin = 0.0f;
        uMax = 1.0f;
    }

    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImGui::InvisibleButton("##waterfall", size);

    float H = (float)texH_;
    float vSplit = (float)writePtr_ / H; // top slice covers rows [writePtr_..H]
    float topFrac = 1.0f - vSplit;       // fraction of height for top slice

    ImTextureID tid = (ImTextureID)(intptr_t)tex_;

    // Top slice: tex rows [writePtr_ .. texH_-1].
    ImVec2 a0 = p0;
    ImVec2 a1 = ImVec2(p0.x + size.x, p0.y + size.y * topFrac);
    dl->AddImage(tid, a0, a1, ImVec2(uMin, vSplit), ImVec2(uMax, 1.0f));

    // Bottom slice: tex rows [0 .. writePtr_-1].
    if (vSplit > 0.0f)
    {
        ImVec2 b0 = ImVec2(p0.x, p0.y + size.y * topFrac);
        ImVec2 b1 = ImVec2(p0.x + size.x, p0.y + size.y);
        dl->AddImage(tid, b0, b1, ImVec2(uMin, 0.0f), ImVec2(uMax, vSplit));
    }
}

void Waterfall::clear()
{
    if (tex_)
    {
        glDeleteTextures(1, &tex_);
        tex_ = 0;
    }
    texW_ = 0;
    writePtr_ = 0;
    filled_ = false;
}
