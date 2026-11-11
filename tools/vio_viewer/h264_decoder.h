// SPDX-License-Identifier: MIT
#pragma once

#include <wels/codec_api.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// H.264 decoder of OpenH264 (BSD). The AppImage has libopenh264.so.7. OpenH264 decodes the constrained baseline
// profile, which is the profile of the x264 stream of granite.
class H264Decoder {
 public:
  H264Decoder() {
    if (WelsCreateDecoder(&decoder) != 0 || !decoder) {
      decoder = nullptr;
      error = "OpenH264: cannot create the decoder";
      return;
    }
    SDecodingParam param{};
    param.sVideoProperty.eVideoBsType = VIDEO_BITSTREAM_AVC;
    // No error concealment: after an error, the viewer waits for the next IDR frame.
    param.eEcActiveIdc = ERROR_CON_DISABLE;
    if (decoder->Initialize(&param) != 0) {
      WelsDestroyDecoder(decoder);
      decoder = nullptr;
      error = "OpenH264: cannot initialize the decoder";
    }
  }
  ~H264Decoder() {
    if (!decoder) return;
    decoder->Uninitialize();
    WelsDestroyDecoder(decoder);
  }
  H264Decoder(const H264Decoder&) = delete;
  H264Decoder& operator=(const H264Decoder&) = delete;

  bool available() const { return decoder != nullptr; }
  const std::string& status() const { return error; }

  // One access unit of the stream. A frame after a lost message depends on the lost one: after a gap, the
  // decoder waits for the next keyframe. Gives the Y plane (the gray image) of a decoded frame.
  bool decode(const std::string& unit, bool keyframe, bool gap, std::vector<uint8_t>* pixels, int* width,
              int* height) {
    if (!available()) return false;
    if (gap) waiting_keyframe = true;
    if (waiting_keyframe && !keyframe) return false;
    waiting_keyframe = false;
    unsigned char* planes[3] = {nullptr, nullptr, nullptr};
    SBufferInfo info{};
    const DECODING_STATE state = decoder->DecodeFrameNoDelay(reinterpret_cast<const unsigned char*>(unit.data()),
                                                             static_cast<int>(unit.size()), planes, &info);
    if (state != dsErrorFree) {
      waiting_keyframe = true;
      return false;
    }
    if (info.iBufferStatus != 1 || !planes[0]) return false;
    const int w = info.UsrData.sSystemBuffer.iWidth;
    const int h = info.UsrData.sSystemBuffer.iHeight;
    const int stride = info.UsrData.sSystemBuffer.iStride[0];
    pixels->resize(static_cast<size_t>(w) * h);
    for (int y = 0; y < h; ++y)
      std::memcpy(pixels->data() + static_cast<size_t>(y) * w, planes[0] + static_cast<size_t>(y) * stride, w);
    *width = w;
    *height = h;
    return true;
  }

 private:
  ISVCDecoder* decoder = nullptr;
  std::string error;
  bool waiting_keyframe = true;
};
