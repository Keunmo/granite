// SPDX-License-Identifier: MIT
#pragma once

#include <dlfcn.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
}

// H.264 decoder of the libavcodec of the host (FFmpeg 6: libavcodec.so.60, libavutil.so.58). The viewer loads it
// at run time, so the AppImage runs without it and shows the JPEG images only.
class H264Decoder {
 public:
  H264Decoder() {
    avcodec = dlopen("libavcodec.so.60", RTLD_NOW | RTLD_LOCAL);
    avutil = dlopen("libavutil.so.58", RTLD_NOW | RTLD_LOCAL);
    if (!avcodec || !avutil) {
      error = "no libavcodec60 (apt install libavcodec60)";
      return;
    }
    bool ok = load(avcodec, "avcodec_find_decoder", &find_decoder) &&
              load(avcodec, "avcodec_alloc_context3", &alloc_context) && load(avcodec, "avcodec_open2", &open) &&
              load(avcodec, "avcodec_send_packet", &send_packet) &&
              load(avcodec, "avcodec_receive_frame", &receive_frame) &&
              load(avcodec, "avcodec_free_context", &free_context) && load(avcodec, "av_packet_alloc", &packet_alloc) &&
              load(avcodec, "av_packet_free", &packet_free) && load(avutil, "av_frame_alloc", &frame_alloc) &&
              load(avutil, "av_frame_free", &frame_free);
    if (!ok) {
      error = "libavcodec60 without the expected functions";
      return;
    }
    const AVCodec* codec = find_decoder(AV_CODEC_ID_H264);
    context = codec ? alloc_context(codec) : nullptr;
    if (!context || open(context, codec, nullptr) < 0) {
      error = "libavcodec60 has no H.264 decoder";
      return;
    }
    packet = packet_alloc();
    frame = frame_alloc();
  }
  ~H264Decoder() {
    if (frame) frame_free(&frame);
    if (packet) packet_free(&packet);
    if (context) free_context(&context);
    if (avutil) dlclose(avutil);
    if (avcodec) dlclose(avcodec);
  }
  H264Decoder(const H264Decoder&) = delete;
  H264Decoder& operator=(const H264Decoder&) = delete;

  bool available() const { return frame != nullptr; }
  const std::string& status() const { return error; }

  // One access unit of the stream. A frame after a lost message depends on the lost one: after a gap, the
  // decoder waits for the next keyframe. Gives the Y plane (the gray image) of a decoded frame.
  bool decode(const std::string& unit, bool keyframe, bool gap, std::vector<uint8_t>* pixels, int* width,
              int* height) {
    if (!available()) return false;
    if (gap) waiting_keyframe = true;
    if (waiting_keyframe && !keyframe) return false;
    waiting_keyframe = false;
    // libavcodec reads up to AV_INPUT_BUFFER_PADDING_SIZE bytes after the data.
    buffer.assign(unit.begin(), unit.end());
    buffer.resize(unit.size() + AV_INPUT_BUFFER_PADDING_SIZE, 0);
    packet->data = buffer.data();
    packet->size = static_cast<int>(unit.size());
    if (send_packet(context, packet) < 0) {
      waiting_keyframe = true;
      return false;
    }
    bool any = false;
    while (receive_frame(context, frame) == 0) {
      *width = frame->width;
      *height = frame->height;
      pixels->resize(static_cast<size_t>(frame->width) * frame->height);
      for (int y = 0; y < frame->height; ++y)
        std::memcpy(pixels->data() + static_cast<size_t>(y) * frame->width,
                    frame->data[0] + static_cast<size_t>(y) * frame->linesize[0], frame->width);
      any = true;
    }
    return any;
  }

 private:
  template <typename F>
  static bool load(void* library, const char* name, F* function) {
    *function = reinterpret_cast<F>(dlsym(library, name));
    return *function != nullptr;
  }

  void* avcodec = nullptr;
  void* avutil = nullptr;
  std::string error;
  decltype(&avcodec_find_decoder) find_decoder = nullptr;
  decltype(&avcodec_alloc_context3) alloc_context = nullptr;
  decltype(&avcodec_open2) open = nullptr;
  decltype(&avcodec_send_packet) send_packet = nullptr;
  decltype(&avcodec_receive_frame) receive_frame = nullptr;
  decltype(&avcodec_free_context) free_context = nullptr;
  decltype(&av_packet_alloc) packet_alloc = nullptr;
  decltype(&av_packet_free) packet_free = nullptr;
  decltype(&av_frame_alloc) frame_alloc = nullptr;
  decltype(&av_frame_free) frame_free = nullptr;
  AVCodecContext* context = nullptr;
  AVPacket* packet = nullptr;
  AVFrame* frame = nullptr;
  std::vector<uint8_t> buffer;
  bool waiting_keyframe = true;
};
