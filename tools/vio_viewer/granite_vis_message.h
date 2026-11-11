// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// GraniteVis of doc/granite_vis.proto (the visualization message of the Granite VIO, send_vis_msg).
struct GraniteVisMessage {
  uint64_t seq = 0;
  int64_t t_ns = 0;
  uint32_t map_idx = 0;
  std::vector<float> pose;           // [px, py, pz, qw, qx, qy, qz]
  std::vector<float> velocity;       // [m/s]
  std::vector<float> bias_gyro;      // [rad/s]
  std::vector<float> bias_accel;     // [m/s^2]
  bool healthy = false;
  bool keyframe = false;
  std::vector<float> window_states;  // 7 floats for each pose
  std::vector<float> window_frames;  // 7 floats for each pose
  std::vector<float> points;         // [x, y, z] for each landmark
  std::vector<float> projections;    // [u, v, inverse distance] for each projection
  std::vector<uint32_t> projection_ids;
  std::vector<float> keypoints;      // [u, v] for each keypoint
  std::string image_jpeg;
  uint32_t image_width = 0;
  uint32_t image_height = 0;
  uint64_t skipped = 0;
};

// Reads the protobuf wire format of GraniteVis: a key varint (field << 3 | wire type) and a value for each field.
// Unknown fields are skipped. Returns false for a truncated or malformed message.
inline bool parseGraniteVis(const uint8_t* data, size_t size, GraniteVisMessage* out) {
  size_t i = 0;
  const auto varint = [&](uint64_t* value) {
    *value = 0;
    for (int shift = 0; shift < 64; shift += 7) {
      if (i >= size) return false;
      const uint8_t b = data[i++];
      *value |= static_cast<uint64_t>(b & 0x7f) << shift;
      if (b < 0x80) return true;
    }
    return false;
  };
  const auto floats = [](const uint8_t* p, size_t n, std::vector<float>* v) {
    v->resize(n / sizeof(float));
    std::memcpy(v->data(), p, v->size() * sizeof(float));  // little-endian on both ends
  };
  *out = GraniteVisMessage();
  while (i < size) {
    uint64_t key = 0;
    if (!varint(&key)) return false;
    const int field = static_cast<int>(key >> 3);
    const int wire = static_cast<int>(key & 7);
    uint64_t value = 0;
    const uint8_t* bytes = nullptr;
    size_t length = 0;
    if (wire == 0) {
      if (!varint(&value)) return false;
    } else if (wire == 2) {
      if (!varint(&value) || value > size - i) return false;
      bytes = data + i;
      length = static_cast<size_t>(value);
      i += length;
    } else if (wire == 5 || wire == 1) {
      const size_t n = wire == 5 ? 4 : 8;
      if (n > size - i) return false;
      i += n;
      continue;
    } else {
      return false;
    }
    switch (field) {
      case 2: out->seq = value; break;
      case 3: out->t_ns = static_cast<int64_t>(value); break;
      case 4: out->map_idx = static_cast<uint32_t>(value); break;
      case 5: floats(bytes, length, &out->pose); break;
      case 6: floats(bytes, length, &out->velocity); break;
      case 7: floats(bytes, length, &out->bias_gyro); break;
      case 8: floats(bytes, length, &out->bias_accel); break;
      case 9: out->healthy = value != 0; break;
      case 10: out->keyframe = value != 0; break;
      case 11: floats(bytes, length, &out->window_states); break;
      case 12: floats(bytes, length, &out->window_frames); break;
      case 13: floats(bytes, length, &out->points); break;
      case 14: floats(bytes, length, &out->projections); break;
      case 15: floats(bytes, length, &out->keypoints); break;
      case 16: out->image_jpeg.assign(reinterpret_cast<const char*>(bytes), length); break;
      case 17: out->image_width = static_cast<uint32_t>(value); break;
      case 18: out->image_height = static_cast<uint32_t>(value); break;
      case 19: out->skipped = value; break;
      case 20: {
        // Packed varints
        const size_t end = i;
        i = static_cast<size_t>(bytes - data);
        while (i < end) {
          uint64_t id = 0;
          if (!varint(&id)) return false;
          out->projection_ids.push_back(static_cast<uint32_t>(id));
        }
        if (i != end) return false;
        break;
      }
      default: break;
    }
  }
  return true;
}
