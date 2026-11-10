// SPDX-License-Identifier: MIT
#pragma once

#include <granite/io/dataset_io.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <granite/calibration/calibration.hpp>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace granite {

// CSI-2 RAW10 packs four pixels into five bytes. The fifth byte holds the
// two least significant bits, which an 8-bit PixelType does not keep.
inline cv::Mat unpackRaw10Msb(const uint8_t* data, int width, int height,
                              size_t stride) {
  if (width <= 0 || height <= 0 || width % 4 != 0 || stride < width / 4 * 5u)
    throw std::invalid_argument("Invalid RAW10 image geometry");
  cv::Mat bayer(height, width, CV_8UC1);
  for (int y = 0; y < height; ++y) {
    const uint8_t* source = data + y * stride;
    uint8_t* target = bayer.ptr<uint8_t>(y);
    for (int x = 0; x < width; x += 4, source += 5) {
      target[x] = source[0];
      target[x + 1] = source[1];
      target[x + 2] = source[2];
      target[x + 3] = source[3];
    }
  }
  return bayer;
}

// Converts a GRBG image to gray at 1/downscale of its size.
//  1: demosaics to luminance.
//  even N: averages N x N pixels. Each block holds whole 2x2 Bayer quads, so
//    every output pixel has the same color mix and demosaicing is not necessary.
//  odd N: averages each 2x2 quad, then resizes by 2/N with area weights. A
//    direct N x N average would mix the colors differently in neighbouring
//    blocks (5 G + 2 R + 2 B or 4 G + 4 R + 1 B for N = 3). That gives a
//    fixed pattern of one-pixel period that the optical flow would track.
// In all cases output pixel (u, v) covers the input pixels around
// (N (u + 0.5) - 0.5, N (v + 0.5) - 0.5), which is what scaleCalibration expects.
inline cv::Mat grbgToGray(const cv::Mat& bayer, int downscale) {
  if (downscale < 1) throw std::invalid_argument("The downscale factor must be positive");
  cv::Mat gray;
  if (downscale == 1) {
    // OpenCV names Bayer codes by the second row; GRBG sensors use BayerGB.
    cv::cvtColor(bayer, gray, cv::COLOR_BayerGB2GRAY);
    return gray;
  }
  const int unit = downscale % 2 == 0 ? downscale : 2 * downscale;
  if (bayer.cols % unit != 0 || bayer.rows % unit != 0)
    throw std::invalid_argument("The image size is not a multiple of the downscale block");
  const cv::Size size(bayer.cols / downscale, bayer.rows / downscale);
  if (downscale % 2 == 0) {
    cv::resize(bayer, gray, size, 0, 0, cv::INTER_AREA);
  } else {
    cv::Mat quads;
    cv::resize(bayer, quads, cv::Size(bayer.cols / 2, bayer.rows / 2), 0, 0, cv::INTER_AREA);
    cv::resize(quads, gray, size, 0, 0, cv::INTER_AREA);
  }
  return gray;
}

inline ManagedImage<PixelType>::Ptr copyGrayImage(const cv::Mat& gray) {
  auto image = std::make_shared<ManagedImage<PixelType>>(gray.cols, gray.rows);
  for (int y = 0; y < gray.rows; ++y) {
    convert_copy_gray_image(gray.cols, 1, gray.ptr<uint8_t>(y), image->RowPtr(y));
  }
  return image;
}

// Scales a calibration recorded at capture resolution to an image that is
// `factor` times smaller (grbgToGray). The focal lengths divide by the factor,
// and a principal point c moves to (c + 0.5) / factor - 0.5.
inline void scaleCalibration(Calibration<double>& calib, int factor) {
  if (factor < 1) throw std::invalid_argument("The calibration scale factor must be positive");
  if (factor == 1) return;
  for (size_t i = 0; i < calib.intrinsics.size(); ++i) {
    std::visit([factor](auto& camera) {
      using Camera = std::decay_t<decltype(camera)>;
      if constexpr (std::is_same_v<Camera, BalCamera<double>>) {
        throw std::invalid_argument("BAL camera intrinsics cannot be scaled");
      } else {
        auto params = camera.getParam();
        params.template head<2>() /= factor;
        params.template segment<2>(2) =
            (params.template segment<2>(2).array() + 0.5) / factor - 0.5;
        camera = Camera(params);
      }
    }, calib.intrinsics[i].variant);
    if (calib.resolution[i].x() % factor != 0 || calib.resolution[i].y() % factor != 0)
      throw std::invalid_argument("The calibration resolution is not a multiple of the factor");
    calib.resolution[i] /= factor;
  }
}

// Halves the pixel scale of a calibration recorded at capture resolution.
inline void scaleCalibrationHalf(Calibration<double>& calib) { scaleCalibration(calib, 2); }

// One MPU-6500/9250 FIFO sample with ACCEL and GYRO enabled: six big-endian
// accelerometer bytes, then six gyroscope bytes.
constexpr size_t MPU_FIFO_SAMPLE_BYTES = 12;

inline void decodeMpuFifoSample(const uint8_t* sample, double accel_scale,
                                double gyro_scale, Eigen::Vector3d& accel,
                                Eigen::Vector3d& gyro) {
  const auto value = [](const uint8_t* bytes) {
    return static_cast<int16_t>(static_cast<uint16_t>(bytes[0] << 8 | bytes[1]));
  };
  for (int i = 0; i < 3; ++i) {
    accel[i] = value(sample + 2 * i) * accel_scale;
    gyro[i] = value(sample + 6 + 2 * i) * gyro_scale;
  }
}

// Assigns host times to FIFO samples from their sample index. The FIFO count
// is read between two host times. The newest sample is no later than the
// end of that read and no earlier than one period before its start. The
// offset is the center of the tightest bounds in a recent window. The sensor
// oscillator is not exact: the period is the slope between the upper bound of
// the first window and that of the current window.
class FifoSampleClock {
 public:
  explicit FifoSampleClock(double nominal_period_ns,
                           int64_t window_ns = 500'000'000)
      : nominal_period(nominal_period_ns), period(nominal_period_ns),
        window(window_ns) {
    if (!(nominal_period_ns > 0) || window_ns <= 0)
      throw std::invalid_argument("Invalid FIFO clock period or window");
  }

  void reset() {
    period = nominal_period;
    drains.clear();
    next_index = 0;
    first_host = 0;
    anchor_index = 0;
    anchor_bound = 0;
    // Keep last_ns: the times after a reset stay later than the times before
    // it, so that the IMU stream stays in time order.
    last_time = 0;
    started = false;
    anchored = false;
    have_time = false;
  }

  // before_ns, after_ns: host times around the FIFO count read. Returns the
  // times of `count` new samples, oldest first.
  std::vector<int64_t> assign(int64_t before_ns, int64_t after_ns, size_t count) {
    std::vector<int64_t> times;
    if (count == 0) return times;
    if (after_ns < before_ns) throw std::invalid_argument("FIFO read ends before it starts");
    const uint64_t newest = next_index + count - 1;
    if (!started) {
      started = true;
      first_host = after_ns;
    }
    drains.push_back({before_ns, after_ns, newest});
    while (drains.size() > 1 && after_ns - drains.front().after > window)
      drains.pop_front();
    // Lowest upper bound of the window for a given period.
    const auto upper = [&](double slope) {
      std::pair<double, uint64_t> result{std::numeric_limits<double>::max(), 0};
      for (const auto& drain : drains) {
        const double bound = drain.after - slope * drain.newest;
        if (bound < result.first) result = {bound, drain.newest};
      }
      return result;
    };
    const auto nominal_bound = upper(nominal_period);
    if (!anchored && after_ns - first_host >= window) {
      anchored = true;
      anchor_bound = nominal_bound.first;
      anchor_index = nominal_bound.second;
    } else if (anchored && nominal_bound.second > anchor_index + 100) {
      const double slope = nominal_period + (nominal_bound.first - anchor_bound) /
                                                (nominal_bound.second - anchor_index);
      period = std::clamp(slope, nominal_period * 0.95, nominal_period * 1.05);
    }
    const double upper_offset = upper(period).first;
    double lower_offset = std::numeric_limits<double>::lowest();
    for (const auto& drain : drains)
      lower_offset = std::max(lower_offset, drain.before - period * (drain.newest + 1));
    const double offset = lower_offset <= upper_offset
                              ? 0.5 * (lower_offset + upper_offset) : upper_offset;
    // Continue from the previous sample and slew small corrections, so
    // preintegration intervals stay close to the period. A period update
    // must not move the whole time line. Jump only after a large error.
    const double target = offset + period * next_index;
    const double max_step = 0.005 * period * count;
    double first = target;
    if (have_time && std::abs(target - (last_time + period)) <= period)
      first = last_time + period +
              std::clamp(target - (last_time + period), -max_step, max_step);
    times.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      last_time = first + period * i;
      int64_t t_ns = static_cast<int64_t>(std::llround(last_time));
      if (last_ns != 0 && t_ns <= last_ns) t_ns = last_ns + 1;
      times.push_back(t_ns);
      last_ns = t_ns;
    }
    have_time = true;
    next_index = newest + 1;
    return times;
  }

  double periodNs() const { return period; }

 private:
  struct Drain {
    int64_t before, after;
    uint64_t newest;
  };
  double nominal_period, period;
  int64_t window;
  std::deque<Drain> drains;
  uint64_t next_index = 0, anchor_index = 0;
  double anchor_bound = 0, last_time = 0;
  int64_t first_host = 0, last_ns = 0;
  bool started = false, anchored = false, have_time = false;
};

// The B0429 bridge takes analogue_gain as about 100 + 382 x (linear gain - 1).
// The sensor applies the gain in steps of 0.3-0.9 dB, and values from 1123
// have no effect. The fit comes from gain steps that xeye measured with the
// Jetson driver of the same module; the offset puts a dB value in the middle
// of its step.
constexpr double B0429_DRIVER_GAIN_PER_X = 382.0;
constexpr double B0429_DRIVER_GAIN_OFFSET_DB = 0.32;
constexpr int B0429_MIN_DRIVER_GAIN = 100;
constexpr int B0429_MAX_DRIVER_GAIN = 1122;
constexpr double B0429_MAX_GAIN_DB = 11.0;

inline int b0429DriverGainFromDb(double gain_db) {
  if (gain_db <= 0) return B0429_MIN_DRIVER_GAIN;
  const double linear = std::pow(10.0, (gain_db + B0429_DRIVER_GAIN_OFFSET_DB) / 20);
  return std::clamp(static_cast<int>(std::lround(B0429_MIN_DRIVER_GAIN +
                                                 B0429_DRIVER_GAIN_PER_X * (linear - 1))),
                    B0429_MIN_DRIVER_GAIN, B0429_MAX_DRIVER_GAIN);
}

inline double b0429DbFromDriverGain(int value) {
  const double linear = 1 + (value - B0429_MIN_DRIVER_GAIN) / B0429_DRIVER_GAIN_PER_X;
  return std::max(0.0, 20 * std::log10(linear) - B0429_DRIVER_GAIN_OFFSET_DB);
}

struct ExposureSetting {
  int lines = 0;
  int gain = 0;
};

// Moves exposure x gain toward the target mean brightness. Exposure grows
// first up to the motion-blur limit; analogue gain covers the rest.
inline ExposureSetting nextAutoExposure(const ExposureSetting& current,
                                        double mean, double target,
                                        int max_lines, int min_gain,
                                        int max_gain) {
  const double ratio = target / std::max(mean, 1.0);
  if (ratio > 0.92 && ratio < 1.08) return current;
  // A square root step damps the two-frame control latency of the sensor.
  const double total = static_cast<double>(current.lines) * current.gain *
                       std::sqrt(std::clamp(ratio, 0.25, 4.0));
  ExposureSetting next;
  next.lines = static_cast<int>(std::clamp(total / min_gain, 2.0,
                                           static_cast<double>(max_lines)));
  next.gain = static_cast<int>(std::clamp(total / next.lines,
                                          static_cast<double>(min_gain),
                                          static_cast<double>(max_gain)));
  return next;
}

// The xeye capture node kurokesu_ar0234_pi (multipleye cross-pi5, capturelib
// camera_v4l2_arducam_b0429_pi.cc and imu_mavlink.cc) processes the frames and
// the IMU samples with the functions below. The live device uses the same
// functions, so that its images and times are the same as those of an xeye
// capture.

// The Kurokesu AR0234 driver writes analogue_gain to R0x3060: the coarse gain
// 2^(value / 16) and the fine gain 32 / (32 - value % 16), from the AR0234
// register reference. Not measured.
constexpr int KUROKESU_MIN_GAIN_VALUE = 13;
constexpr int KUROKESU_MAX_GAIN_VALUE = 64;
// The pixel_rate and horizontal_blanking controls of the Kurokesu driver do
// not give the line time. 13.6 us is measured in all four modes (2 lanes).
constexpr double KUROKESU_LINE_TIME_NS = 13600;

inline double kurokesuLinearGain(int value) {
  return std::pow(2.0, value / 16) * 32.0 / (32 - value % 16);
}

inline int kurokesuGainValue(double linear_gain) {
  int best = KUROKESU_MIN_GAIN_VALUE;
  for (int value = KUROKESU_MIN_GAIN_VALUE; value <= KUROKESU_MAX_GAIN_VALUE;
       ++value) {
    if (std::abs(std::log(kurokesuLinearGain(value) / linear_gain)) <
        std::abs(std::log(kurokesuLinearGain(best) / linear_gain)))
      best = value;
  }
  return best;
}

// The rp1-cfe writes RAW10 as 16-bit words with the value in the upper 10
// bits (GR16). A pixel reads its LUT with the upper 10 bits.
constexpr int BAYER10L_LUT_SHIFT = 6;
constexpr int BAYER10_MAX = 1023;
// One of every four Bayer row pairs gives the WB statistics.
constexpr int WB_STATS_ROW_PAIR_STEP = 4;
constexpr int GAMMA_CURVE_SIZE = 1 << 16;

// greens, rbs0, rbs1 of the 10-bit values minus the black level.
using WbStats = std::array<int64_t, 3>;

// The reduction of the 10-bit values to 8 bits, with the WB gains: [0] the
// green pixels, [1] and [2] the other pixels of the even and odd rows.
using Bayer8Lut = std::array<std::array<uint8_t, BAYER10_MAX + 1>, 3>;

// curve[k] is the 8-bit output for the linear value k / (GAMMA_CURVE_SIZE - 1).
inline std::vector<uint8_t> buildGammaCurve(double gamma_value) {
  std::vector<uint8_t> curve(GAMMA_CURVE_SIZE);
  for (int k = 0; k < GAMMA_CURVE_SIZE; ++k) {
    const double t = static_cast<double>(k) / (GAMMA_CURVE_SIZE - 1);
    curve[k] =
        static_cast<uint8_t>(std::lround(255.0 * std::pow(t, gamma_value)));
  }
  return curve;
}

// The inverse of the gamma curve without the WB gain: linear10[o] is the
// 10-bit value, with the black level, that gives the output o. The AE mean of
// the output is then linear, as without gamma.
inline std::array<uint16_t, 256> buildGammaLinear10(double gamma_value,
                                                    int black) {
  std::array<uint16_t, 256> linear10;
  for (int o = 0; o < 256; ++o) {
    const double t = std::pow(o / 255.0, 1 / gamma_value);
    linear10[o] =
        static_cast<uint16_t>(std::lround(black + t * (BAYER10_MAX - black)));
  }
  return linear10;
}

// The WB gain applies to the signal above the black level. An empty curve
// gives x >> 2 after the gain, so the black level stays at black / 4.
inline void buildBayer8Lut(const std::vector<uint8_t>& curve,
                           const std::array<double, 2>& gains, int black,
                           Bayer8Lut* lut) {
  const std::array<double, 3> channel_gains{1.0, gains[0], gains[1]};
  const double range = BAYER10_MAX - black;
  for (size_t c = 0; c < 3; ++c) {
    for (int v = 0; v <= BAYER10_MAX; ++v) {
      if (curve.empty()) {
        const double v10 = black + (v - black) * channel_gains[c];
        (*lut)[c][v] = static_cast<uint8_t>(std::clamp(v10 / 4, 0.0, 255.0));
        continue;
      }
      const double t = (v - black) * channel_gains[c] / range;
      const long k =
          std::lround(std::clamp(t, 0.0, 1.0) * (GAMMA_CURVE_SIZE - 1));
      (*lut)[c][v] = curve[k];
    }
  }
}

// non_green_col: the column of the non-green pixel in the even rows (1 for
// GRBG). A row sum is at most 1920 x 1023, so 32 bits hold it.
inline void accumulateWbStats(const uint16_t* row, int w, int y,
                              int non_green_col, int black, WbStats* stats) {
  const int paired_w = w - w % 2;
  uint32_t sums[2] = {0, 0};
  for (int x = 0; x < paired_w; x += 2) {
    sums[0] += row[x] >> BAYER10L_LUT_SHIFT;
    sums[1] += row[x + 1] >> BAYER10L_LUT_SHIFT;
  }
  const int64_t black_sum = static_cast<int64_t>(black) * (paired_w / 2);
  const int col = y % 2 == 0 ? non_green_col : 1 - non_green_col;
  (*stats)[0] += sums[1 - col] - black_sum;
  (*stats)[1 + y % 2] += sums[col] - black_sum;
}

// The functions below read each row of the V4L2 buffer with memcpy first: a
// read of one pixel at a time is slow when the buffer is not CPU cached.
// src_stride is in pixels.

// The statistics of one of every WB_STATS_ROW_PAIR_STEP row pairs.
inline WbStats accumulateBayer10LStats(const uint16_t* src, int src_stride,
                                       int w, int h, int non_green_col,
                                       int black) {
  std::vector<uint16_t> src_row(w);
  WbStats stats{0, 0, 0};
  for (int y = 0; y < h; ++y) {
    if ((y / 2) % WB_STATS_ROW_PAIR_STEP != 0) continue;
    std::memcpy(src_row.data(), src + static_cast<size_t>(y) * src_stride,
                w * sizeof(uint16_t));
    accumulateWbStats(src_row.data(), w, y, non_green_col, black, &stats);
  }
  return stats;
}

// 8 bits without WB and gamma: the upper byte of each word, x >> 2 of the
// 10-bit value. The loop reads whole vectors, so it needs no memcpy.
inline void copyBayer10LShift8(const uint16_t* src, int src_stride, int w,
                               int h, uint8_t* dst) {
  for (int y = 0; y < h; ++y) {
    const uint16_t* src_row = src + static_cast<size_t>(y) * src_stride;
    uint8_t* row = dst + static_cast<size_t>(y) * w;
    for (int x = 0; x < w; ++x) row[x] = static_cast<uint8_t>(src_row[x] >> 8);
  }
}

// Reduces a 10-bit Bayer frame to 8 bits through lut. When stats is not null,
// it also collects the statistics of accumulateBayer10LStats() from the rows.
inline void copyBayer10LTo8(const uint16_t* src, int src_stride, int w, int h,
                            int non_green_col, const Bayer8Lut& lut, int black,
                            WbStats* stats, uint8_t* dst) {
  static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
                "the 64-bit load below keeps the first pixel in the low bits");
  std::vector<uint16_t> src_row(w);
  for (int y = 0; y < h; ++y) {
    std::memcpy(src_row.data(), src + static_cast<size_t>(y) * src_stride,
                w * sizeof(uint16_t));
    const uint16_t* in = src_row.data();
    uint8_t* row = dst + static_cast<size_t>(y) * w;
    const int col = y % 2 == 0 ? non_green_col : 1 - non_green_col;
    const uint8_t* non_green = lut[1 + y % 2].data();
    const uint8_t* even = col == 0 ? non_green : lut[0].data();
    const uint8_t* odd = col == 1 ? non_green : lut[0].data();
    // Four pixels with one 64-bit load and one 32-bit store, in place of four
    // loads and four stores. Pixel k is bits [16k + 6, 16k + 16) of the load,
    // and byte k of the store.
    int x = 0;
    for (; x + 4 <= w; x += 4) {
      uint64_t p;
      std::memcpy(&p, in + x, sizeof(p));
      const uint32_t out = uint32_t{even[(p >> 6) & BAYER10_MAX]} |
                           uint32_t{odd[(p >> 22) & BAYER10_MAX]} << 8 |
                           uint32_t{even[(p >> 38) & BAYER10_MAX]} << 16 |
                           uint32_t{odd[(p >> 54) & BAYER10_MAX]} << 24;
      std::memcpy(row + x, &out, sizeof(out));
    }
    for (; x < w; ++x)
      row[x] = (x % 2 == 0 ? even : odd)[in[x] >> BAYER10L_LUT_SHIFT];
    if (stats != nullptr && (y / 2) % WB_STATS_ROW_PAIR_STEP == 0)
      accumulateWbStats(in, w, y, non_green_col, black, stats);
  }
}

// The WB gains of the non-green pixels of the even and odd rows. The pixel
// counts of the three sums are 2 : 1 : 1.
inline std::optional<std::array<double, 2>> gainsFromWbStats(
    const WbStats& stats) {
  const auto [greens, rbs0, rbs1] = stats;
  if (greens <= 0 || rbs0 <= 0 || rbs1 <= 0) return std::nullopt;
  return std::array<double, 2>{0.5 * greens / static_cast<double>(rbs0),
                               0.5 * greens / static_cast<double>(rbs1)};
}

// The gray of the xcv DemosaicAndToGray8 of xeye in one pass, without the RGB
// image: the mean of R, G, and B of the bilinear demosaic of xcv
// DemosaicBayer, with the same integer divisions, so the output is the same.
// The border pixels keep their values, as there.
inline void bayer8ToGray8(const uint8_t* src, int w, int h, int non_green_col,
                          uint8_t* dst) {
  std::memcpy(dst, src, w);
  std::memcpy(dst + static_cast<size_t>(h - 1) * w,
              src + static_cast<size_t>(h - 1) * w, w);
  const int green_col = 1 - non_green_col;
  for (int y = 1; y < h - 1; ++y) {
    const uint8_t* up = src + static_cast<size_t>(y - 1) * w;
    const uint8_t* mid = up + w;
    const uint8_t* down = mid + w;
    uint8_t* out = dst + static_cast<size_t>(y) * w;
    out[0] = mid[0];
    out[w - 1] = mid[w - 1];
    const int green_x = (green_col + y) % 2;
    for (int x = 1; x < w - 1; ++x) {
      const int c = mid[x];
      const int ns = up[x] + down[x];
      const int we = mid[x - 1] + mid[x + 1];
      if (x % 2 == green_x) {
        out[x] = static_cast<uint8_t>((c + we / 2 + ns / 2) / 3);
      } else {
        const int diag = up[x - 1] + up[x + 1] + down[x - 1] + down[x + 1];
        out[x] = static_cast<uint8_t>((c + (ns + we) / 4 + diag / 4) / 3);
      }
    }
  }
}

// The automatic exposure of the xeye node (UpdateExposureGain of
// multi_camera_v4l2_arducam_b0429): the keys and their meaning are the same.
struct XeyeAutoExposureOptions {
  bool auto_exposure = true;
  bool auto_gain = true;
  std::vector<double> exposure_range_ms = {0.1, 10.0};
  std::vector<double>
      gain_range_db;  // Of the linear gain; empty: the driver range
  std::string prior =
      "exposure";  // "exposure" or "gain": the one that changes first
  double target_gray_value = 20.0;  // [%] of the full scale, linear
  double ev_compensation = 0.0;     // [-3, 3], target x 2^ev
  double damping = 0.5;             // [0, 1)
  std::string metering =
      "average";  // "average", "spot" (center 3 %), or "partial" (center 11 %)
  std::vector<int> roi;     // [x, y, w, h]; replaces metering
  std::vector<int> circle;  // [cx, cy, r]; only the pixels in it count
};

// One sample is a 2x2 cell at every 8th row and column.
constexpr int AE_SAMPLE_STEP = 8;
// No change while the mean is within about 10 % of the target.
constexpr double AE_DEADBAND = 0.1;
// One update changes the brightness by at most 2x.
constexpr double AE_MAX_LOG_STEP = 0.693;
// A new exposure or gain shows in a later frame.
constexpr int AE_SETTLE_FRAMES = 2;

struct PixelRegion {
  int x = 0, y = 0, w = 0, h = 0;
};

// The region has even x, y, w, and h, so that each 2x2 cell has all Bayer
// colors.
inline PixelRegion autoExposureRegion(const XeyeAutoExposureOptions& ae,
                                      int width, int height) {
  const auto floor2 = [](int v) { return v - v % 2; };
  if (ae.roi.size() == 4)
    return PixelRegion{floor2(ae.roi[0]), floor2(ae.roi[1]), floor2(ae.roi[2]),
                       floor2(ae.roi[3])};
  double side = 1.0;
  if (ae.metering == "spot") side = 0.173;
  if (ae.metering == "partial") side = 0.332;
  const int w = std::max(2, floor2(static_cast<int>(width * side)));
  const int h = std::max(2, floor2(static_cast<int>(height * side)));
  return PixelRegion{floor2((width - w) / 2), floor2((height - h) / 2), w, h};
}

// The mean of a region in percent of the full scale. A circle {cx, cy, r}
// removes the samples outside it. linear(v) is the linear value of a pixel v,
// from 0 to full_scale.
template <typename T, typename Linear>
double meanPercent(const T* img, int stride, const PixelRegion& region,
                   const std::vector<int>& circle, double full_scale,
                   Linear linear) {
  uint64_t sum = 0, num = 0;
  for (int y = region.y; y + 1 < region.y + region.h; y += AE_SAMPLE_STEP) {
    int x_begin = region.x;
    int x_end = region.x + region.w;
    if (circle.size() == 3) {
      const double dy = y + 1 - circle[1];
      const double r2 = static_cast<double>(circle[2]) * circle[2] - dy * dy;
      if (r2 <= 0.0) continue;
      const double half = std::sqrt(r2);
      int x_left = static_cast<int>(std::ceil(circle[0] - half));
      if (x_left % 2 != 0) ++x_left;
      x_begin = std::max(x_begin, x_left);
      x_end = std::min(x_end, static_cast<int>(circle[0] + half));
    }
    const T* row0 = img + static_cast<size_t>(y) * stride;
    const T* row1 = row0 + stride;
    for (int x = x_begin; x + 1 < x_end; x += AE_SAMPLE_STEP) {
      sum += linear(row0[x]) + linear(row0[x + 1]) + linear(row1[x]) +
             linear(row1[x + 1]);
      num += 4;
    }
  }
  return num > 0 ? 100.0 * sum / (full_scale * num) : 0.0;
}

// The next exposure [ms] and linear gain, or nullopt when the mean is in the
// dead band. exposure x gain (linear) is the brightness. The first control
// takes the change with the second control at its minimum. The second control
// takes the rest.
inline std::optional<std::pair<double, double>> nextXeyeExposure(
    const XeyeAutoExposureOptions& ae, double gray_percent, double exposure_ms,
    double gain) {
  const double target_percent =
      std::min(99.0, ae.target_gray_value * std::exp2(ae.ev_compensation));
  const double error = std::log(target_percent / std::max(gray_percent, 0.1));
  if (std::abs(error) < AE_DEADBAND) return std::nullopt;
  const auto from_db = [](double db) { return std::pow(10.0, db / 20.0); };
  const double min_exposure =
      ae.auto_exposure ? ae.exposure_range_ms[0] : exposure_ms;
  const double max_exposure =
      ae.auto_exposure ? ae.exposure_range_ms[1] : exposure_ms;
  const double min_gain = ae.auto_gain ? from_db(ae.gain_range_db[0]) : gain;
  const double max_gain = ae.auto_gain ? from_db(ae.gain_range_db[1]) : gain;
  const double step =
      std::clamp((1.0 - ae.damping) * error, -AE_MAX_LOG_STEP, AE_MAX_LOG_STEP);
  const double target = exposure_ms * gain * std::exp(step);
  double new_exposure = 0, new_gain = 0;
  if (ae.prior == "exposure") {
    new_exposure = std::clamp(target / min_gain, min_exposure, max_exposure);
    new_gain = std::clamp(target / new_exposure, min_gain, max_gain);
  } else {
    new_gain = std::clamp(target / min_exposure, min_gain, max_gain);
    new_exposure = std::clamp(target / new_gain, min_exposure, max_exposure);
  }
  return std::make_pair(new_exposure, new_gain);
}

// Maps FC sample times to strictly increasing host times, as the xeye
// imu_mavlink node. The smallest (receive time - sample time) within the
// window approximates the transport latency. The window is split into buckets
// so the estimate follows the FC/host clock drift. The applied offset moves
// toward that estimate by at most MAX_SLEW of each FC sample interval, so a
// step of the estimate changes the published intervals by at most that ratio.
// The samples of the first bucket only warm the estimate up.
class FcClockMapper {
 public:
  // True when fc_ns went back far enough to mean an FC reboot. reset() then
  // starts the mapping over.
  bool isFcReboot(int64_t fc_ns) const {
    return last_fc_ns >= 0 && fc_ns + FC_REBOOT_JUMP_NS < last_fc_ns;
  }

  // The host time of the FC sample time fc_ns received at host time rx_ns, or
  // nullopt for a sample at or before the previous one or within the warm-up.
  std::optional<int64_t> toHostNs(int64_t fc_ns, int64_t rx_ns) {
    if (fc_ns <= last_fc_ns) return std::nullopt;
    const int64_t fc_interval_ns = fc_ns - last_fc_ns;
    last_fc_ns = fc_ns;
    if (first_rx_ns < 0) first_rx_ns = rx_ns;
    const int64_t offset_ns = rx_ns - fc_ns;
    if (buckets.empty() || rx_ns - buckets.back().start_rx_ns >= BUCKET_NS) {
      buckets.push_back({rx_ns, offset_ns});
      if (buckets.size() > NUM_BUCKETS) buckets.pop_front();
    } else {
      buckets.back().min_offset_ns =
          std::min(buckets.back().min_offset_ns, offset_ns);
    }
    if (rx_ns - first_rx_ns < BUCKET_NS) return std::nullopt;
    int64_t min_offset_ns = offset_ns;
    for (const Bucket& bucket : buckets)
      min_offset_ns = std::min(min_offset_ns, bucket.min_offset_ns);
    if (!applied_offset_ns) {
      applied_offset_ns = min_offset_ns;
    } else {
      // The FC interval is at least 1 us, so the host time keeps increasing.
      const auto max_step_ns =
          static_cast<int64_t>(static_cast<double>(fc_interval_ns) * MAX_SLEW);
      *applied_offset_ns += std::clamp(min_offset_ns - *applied_offset_ns,
                                       -max_step_ns, max_step_ns);
    }
    return fc_ns + *applied_offset_ns;
  }

  void reset() {
    buckets.clear();
    first_rx_ns = -1;
    last_fc_ns = -1;
    applied_offset_ns.reset();
  }

 private:
  static constexpr int64_t BUCKET_NS = 1'000'000'000;
  static constexpr size_t NUM_BUCKETS = 5;
  // Far above the FC/host clock drift (< 1e-4); a 5 ms latency step settles
  // in 0.5 s at 200 Hz.
  static constexpr double MAX_SLEW = 0.01;
  static constexpr int64_t FC_REBOOT_JUMP_NS = 1'000'000'000;

  struct Bucket {
    int64_t start_rx_ns;
    int64_t min_offset_ns;
  };
  std::deque<Bucket> buckets;
  int64_t first_rx_ns = -1;
  int64_t last_fc_ns = -1;
  std::optional<int64_t> applied_offset_ns;
};

}  // namespace granite
