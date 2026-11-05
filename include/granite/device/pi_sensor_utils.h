// SPDX-License-Identifier: MIT
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <granite/calibration/calibration.hpp>
#include <granite/io/dataset_io.h>

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

// Full resolution demosaics GRBG to luminance. Half resolution averages each
// 2x2 Bayer quad, so it does not need demosaicing.
inline cv::Mat grbgToGray(const cv::Mat& bayer, bool resize_half) {
  cv::Mat gray;
  if (resize_half) {
    cv::resize(bayer, gray, cv::Size(bayer.cols / 2, bayer.rows / 2), 0, 0,
               cv::INTER_AREA);
  } else {
    // OpenCV names Bayer codes by the second row; GRBG sensors use BayerGB.
    cv::cvtColor(bayer, gray, cv::COLOR_BayerGB2GRAY);
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

// Halves the pixel scale of a calibration recorded at capture resolution.
// INTER_AREA maps the center of each 2x2 block to one output pixel.
inline void scaleCalibrationHalf(Calibration<double>& calib) {
  for (size_t i = 0; i < calib.intrinsics.size(); ++i) {
    std::visit([](auto& camera) {
      using Camera = std::decay_t<decltype(camera)>;
      if constexpr (std::is_same_v<Camera, BalCamera<double>>) {
        throw std::invalid_argument("BAL camera intrinsics cannot be scaled");
      } else {
        auto params = camera.getParam();
        params.template head<2>() *= 0.5;
        params.template segment<2>(2) =
            (params.template segment<2>(2).array() + 0.5) * 0.5 - 0.5;
        camera = Camera(params);
      }
    }, calib.intrinsics[i].variant);
    calib.resolution[i] /= 2;
  }
}

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
    last_ns = 0;
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

}  // namespace granite
