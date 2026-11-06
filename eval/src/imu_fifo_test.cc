// Standalone copy of the xeye ImuMPU6050 FIFO loop (imu_mpu6050.cc,
// SetupFifoMode and FetchFifoData) with timing logs per transfer.
// Usage: imu_fifo_test /dev/i2c-N seconds [chunk_bytes=192] [poll_ms=20] [mode=0] [settle_ms=0]
//   mode 0: reset the FIFO after an error (xeye). mode 1: keep the FIFO and realign to the
//   12-byte sample boundary (Granite pi_camera_imu.cpp since 1af850d).
// Build (desktop, for the Pi 4):
//   aarch64-linux-gnu-g++-13 -O2 -std=c++17 eval/src/imu_fifo_test.cc -o imu_fifo_test
// See doc/mpu6500-cp2112-issue.md.
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std;
using Clock = chrono::steady_clock;

static int fd = -1;
static const uint16_t kAddr = 0x68;

static int64_t nowNs() {
  return chrono::duration_cast<chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}

struct Stat {
  vector<double> ms;
  map<int, int> errno_count;
};
static Stat st_count, st_read, st_write;

static void writeReg(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};
  i2c_msg m{kAddr, 0, 2, data};
  i2c_rdwr_ioctl_data t{&m, 1};
  const int64_t a = nowNs();
  const int r = ioctl(fd, I2C_RDWR, &t);
  st_write.ms.push_back((nowNs() - a) * 1e-6);
  if (r != 1) {
    printf("FAIL start=%.6f dur=%.1f ms write reg=0x%02x\n", a * 1e-9, (nowNs() - a) * 1e-6, reg);
    st_write.errno_count[errno]++;
    throw runtime_error(string("write ") + strerror(errno));
  }
}

static void readRegs(uint8_t reg, uint8_t* data, size_t n, Stat& st) {
  i2c_msg m[2] = {{kAddr, 0, 1, &reg}, {kAddr, I2C_M_RD, (uint16_t)n, data}};
  i2c_rdwr_ioctl_data t{m, 2};
  const int64_t a = nowNs();
  const int r = ioctl(fd, I2C_RDWR, &t);
  st.ms.push_back((nowNs() - a) * 1e-6);
  if (r != 2) {
    printf("FAIL start=%.6f dur=%.1f ms reg=0x%02x n=%zu\n", a * 1e-9, (nowNs() - a) * 1e-6, reg, n);
    st.errno_count[errno]++;
    throw runtime_error(string("read ") + strerror(errno));
  }
}

static void report(const char* name, Stat& s) {
  if (s.ms.empty()) return;
  vector<double> v = s.ms;
  sort(v.begin(), v.end());
  printf("%-6s n=%zu p50=%.2f p95=%.2f p99=%.2f max=%.2f ms", name, v.size(), v[v.size() / 2],
         v[v.size() * 95 / 100], v[v.size() * 99 / 100], v.back());
  for (auto& e : s.errno_count) printf("  errno %d (%s) x%d", e.first, strerror(e.first), e.second);
  printf("\n");
}

int main(int argc, char** argv) {
  if (argc < 3) return 2;
  const double seconds = atof(argv[2]);
  const size_t chunk = argc > 3 ? atoi(argv[3]) : 16 * 12;
  const int poll_ms = argc > 4 ? atoi(argv[4]) : 20;
  const int mode = argc > 5 ? atoi(argv[5]) : 0;  // 0: reset after an error (xeye), 1: realign
  const int settle_ms = argc > 6 ? atoi(argv[6]) : 0;
  long realigns = 0, partial = 0, partial_bytes = 0;
  bool realign = false;
  fd = open(argv[1], O_RDWR);
  if (fd < 0) { perror("open"); return 1; }
  if (ioctl(fd, I2C_SLAVE, kAddr) < 0) { perror("I2C_SLAVE"); return 1; }

  // SetupFifoMode (xeye), 200 Hz
  uint8_t id = 0;
  readRegs(0x75, &id, 1, st_read);
  printf("WHO_AM_I 0x%02x chunk=%zu poll=%d ms\n", id, chunk, poll_ms);
  writeReg(0x6B, 0x80);
  this_thread::sleep_for(chrono::milliseconds(100));
  writeReg(0x6B, 0x01);
  this_thread::sleep_for(chrono::milliseconds(50));
  writeReg(0x6C, 0x00);
  writeReg(0x1A, 0x42);
  writeReg(0x19, 4);  // 1 kHz / 5 = 200 Hz
  writeReg(0x1B, 0x10);
  writeReg(0x1C, 0x08);
  writeReg(0x1D, 0x02);

  // FetchFifoData (xeye)
  vector<uint8_t> bytes;
  bool reset = true;
  long samples_total = 0, errors = 0, resets = 0, overflows = 0;
  int64_t last_good_ns = 0, max_gap_ns = 0;
  vector<double> gaps;
  const int64_t t_end = nowNs() + (int64_t)(seconds * 1e9);
  auto next = Clock::now();
  while (nowNs() < t_end) {
    next += chrono::milliseconds(poll_ms);
    this_thread::sleep_until(next);
    try {
      if (reset) {
        ++resets;
        writeReg(0x23, 0x00);
        writeReg(0x6A, 0x04);
        this_thread::sleep_for(chrono::milliseconds(1));
        writeReg(0x6A, 0x40);
        writeReg(0x23, 0x78);
        reset = false;
        next = Clock::now();
        continue;
      }
      uint8_t cb[2];
      readRegs(0x72, cb, 2, st_count);
      const size_t count = (cb[0] & 0x1F) * 256 + cb[1];
      if (count + 12 > 512) { ++overflows; reset = true; continue; }
      if (realign) {
        realign = false;
        ++realigns;
        const size_t r = count % 12;
        if (r) {
          ++partial; partial_bytes += r;
          uint8_t junk[12];
          readRegs(0x74, junk, r, st_read);
          continue;
        }
      }
      const size_t n = count / 12;
      if (n == 0) continue;
      bytes.resize(n * 12);
      for (size_t off = 0; off < bytes.size(); off += chunk)
        readRegs(0x74, bytes.data() + off, min(chunk, bytes.size() - off), st_read);
      samples_total += n;
      const int64_t t = nowNs();
      if (last_good_ns) {
        const int64_t g = t - last_good_ns;
        if (g > 3 * poll_ms * 1000000LL) gaps.push_back(g * 1e-6);
        max_gap_ns = max(max_gap_ns, g);
      }
      last_good_ns = t;
      next = max(next, Clock::now() - chrono::milliseconds(poll_ms));
    } catch (const exception& e) {
      ++errors;
      printf("ERR mono=%.6f %s\n", nowNs() * 1e-9, e.what());
      if (mode == 0) reset = true; else realign = true;
      if (settle_ms) this_thread::sleep_for(chrono::milliseconds(settle_ms));
      next = Clock::now();
    }
  }
  printf("samples=%ld rate=%.1f Hz errors=%ld resets=%ld overflows=%ld max_gap=%.1f ms gaps>%dms=%zu\n",
         samples_total, samples_total / seconds, errors, resets - 1, overflows, max_gap_ns * 1e-6, 3 * poll_ms,
         gaps.size());
  printf("mode=%d settle=%d realigns=%ld partial=%ld partial_bytes=%ld\n", mode, settle_ms, realigns, partial, partial_bytes);
  report("count", st_count);
  report("read", st_read);
  report("write", st_write);
  return 0;
}
