// SPDX-License-Identifier: MIT
// UVIFY VIO Viewer: shows the visualization messages of the Granite VIO (send_vis_msg, doc/granite_vis.proto)
// on the ground. The GUI follows the openvio viewer (src/vio/ui/viewer.cc): ImGui and ImPlot on GLFW.
//
// Usage: uvify-vio-viewer [--ip LISTEN_IP] [--port PORT] [--snapshot FILE.jpg] [--snapshot-delay SECONDS]
//   --snapshot  writes the window as a JPEG after --snapshot-delay (default 5 s) and continues.
#include "camera_projection.h"
#include "granite_vis_message.h"
#include "h264_decoder.h"

#include <arpa/inet.h>
#include <jpeglib.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csetjmp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <GLFW/glfw3.h>

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "imgui.h"
#include "implot.h"

namespace {

using Clock = std::chrono::steady_clock;

// Received positions kept for the trajectory; the oldest 10 % go when it is full. The 3D view draws at most
// kMaxDrawnSegments of them (evenly spaced).
constexpr size_t kMaxTrajectory = 400000;
constexpr size_t kMaxDrawnSegments = 20000;
// Samples kept for the plots (20 minutes at 20 Hz)
constexpr size_t kMaxPlotSamples = 24000;
// Time span of the plots [s]: a fixed x range that moves with the newest sample
constexpr double kPlotSeconds = 30.0;
// Without a message for this time the viewer marks the data as old [s]
constexpr double kStaleSeconds = 1.0;

const ImU32 kColorTrajectory = IM_COL32(235, 235, 235, 255);
const ImU32 kColorWindow = IM_COL32(255, 220, 40, 255);
const ImU32 kColorLandmark = IM_COL32(60, 220, 230, 255);
const ImU32 kColorKeypoint = IM_COL32(60, 230, 60, 255);

ImVec4 toVec4(ImU32 c) { return ImGui::ColorConvertU32ToFloat4(c); }

// getcolor of granite (include/granite/utils/vis_utils.h) as the granite GUI calls it, with the red and blue
// outputs swapped: p = 0 is blue, p = range is red, green and yellow between.
ImU32 depthColor(float p, float range) {
  const float x = range > 0.0f ? 4.0f * p / range : 0.0f;
  float r = 0.0f, g = 0.0f, b = 0.0f;
  if ((0 <= x && x <= 1) || (5 <= x && x <= 6)) {
    b = 1.0f;
  } else if (4 <= x && x <= 5) {
    b = x - 4;
  } else if (1 <= x && x <= 2) {
    b = 1.0f - (x - 1);
  }
  if (1 <= x && x <= 3) {
    g = 1.0f;
  } else if (0 <= x && x <= 1) {
    g = x;
  } else if (3 <= x && x <= 4) {
    g = 1.0f - (x - 3);
  }
  if (3 <= x && x <= 5) {
    r = 1.0f;
  } else if (2 <= x && x <= 3) {
    r = x - 2;
  } else if (5 <= x && x <= 6) {
    r = 1.0f - (x - 5);
  }
  return IM_COL32(static_cast<int>(255 * r), static_cast<int>(255 * g), static_cast<int>(255 * b), 255);
}

struct JpegError {
  jpeg_error_mgr manager;
  std::jmp_buf jump;
};

void jpegErrorExit(j_common_ptr info) { std::longjmp(reinterpret_cast<JpegError*>(info->err)->jump, 1); }

// A gray JPEG to 8-bit pixels. libjpeg stops the process on a broken image by default; the jump returns false.
bool decodeGrayJpeg(const std::string& jpeg, std::vector<uint8_t>* pixels, int* width, int* height) {
  jpeg_decompress_struct info;
  JpegError error;
  info.err = jpeg_std_error(&error.manager);
  error.manager.error_exit = jpegErrorExit;
  if (setjmp(error.jump)) {
    jpeg_destroy_decompress(&info);
    return false;
  }
  jpeg_create_decompress(&info);
  jpeg_mem_src(&info, reinterpret_cast<const unsigned char*>(jpeg.data()), jpeg.size());
  jpeg_read_header(&info, TRUE);
  info.out_color_space = JCS_GRAYSCALE;
  jpeg_start_decompress(&info);
  *width = static_cast<int>(info.output_width);
  *height = static_cast<int>(info.output_height);
  pixels->resize(static_cast<size_t>(*width) * *height);
  while (info.output_scanline < info.output_height) {
    JSAMPROW row = pixels->data() + static_cast<size_t>(info.output_scanline) * *width;
    jpeg_read_scanlines(&info, &row, 1);
  }
  jpeg_finish_decompress(&info);
  jpeg_destroy_decompress(&info);
  return true;
}

// The frame buffer of the window as a JPEG.
bool saveSnapshot(GLFWwindow* window, const std::string& path) {
  int w = 0, h = 0;
  glfwGetFramebufferSize(window, &w, &h);
  std::vector<uint8_t> pixels(static_cast<size_t>(w) * h * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
  FILE* file = std::fopen(path.c_str(), "wb");
  if (!file) return false;
  jpeg_compress_struct info;
  jpeg_error_mgr error;
  info.err = jpeg_std_error(&error);
  jpeg_create_compress(&info);
  jpeg_stdio_dest(&info, file);
  info.image_width = static_cast<JDIMENSION>(w);
  info.image_height = static_cast<JDIMENSION>(h);
  info.input_components = 3;
  info.in_color_space = JCS_RGB;
  jpeg_set_defaults(&info);
  jpeg_set_quality(&info, 90, TRUE);
  jpeg_start_compress(&info, TRUE);
  while (info.next_scanline < info.image_height) {
    // OpenGL rows start at the bottom
    JSAMPROW row = pixels.data() + static_cast<size_t>(h - 1 - info.next_scanline) * w * 3;
    jpeg_write_scanlines(&info, &row, 1);
  }
  jpeg_finish_compress(&info);
  jpeg_destroy_compress(&info);
  std::fclose(file);
  return true;
}

// Keeps at most `max` samples: removes the oldest 10 % when full.
template <typename T>
void pushBounded(std::vector<T>& v, const T& value, size_t max) {
  if (v.size() >= max) v.erase(v.begin(), v.begin() + static_cast<long>(max / 10));
  v.push_back(value);
}

// Data that the receiver thread writes and the GUI thread reads, under mutex.
struct Shared {
  std::mutex mutex;
  bool have_state = false;
  GraniteVisMessage state;   // the newest message
  GraniteVisMessage detail;  // the newest message with an image, with the projections of the landmarks
  // The newest landmarks (they come at vis_msg_points_hz)
  std::vector<float> points;
  bool new_points = false;
  std::string image_codec, decoder_status;
  std::vector<uint8_t> image;
  int image_width = 0, image_height = 0;
  bool new_image = false;
  std::vector<Eigen::Vector3f> trajectory;
  std::vector<float> plot_t, plot_speed, plot_x, plot_y, plot_z;
  int64_t first_t_ns = -1;
  // Receive statistics
  std::string listening, error;
  double message_rate = 0, kilobytes_rate = 0;
  uint64_t received = 0, lost = 0, malformed = 0;
  Clock::time_point last_receive, last_image;

  void clearHistory() {
    trajectory.clear();
    plot_t.clear();
    plot_speed.clear();
    plot_x.clear();
    plot_y.clear();
    plot_z.clear();
    first_t_ns = -1;
  }
};

class Receiver {
 public:
  explicit Receiver(Shared* shared) : shared(shared), thread([this] { run(); }) {}
  ~Receiver() {
    stopping = true;
    thread.join();
    if (fd >= 0) close(fd);
  }
  // Binds the UDP socket to ip:port (0.0.0.0: every interface) on the receiver thread.
  void listen(const std::string& ip, int port) {
    std::lock_guard<std::mutex> lock(request_mutex);
    request_ip = ip;
    request_port = port;
    rebind = true;
  }

 private:
  void bindSocket() {
    std::string ip;
    int port = 0;
    {
      std::lock_guard<std::mutex> lock(request_mutex);
      ip = request_ip;
      port = request_port;
      rebind = false;
    }
    if (fd >= 0) close(fd);
    fd = -1;
    std::string error;
    sockaddr_in address{};
    address.sin_family = AF_INET;
    if (port < 1 || port > 65535) {
      error = "port must be 1-65535";
    } else if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1) {
      error = "not an IPv4 address: " + ip;
    } else {
      address.sin_port = htons(static_cast<uint16_t>(port));
      fd = socket(AF_INET, SOCK_DGRAM, 0);
      const int buffer = 4 << 20;
      setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &buffer, sizeof(buffer));
      if (bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        error = std::string("bind failed: ") + std::strerror(errno);
        close(fd);
        fd = -1;
      }
    }
    std::lock_guard<std::mutex> lock(shared->mutex);
    shared->listening = ip + ":" + std::to_string(port);
    shared->error = error;
    shared->have_state = false;
    shared->clearHistory();
    shared->points.clear();
    shared->new_points = true;
    last_seq = -1;
    last_image_seq = 0;
    landmarks.clear();
    landmark_points.clear();
  }

  void run() {
    std::vector<uint8_t> buffer(65536);
    GraniteVisMessage message;
    std::vector<uint8_t> pixels;
    auto window_start = Clock::now();
    uint64_t window_messages = 0, window_bytes = 0;
    while (!stopping) {
      if (rebind) bindSocket();
      const auto now = Clock::now();
      if (now - window_start >= std::chrono::seconds(1)) {
        const double seconds = std::chrono::duration<double>(now - window_start).count();
        std::lock_guard<std::mutex> lock(shared->mutex);
        shared->message_rate = window_messages / seconds;
        shared->kilobytes_rate = window_bytes / 1e3 / seconds;
        window_start = now;
        window_messages = window_bytes = 0;
      }
      if (fd < 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      pollfd p{fd, POLLIN, 0};
      if (poll(&p, 1, 100) <= 0) continue;
      const ssize_t n = recv(fd, buffer.data(), buffer.size(), 0);
      if (n <= 0) continue;
      // The rate counts the datagrams with their IP and UDP headers, as the publisher limits it.
      window_bytes += static_cast<uint64_t>(n) + 28;
      if (!parseGraniteVis(buffer.data(), static_cast<size_t>(n), &message)) {
        std::lock_guard<std::mutex> lock(shared->mutex);
        ++shared->malformed;
        continue;
      }
      if (message.fragment_seq != 0) {
        if (!fragments.add(message, &joined)) continue;
        if (!parseGraniteVis(reinterpret_cast<const uint8_t*>(joined.data()), joined.size(), &message)) {
          std::lock_guard<std::mutex> lock(shared->mutex);
          ++shared->malformed;
          continue;
        }
      }
      ++window_messages;
      if (!message.points.empty()) {
        landmark_points = message.points;
        landmarks.clear();
        for (size_t k = 0; k < message.point_ids.size() && 3 * k + 2 < landmark_points.size(); ++k)
          landmarks[message.point_ids[k]] = k;
      }
      int width = 0, height = 0;
      bool image = false;
      if (!message.image.empty()) {
        if (message.image_h264) {
          const bool gap = message.image_seq != last_image_seq + 1;
          image = h264.decode(message.image, message.image_keyframe, gap, &pixels, &width, &height);
        } else {
          image = decodeGrayJpeg(message.image, &pixels, &width, &height);
        }
        last_image_seq = message.image_seq;
        message.image.clear();
      }
      if (image) projectLandmarks(&message);

      std::lock_guard<std::mutex> lock(shared->mutex);
      ++shared->received;
      shared->last_receive = Clock::now();
      // A smaller seq is a new VIO process: its world is new.
      if (last_seq >= 0 && static_cast<int64_t>(message.seq) < last_seq) shared->clearHistory();
      if (last_seq >= 0 && static_cast<int64_t>(message.seq) > last_seq + 1)
        shared->lost += message.seq - static_cast<uint64_t>(last_seq) - 1;
      last_seq = static_cast<int64_t>(message.seq);
      if (message.pose.size() >= 3) {
        const Eigen::Vector3f p(message.pose[0], message.pose[1], message.pose[2]);
        pushBounded(shared->trajectory, p, kMaxTrajectory);
        if (shared->first_t_ns < 0) shared->first_t_ns = message.t_ns;
        pushBounded(shared->plot_t, static_cast<float>((message.t_ns - shared->first_t_ns) * 1e-9), kMaxPlotSamples);
        const float speed = message.velocity.size() >= 3
                                ? Eigen::Vector3f(message.velocity[0], message.velocity[1], message.velocity[2]).norm()
                                : 0.0f;
        pushBounded(shared->plot_speed, speed, kMaxPlotSamples);
        pushBounded(shared->plot_x, p.x(), kMaxPlotSamples);
        pushBounded(shared->plot_y, p.y(), kMaxPlotSamples);
        pushBounded(shared->plot_z, p.z(), kMaxPlotSamples);
      }
      if (!message.points.empty()) {
        shared->points = landmark_points;
        shared->new_points = true;
      }
      if (message.image_h264) {
        shared->image_codec = "h264";
        shared->decoder_status = h264.available() ? "" : h264.status();
      } else if (image) {
        shared->image_codec = "mjpeg";
      }
      if (image) {
        shared->image.swap(pixels);
        shared->image_width = width;
        shared->image_height = height;
        shared->new_image = true;
        shared->last_image = shared->last_receive;
        shared->detail = message;
      }
      shared->state = std::move(message);
      shared->have_state = true;
    }
  }

  // The projections of the landmarks seen in this image (the keypoints with a landmark), as granite computes them
  // for its GUI: the pose of the message, T_i_c and the camera model of the message.
  void projectLandmarks(GraniteVisMessage* m) const {
    m->projections.clear();
    if (m->pose.size() < 7 || m->T_i_c.size() < 7 || landmarks.empty()) return;
    const auto pose = [](const std::vector<float>& v) {
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.linear() = Eigen::Quaterniond(v[3], v[4], v[5], v[6]).normalized().toRotationMatrix();
      T.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
      return T;
    };
    const Eigen::Isometry3d T_c_w = (pose(m->pose) * pose(m->T_i_c)).inverse();
    for (uint64_t id : m->keypoint_ids) {
      const auto it = landmarks.find(id);
      if (it == landmarks.end()) continue;
      const float* p = &landmark_points[3 * it->second];
      const Eigen::Vector3d x = T_c_w * Eigen::Vector3d(p[0], p[1], p[2]);
      Eigen::Vector2d uv;
      if (!projectPoint(m->camera_model, m->camera_params, x, &uv)) continue;
      m->projections.push_back(static_cast<float>(uv.x()));
      m->projections.push_back(static_cast<float>(uv.y()));
      m->projections.push_back(static_cast<float>(1.0 / x.norm()));
    }
  }

  Shared* shared;
  FragmentJoiner fragments;
  std::string joined;
  H264Decoder h264;
  std::unordered_map<uint64_t, size_t> landmarks;  // landmark id to its index in landmark_points
  std::vector<float> landmark_points;
  uint64_t last_image_seq = 0;
  std::atomic<bool> stopping{false};
  std::mutex request_mutex;
  std::string request_ip = "0.0.0.0";
  int request_port = 5100;
  std::atomic<bool> rebind{false};
  int fd = -1;
  int64_t last_seq = -1;
  std::thread thread;
};

std::string configPath() {
  const char* home = std::getenv("HOME");
  return std::string(home ? home : ".") + "/.config/uvify-vio-viewer/listen.conf";
}

void loadListen(std::string* ip, int* port) {
  std::ifstream in(configPath());
  std::string i;
  int p = 0;
  if (in >> i >> p) {
    *ip = i;
    *port = p;
  }
}

void saveListen(const std::string& ip, int port) {
  const std::string path = configPath();
  if (std::system(("mkdir -p \"" + path.substr(0, path.rfind('/')) + "\"").c_str()) != 0) return;
  std::ofstream(path) << ip << ' ' << port << '\n';
}

Eigen::Matrix3d rotationOf(const std::vector<float>& pose) {
  return Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]).normalized().toRotationMatrix();
}

class Viewer {
 public:
  Viewer(Shared* shared, Receiver* receiver, const std::string& ip, int port)
      : shared(shared), receiver(receiver), port(port) {
    std::snprintf(ip_text, sizeof(ip_text), "%s", ip.c_str());
  }

  bool initialize() {
    if (!glfwInit()) return false;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    window = glfwCreateWindow(1600, 900, "UVIFY VIO Viewer", nullptr, nullptr);
    if (!window) {
      glfwTerminate();
      return false;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGui::GetStyle().WindowRounding = 0.0f;
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");
    glGenTextures(1, &texture);
    return true;
  }

  ~Viewer() {
    if (!window) return;
    glDeleteTextures(1, &texture);
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
  }

  // One frame; false when the window closes.
  bool frame(const std::string& snapshot) {
    if (glfwWindowShouldClose(window)) return false;
    glfwPollEvents();
    copyShared();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    const ImVec2 size = ImGui::GetIO().DisplaySize;
    const float status_width = 340.0f;
    const float plot_height = std::max(160.0f, 0.26f * size.y);
    const float top_height = size.y - plot_height;
    const float image_width = 0.45f * (size.x - status_width);
    drawStatus(ImVec2(0, 0), ImVec2(status_width, size.y));
    drawImage(ImVec2(status_width, 0), ImVec2(image_width, top_height));
    drawScene(ImVec2(status_width + image_width, 0), ImVec2(size.x - status_width - image_width, top_height));
    drawPlots(ImVec2(status_width, top_height), ImVec2(size.x - status_width, plot_height));

    ImGui::Render();
    int fb_width = 0, fb_height = 0;
    glfwGetFramebufferSize(window, &fb_width, &fb_height);
    glViewport(0, 0, fb_width, fb_height);
    glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    if (!snapshot.empty()) std::cout << (saveSnapshot(window, snapshot) ? "Wrote " : "Cannot write ") << snapshot << std::endl;
    if (!screenshot.empty()) {
      std::cout << (saveSnapshot(window, screenshot) ? "Wrote " : "Cannot write ") << screenshot << std::endl;
      screenshot.clear();
    }
    glfwSwapBuffers(window);
    return true;
  }

 private:
  void copyShared() {
    std::lock_guard<std::mutex> lock(shared->mutex);
    if (shared->new_image) {
      glBindTexture(GL_TEXTURE_2D, texture);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
      // Gray to RGB: the OpenGL 3 core path of the ImGui backend has no luminance texture
      rgb.resize(shared->image.size() * 3);
      for (size_t i = 0; i < shared->image.size(); ++i) rgb[3 * i] = rgb[3 * i + 1] = rgb[3 * i + 2] = shared->image[i];
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, shared->image_width, shared->image_height, 0, GL_RGB, GL_UNSIGNED_BYTE,
                   rgb.data());
      texture_width = shared->image_width;
      texture_height = shared->image_height;
      shared->new_image = false;
      detail = shared->detail;
    }
    have_state = shared->have_state;
    if (shared->new_points) {
      points = shared->points;
      shared->new_points = false;
    }
    image_codec = shared->image_codec;
    decoder_status = shared->decoder_status;
    if (have_state) state = shared->state;
    // At most kMaxDrawnSegments + 1 evenly spaced positions, and always the newest one
    const size_t n = shared->trajectory.size();
    const size_t step = std::max<size_t>(1, n / kMaxDrawnSegments);
    drawn.clear();
    for (size_t i = 0; i < n; i += step) drawn.push_back(shared->trajectory[i]);
    if (n > 0 && (n - 1) % step != 0) drawn.push_back(shared->trajectory.back());
    listening = shared->error.empty() ? shared->listening : shared->error;
    listen_error = !shared->error.empty();
    message_rate = shared->message_rate;
    kilobytes_rate = shared->kilobytes_rate;
    received = shared->received;
    lost = shared->lost;
    malformed = shared->malformed;
    age = received ? std::chrono::duration<double>(Clock::now() - shared->last_receive).count() : -1.0;
    image_age = texture_width > 0 ? std::chrono::duration<double>(Clock::now() - shared->last_image).count() : -1.0;
    trajectory_points = n;
  }

  void drawStatus(const ImVec2& pos, const ImVec2& size) {
    ImGui::SetNextWindowPos(pos);
    ImGui::SetNextWindowSize(size);
    ImGui::Begin("Status", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    ImGui::SeparatorText("Listen (UDP)");
    ImGui::PushItemWidth(170.0f);
    bool apply = ImGui::InputText("IP", ip_text, sizeof(ip_text), ImGuiInputTextFlags_EnterReturnsTrue);
    apply |= ImGui::InputInt("port", &port, 0, 0, ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::PopItemWidth();
    apply |= ImGui::Button("Apply");
    ImGui::SameLine();
    ImGui::TextDisabled("0.0.0.0: every interface");
    if (apply) {
      receiver->listen(ip_text, port);
      saveListen(ip_text, port);
    }
    ImGui::TextColored(listen_error ? ImVec4(1.0f, 0.35f, 0.35f, 1.0f) : ImVec4(0.6f, 0.9f, 0.6f, 1.0f), "%s",
                       listening.c_str());

    ImGui::SeparatorText("Messages");
    ImGui::Text("rate      %.1f msg/s, %.0f kbit/s", message_rate, 8.0 * kilobytes_rate);
    if (age < 0) {
      ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "no message yet");
    } else if (age > kStaleSeconds) {
      ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "last      %.1f s ago (no messages)", age);
    } else {
      ImGui::Text("last      %.1f s ago", age);
    }
    ImGui::Text("received  %llu (seq %llu)", static_cast<unsigned long long>(received),
                static_cast<unsigned long long>(state.seq));
    ImGui::Text("lost      %llu  skipped %llu  bad %llu", static_cast<unsigned long long>(lost),
                static_cast<unsigned long long>(state.skipped), static_cast<unsigned long long>(malformed));

    if (!image_codec.empty()) ImGui::Text("image     %s", image_codec.c_str());
    if (!decoder_status.empty()) ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "h264: %s", decoder_status.c_str());

    ImGui::SeparatorText("VIO");
    if (!have_state) {
      ImGui::TextDisabled("no state");
    } else {
      ImGui::TextColored(state.healthy ? ImVec4(0.4f, 1.0f, 0.4f, 1.0f) : ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%s",
                         state.healthy ? "healthy" : "UNHEALTHY (ODOMETRY quality -1)");
      if (state.pose.size() >= 3)
        ImGui::Text("position  %.2f %.2f %.2f m", state.pose[0], state.pose[1], state.pose[2]);
      if (state.velocity.size() >= 3) {
        const float speed = Eigen::Vector3f(state.velocity[0], state.velocity[1], state.velocity[2]).norm();
        ImGui::Text("speed     %.2f m/s", speed);
      }
      if (state.bias_gyro.size() >= 3)
        ImGui::Text("bias gyro %.4f %.4f %.4f", state.bias_gyro[0], state.bias_gyro[1], state.bias_gyro[2]);
      if (state.bias_accel.size() >= 3)
        ImGui::Text("bias acc  %.3f %.3f %.3f", state.bias_accel[0], state.bias_accel[1], state.bias_accel[2]);
      ImGui::Text("window    %zu states, %zu landmarks", state.window_states.size() / 7, points.size() / 3);
      ImGui::Text("image     %zu keypoints, %zu projections", detail.keypoints.size() / 2,
                  detail.projections.size() / 3);
      ImGui::Text("trajectory %zu points", trajectory_points);
    }

    ImGui::SeparatorText("View");
    ImGui::Checkbox("follow", &follow);
    ImGui::SameLine();
    ImGui::Checkbox("grid", &show_grid);
    ImGui::Checkbox("trajectory", &show_trajectory);
    ImGui::SameLine();
    ImGui::Checkbox("landmarks", &show_landmarks);
    ImGui::SameLine();
    ImGui::Checkbox("window", &show_window);
    if (ImGui::Button("Clear trajectory")) {
      std::lock_guard<std::mutex> lock(shared->mutex);
      shared->clearHistory();
    }
    ImGui::SameLine();
    if (ImGui::Button("Screenshot")) screenshot = "uvify_vio_viewer_" + std::to_string(state.seq) + ".jpg";
    ImGui::End();
  }

  void drawImage(const ImVec2& pos, const ImVec2& size) {
    ImGui::SetNextWindowPos(pos);
    ImGui::SetNextWindowSize(size);
    ImGui::Begin("Image", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoScrollbar);
    if (texture_width > 0) {
      // Fit the image in the panel with its aspect ratio and keep one line for the legend
      const ImVec2 avail = ImGui::GetContentRegionAvail();
      const float legend = ImGui::GetStyle().ItemSpacing.y + ImGui::GetTextLineHeight();
      const float scale = std::max(0.0f, std::min(avail.x / static_cast<float>(texture_width),
                                                  (avail.y - legend) / static_cast<float>(texture_height)));
      const ImVec2 image_size(texture_width * scale, texture_height * scale);
      ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 0.5f * (avail.x - image_size.x));
      const ImVec2 origin = ImGui::GetCursorScreenPos();
      ImGui::Image(static_cast<ImTextureID>(texture), image_size);
      ImDrawList* draw = ImGui::GetWindowDrawList();
      draw->PushClipRect(origin, ImVec2(origin.x + image_size.x, origin.y + image_size.y), true);
      for (size_t i = 0; i + 1 < detail.keypoints.size(); i += 2)
        draw->AddCircle(ImVec2(origin.x + detail.keypoints[i] * scale, origin.y + detail.keypoints[i + 1] * scale),
                        2.5f, kColorKeypoint);
      // As the granite GUI: the inverse distances of this image from their minimum (blue, far) to their maximum
      // (red, near)
      float min_inv = std::numeric_limits<float>::infinity(), max_inv = 0.0f;
      for (size_t i = 2; i < detail.projections.size(); i += 3) {
        if (detail.projections[i] <= 0.0f) continue;
        min_inv = std::min(min_inv, detail.projections[i]);
        max_inv = std::max(max_inv, detail.projections[i]);
      }
      for (size_t i = 0; i + 2 < detail.projections.size(); i += 3) {
        const float inv = detail.projections[i + 2];
        const ImU32 color = inv > 0.0f ? depthColor(inv - min_inv, max_inv - min_inv) : IM_COL32(0, 0, 255, 255);
        draw->AddCircleFilled(
            ImVec2(origin.x + detail.projections[i] * scale, origin.y + detail.projections[i + 1] * scale), 3.0f,
            color);
      }
      if (image_age > kStaleSeconds) {
        char text[64];
        std::snprintf(text, sizeof(text), "no new image for %.0f s", image_age);
        draw->AddRectFilled(origin, ImVec2(origin.x + image_size.x, origin.y + ImGui::GetTextLineHeight() + 8.0f),
                            IM_COL32(120, 20, 20, 220));
        draw->AddText(ImVec2(origin.x + 6.0f, origin.y + 4.0f), IM_COL32(255, 255, 255, 255), text);
      }
      draw->PopClipRect();
      ImGui::TextColored(toVec4(kColorKeypoint), "keypoints %zu", detail.keypoints.size() / 2);
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(0.9f, 0.4f, 0.6f, 1.0f), "  landmark projections %zu (red near, blue far in this image)",
                         detail.projections.size() / 3);
    } else {
      ImGui::TextDisabled("no image yet");
    }
    ImGui::End();
  }

  void drawScene(const ImVec2& pos, const ImVec2& size) {
    ImGui::SetNextWindowPos(pos);
    ImGui::SetNextWindowSize(size);
    ImGui::Begin("3D view  (left drag: rotate, right drag: pan, wheel: zoom)", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 canvas = ImGui::GetContentRegionAvail();
    if (canvas.x < 10.0f || canvas.y < 10.0f) {
      ImGui::End();
      return;
    }
    ImGui::InvisibleButton("scene", canvas, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    const ImGuiIO& io = ImGui::GetIO();

    // Orbit camera around the target (world z is up)
    if (ImGui::IsItemHovered() && io.MouseWheel != 0.0f)
      distance = std::clamp(distance * std::pow(0.88f, io.MouseWheel), 0.5f, 5000.0f);
    if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      yaw -= io.MouseDelta.x * 0.008f;
      pitch = std::clamp(pitch + io.MouseDelta.y * 0.008f, -1.5f, 1.5f);
    }
    const Eigen::Vector3d eye_dir(std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch));
    const Eigen::Vector3d forward = -eye_dir;
    const Eigen::Vector3d right = forward.cross(Eigen::Vector3d::UnitZ()).normalized();
    const Eigen::Vector3d up = right.cross(forward);
    if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
      target += (-io.MouseDelta.x * right + io.MouseDelta.y * up) * (distance * 0.0015);
      follow = false;
    }
    if (follow && have_state && state.pose.size() >= 3) target = Eigen::Vector3d(state.pose[0], state.pose[1], state.pose[2]);
    const Eigen::Vector3d eye = target + distance * eye_dir;
    const double focal = 0.5 * canvas.y / std::tan(30.0 * M_PI / 180.0);
    const ImVec2 center(origin.x + 0.5f * canvas.x, origin.y + 0.5f * canvas.y);
    const auto project = [&](const Eigen::Vector3d& p, ImVec2& out) {
      const Eigen::Vector3d d = p - eye;
      const double z = d.dot(forward);
      if (z < 0.05) return false;
      out = ImVec2(static_cast<float>(center.x + focal * d.dot(right) / z),
                   static_cast<float>(center.y - focal * d.dot(up) / z));
      return true;
    };

    ImDrawList* draw = ImGui::GetWindowDrawList();
    // Everything stays inside the canvas of this window
    draw->PushClipRect(origin, ImVec2(origin.x + canvas.x, origin.y + canvas.y), true);
    draw->AddRectFilled(origin, ImVec2(origin.x + canvas.x, origin.y + canvas.y), IM_COL32(18, 20, 26, 255));
    const auto line = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b, ImU32 color, float width) {
      ImVec2 pa, pb;
      if (project(a, pa) && project(b, pb)) draw->AddLine(pa, pb, color, width);
    };

    if (show_grid) {
      const double ground = std::floor(target.z() - 1.0);
      const double spacing = std::pow(10.0, std::floor(std::log10(std::max(1.0, distance / 4.0))));
      const int half = 10;
      const double cx = std::round(target.x() / spacing) * spacing;
      const double cy = std::round(target.y() / spacing) * spacing;
      for (int k = -half; k <= half; ++k) {
        const ImU32 color = (k % 5 == 0) ? IM_COL32(70, 75, 90, 255) : IM_COL32(45, 48, 58, 255);
        line(Eigen::Vector3d(cx + k * spacing, cy - half * spacing, ground),
             Eigen::Vector3d(cx + k * spacing, cy + half * spacing, ground), color, 1.0f);
        line(Eigen::Vector3d(cx - half * spacing, cy + k * spacing, ground),
             Eigen::Vector3d(cx + half * spacing, cy + k * spacing, ground), color, 1.0f);
      }
    }
    if (show_landmarks) {
      ImVec2 q;
      for (size_t i = 0; i + 2 < points.size(); i += 3)
        if (project(Eigen::Vector3d(points[i], points[i + 1], points[i + 2]), q))
          draw->AddRectFilled(ImVec2(q.x - 1.0f, q.y - 1.0f), ImVec2(q.x + 1.0f, q.y + 1.0f), kColorLandmark);
    }
    if (show_trajectory)
      for (size_t i = 1; i < drawn.size(); ++i)
        line(drawn[i - 1].cast<double>(), drawn[i].cast<double>(), kColorTrajectory, 2.0f);
    if (show_window) {
      ImVec2 q;
      for (size_t i = 0; i + 6 < state.window_states.size(); i += 7)
        if (project(Eigen::Vector3d(state.window_states[i], state.window_states[i + 1], state.window_states[i + 2]),
                    q))
          draw->AddCircle(q, 3.0f, kColorWindow);
    }
    if (have_state && state.pose.size() >= 7) {
      const Eigen::Vector3d p(state.pose[0], state.pose[1], state.pose[2]);
      const Eigen::Matrix3d R = rotationOf(state.pose);
      const double length = 0.06 * distance;
      line(p, p + length * R.col(0), IM_COL32(230, 70, 70, 255), 3.0f);
      line(p, p + length * R.col(1), IM_COL32(70, 230, 70, 255), 3.0f);
      line(p, p + length * R.col(2), IM_COL32(80, 120, 255, 255), 3.0f);
      if (state.velocity.size() >= 3)
        line(p, p + Eigen::Vector3d(state.velocity[0], state.velocity[1], state.velocity[2]),
             IM_COL32(255, 240, 80, 255), 2.0f);
    }
    draw->PopClipRect();

    ImGui::SetCursorScreenPos(ImVec2(origin.x + 10.0f, origin.y + 8.0f));
    ImGui::TextColored(toVec4(kColorTrajectory), "trajectory");
    ImGui::SameLine();
    ImGui::TextColored(toVec4(kColorWindow), "  window states");
    ImGui::SameLine();
    ImGui::TextColored(toVec4(kColorLandmark), "  landmarks");
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f, 0.94f, 0.3f, 1.0f), "  velocity");
    ImGui::End();
  }

  void drawPlots(const ImVec2& pos, const ImVec2& size) {
    ImGui::SetNextWindowPos(pos);
    ImGui::SetNextWindowSize(size);
    ImGui::Begin("Plots", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
    const float half = 0.5f * ImGui::GetContentRegionAvail().x - 4.0f;
    const float height = ImGui::GetContentRegionAvail().y;
    std::lock_guard<std::mutex> lock(shared->mutex);
    const auto& t = shared->plot_t;
    const double t_end = std::max(kPlotSeconds, t.empty() ? 0.0 : static_cast<double>(t.back()));
    const double t_begin = t_end - kPlotSeconds;
    const int first = static_cast<int>(std::lower_bound(t.begin(), t.end(), static_cast<float>(t_begin)) - t.begin());
    const int n = static_cast<int>(t.size()) - first;
    // RangeFit: the y axis fits the samples in the x range only.
    const ImPlotAxisFlags y_flags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
    if (ImPlot::BeginPlot("speed", ImVec2(half, height))) {
      ImPlot::SetupAxes("time [s]", "speed [m/s]", ImPlotAxisFlags_None, y_flags);
      ImPlot::SetupAxisLimits(ImAxis_X1, t_begin, t_end, ImPlotCond_Always);
      if (n > 0) ImPlot::PlotLine("|v|", t.data() + first, shared->plot_speed.data() + first, n);
      ImPlot::EndPlot();
    }
    ImGui::SameLine();
    if (ImPlot::BeginPlot("position", ImVec2(half, height))) {
      ImPlot::SetupAxes("time [s]", "position [m]", ImPlotAxisFlags_None, y_flags);
      ImPlot::SetupAxisLimits(ImAxis_X1, t_begin, t_end, ImPlotCond_Always);
      if (n > 0) {
        ImPlot::PlotLine("x", t.data() + first, shared->plot_x.data() + first, n);
        ImPlot::PlotLine("y", t.data() + first, shared->plot_y.data() + first, n);
        ImPlot::PlotLine("z", t.data() + first, shared->plot_z.data() + first, n);
      }
      ImPlot::EndPlot();
    }
    ImGui::End();
  }

  Shared* shared;
  Receiver* receiver;
  GLFWwindow* window = nullptr;
  GLuint texture = 0;
  int texture_width = 0, texture_height = 0;
  std::vector<uint8_t> rgb;
  char ip_text[64] = {};
  int port;
  // Copies of the shared data for one frame
  bool have_state = false;
  GraniteVisMessage state, detail;
  std::vector<float> points;
  std::string image_codec, decoder_status;
  std::vector<Eigen::Vector3f> drawn;
  std::string listening;
  bool listen_error = false;
  double message_rate = 0, kilobytes_rate = 0, age = -1, image_age = -1;
  uint64_t received = 0, lost = 0, malformed = 0;
  size_t trajectory_points = 0;
  // View
  bool follow = true, show_grid = true, show_trajectory = true, show_landmarks = true, show_window = true;
  float yaw = -2.3f, pitch = 0.6f, distance = 25.0f;
  Eigen::Vector3d target = Eigen::Vector3d::Zero();
  std::string screenshot;
};

}  // namespace

int main(int argc, char** argv) {
  std::string ip = "0.0.0.0";
  int port = 5100;
  std::string snapshot;
  double snapshot_delay = 5.0;
  loadListen(&ip, &port);
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string option = argv[i];
    if (option == "--ip") {
      ip = argv[i + 1];
    } else if (option == "--port") {
      port = std::atoi(argv[i + 1]);
    } else if (option == "--snapshot") {
      snapshot = argv[i + 1];
    } else if (option == "--snapshot-delay") {
      snapshot_delay = std::atof(argv[i + 1]);
    } else {
      std::cerr << "Usage: " << argv[0] << " [--ip LISTEN_IP] [--port PORT] [--snapshot FILE.jpg] [--snapshot-delay SECONDS]"
              << std::endl;
      return 2;
    }
  }

  Shared shared;
  Receiver receiver(&shared);
  receiver.listen(ip, port);
  Viewer viewer(&shared, &receiver, ip, port);
  if (!viewer.initialize()) {
    std::cerr << "Cannot open the window (GLFW, OpenGL 3)" << std::endl;
    return 1;
  }
  const auto start = Clock::now();
  while (true) {
    std::string snap;
    if (!snapshot.empty() && std::chrono::duration<double>(Clock::now() - start).count() > snapshot_delay) snap.swap(snapshot);
    if (!viewer.frame(snap)) break;
  }
  return 0;
}
