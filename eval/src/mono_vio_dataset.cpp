// Offline Granite mono VIO runner for an EuRoC folder (granite_vio rejects mono-inertial).
// Wiring follows src/pi_camera_imu_vio.cpp, the live app, which uses the same estimator path as --realtime.
// It reads EuRoC itself (no rosbag/dataset_io dependency).
// Modes:
//   max speed (default): images as fast as the pipeline takes them; vio_enforce_realtime off; after metric
//     initialization the feeder stays at most --max-backlog frames ahead of the estimator output.
//   --realtime: each image is released at its data time (start + t - t0). The flow input holds at most 4 waiting
//     images; a 5th image drops the oldest waiting one (the openvio --realtime policy). vio_enforce_realtime is on
//     (the setting of the live example), so the estimator also skips to the newest flow result.
// Output: <out>_traj_tum.txt (T_w_i), <out>_states.csv (state, support, health, latency), <out>_threads.log (CPU
// per thread). The live app writes the same two files with --out.
#include <dirent.h>
#include <sys/resource.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <thread>

#include <CLI/CLI.hpp>
#include <cereal/archives/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tbb/concurrent_queue.h>
#include <tbb/global_control.h>

#include <granite/optical_flow/optical_flow.h>
#include <granite/serialization/headers_serialization.h>
#include <granite/vi_estimator/mono_vio.h>
#include <granite/vi_estimator/vio_estimator.h>

namespace {
using Clock = std::chrono::steady_clock;

std::set<int> Tasks() {
  std::set<int> tids;
  if (DIR* d = opendir("/proc/self/task")) {
    while (dirent* e = readdir(d))
      if (e->d_name[0] != '.') tids.insert(std::atoi(e->d_name));
    closedir(d);
  }
  return tids;
}

void NameTask(int tid, const std::string& name) {
  std::ofstream f("/proc/self/task/" + std::to_string(tid) + "/comm");
  f << name.substr(0, 15);
}

double TaskCpu(int tid) {
  std::ifstream f("/proc/self/task/" + std::to_string(tid) + "/stat");
  std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  const auto p = s.rfind(')');
  if (p == std::string::npos) return 0;
  std::istringstream is(s.substr(p + 2));
  std::string tok;
  double ut = 0, st = 0;
  for (int i = 3; i <= 15 && is >> tok; ++i) {
    if (i == 14) ut = std::stod(tok);
    if (i == 15) st = std::stod(tok);
  }
  return (ut + st) / sysconf(_SC_CLK_TCK);
}

std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string tok;
  while (std::getline(ss, tok, ',')) out.push_back(tok);
  return out;
}

granite::OpticalFlowInput::Ptr LoadImage(const std::string& path, int64_t t_ns) {
  cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
  if (img.empty()) throw std::runtime_error("cannot read " + path);
  auto in = std::make_shared<granite::OpticalFlowInput>();
  in->t_ns = t_ns;
  in->img_data.resize(1);
  in->img_data[0].img.reset(new granite::ManagedImage<granite::PixelType>(img.cols, img.rows));
  auto& m = *in->img_data[0].img;
  for (int y = 0; y < img.rows; ++y)
    for (int x = 0; x < img.cols; ++x) m(x, y) = img.at<uint8_t>(y, x);
  return in;
}

double Percentile(std::vector<double> v, double p) {
  if (v.empty()) return -1;
  std::sort(v.begin(), v.end());
  return v[std::min(v.size() - 1, static_cast<size_t>(p * (v.size() - 1) + 0.5))];
}
}  // namespace

int main(int argc, char** argv) {
  std::string dataset, calib_path, config_path, out_prefix;
  int num_threads = 4, max_backlog = 8, max_frames = 0, queue_limit = 4;
  bool realtime = false, preload = false;
  int64_t cam_time_offset_ns = 0;
  CLI::App app{"Granite mono VIO on an EuRoC dataset"};
  app.add_option("--dataset-path", dataset)->required();
  app.add_option("--cam-calib", calib_path)->required();
  app.add_option("--config-path", config_path);
  app.add_option("--out", out_prefix)->required();
  app.add_option("--num-threads", num_threads, "TBB max_allowed_parallelism");
  app.add_option("--max-backlog", max_backlog, "Max speed: max frames ahead of the metric estimator");
  app.add_option("--max-frames", max_frames, "Use only the first N images (0: all)");
  app.add_flag("--realtime", realtime, "Release images at their data time; drop the oldest above --queue-limit");
  app.add_option("--queue-limit", queue_limit, "Realtime: max waiting images at the flow input");
  app.add_flag("--preload", preload, "Decode all images before the start (no PNG decoding during the run)");
  app.add_option("--cam-time-offset-ns", cam_time_offset_ns, "Added to image timestamps");
  std::vector<double> init_ba{0, 0, 0}, init_bg{0, 0, 0};
  app.add_option("--init-ba", init_ba, "Initial accelerometer bias (3 values)")->expected(3);
  app.add_option("--init-bg", init_bg, "Initial gyroscope bias (3 values)")->expected(3);
  CLI11_PARSE(app, argc, argv);

  tbb::global_control threads(tbb::global_control::max_allowed_parallelism, num_threads);
  cv::setNumThreads(1);

  granite::VioConfig config;
  if (!config_path.empty()) config.load(config_path);
  config.vio_enforce_realtime = realtime;
  granite::Calibration<double> calib;
  {
    std::ifstream input(calib_path);
    cereal::JSONInputArchive archive(input);
    archive(calib);
  }
  calib.main_cam_idx = {0};
  calib.stereo_pairs.clear();

  // EuRoC: mav0/cam0/data.csv (t_ns,filename), mav0/imu0/data.csv (t_ns,wx,wy,wz,ax,ay,az)
  std::vector<int64_t> image_ts;
  std::vector<std::string> image_files;
  {
    std::ifstream f(dataset + "/mav0/cam0/data.csv");
    std::string line;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') continue;
      auto v = SplitCsv(line);
      std::string name = v.at(1);
      while (!name.empty() && (name.back() == '\r' || name.back() == ' ')) name.pop_back();
      image_ts.push_back(std::stoll(v[0]));
      image_files.push_back(dataset + "/mav0/cam0/data/" + name);
      if (max_frames > 0 && (int)image_ts.size() >= max_frames) break;
    }
  }
  tbb::concurrent_bounded_queue<granite::ImuData::Ptr> imu_queue;
  size_t imu_count = 0;
  {
    std::ifstream f(dataset + "/mav0/imu0/data.csv");
    std::string line;
    const int64_t imu_end = image_ts.back() + 1000000000LL;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') continue;
      auto v = SplitCsv(line);
      auto s = std::make_shared<granite::ImuData>();
      s->t_ns = std::stoll(v[0]);
      if (s->t_ns > imu_end) break;
      s->gyro = Eigen::Vector3d(std::stod(v[1]), std::stod(v[2]), std::stod(v[3]));
      s->accel = Eigen::Vector3d(std::stod(v[4]), std::stod(v[5]), std::stod(v[6]));
      imu_queue.push(s);
      ++imu_count;
    }
    imu_queue.push(nullptr);
  }
  const size_t n = image_ts.size();
  std::cout << "frames=" << n << " imu=" << imu_count << " mode=" << (realtime ? "realtime" : "max")
            << " threads=" << num_threads << " preload=" << preload << std::endl;
  std::vector<granite::OpticalFlowInput::Ptr> preloaded;
  double load_seconds = 0;
  if (preload) {
    const auto t0 = Clock::now();
    for (size_t k = 0; k < n; ++k) preloaded.push_back(LoadImage(image_files[k], image_ts[k] + cam_time_offset_ns));
    std::cout << "preloaded " << n << " images in " << std::chrono::duration<double>(Clock::now() - t0).count()
              << " s" << std::endl;
  }
  std::map<int64_t, size_t> index_of;
  for (size_t k = 0; k < n; ++k) index_of[image_ts[k] + cam_time_offset_ns] = k;

  const auto t_launch = Clock::now();
  std::map<int, std::string> names;
  std::map<int, double> cpu;
  int unnamed = 0;
  auto name_new = [&](const std::string& label) {
    int k = 0;
    for (int tid : Tasks())
      if (!names.count(tid)) {
        const std::string nm =
            label == "t" ? "t" + std::to_string(unnamed++) : label + (k ? std::to_string(k) : "");
        NameTask(tid, nm);
        names[tid] = nm;
        std::cout << "THREAD_NEW tid=" << tid << " name=" << nm << " wall_s="
                  << std::chrono::duration<double>(Clock::now() - t_launch).count() << std::endl;
        ++k;
      }
  };
  NameTask(gettid(), "main");
  name_new("main");

  tbb::concurrent_bounded_queue<granite::VioStateData::Ptr> state_queue;
  state_queue.set_capacity(200);
  auto flow = granite::OpticalFlowFactory::getOpticalFlow(config, calib);
  flow->input_queue.set_capacity(queue_limit);
  name_new("flow");

  auto vio = granite::VioEstimatorFactory::getVioEstimator(config, calib, granite::constants::g, true);
  auto mono = std::dynamic_pointer_cast<granite::MonoVioEstimator>(vio);
  if (!mono) {
    std::cerr << "not mono VIO" << std::endl;
    return 1;
  }
  vio->vision_data_queue = &flow->output_queue;
  vio->imu_data_queue = &imu_queue;
  vio->out_state_queue = &state_queue;

  std::vector<double> release_wall(n, -1.0);
  std::atomic<int64_t> last_state_ns{0};
  std::atomic<size_t> fed{0}, dropped{0};
  std::atomic<bool> feed_done{false};
  double late_max_s = 0;  // realtime: how late the feeder released an image (decode/scheduling)
  const auto start = Clock::now();
  vio->initialize(Eigen::Vector3d(init_bg[0], init_bg[1], init_bg[2]),
                  Eigen::Vector3d(init_ba[0], init_ba[1], init_ba[2]));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  name_new("wrap_vo");  // MonoVioEstimator thread (lower tid) and bootstrap KeypointVoEstimator thread
  const auto rt_start = Clock::now();
  std::thread feeder([&] {
    for (size_t k = 0; k < n; ++k) {
      granite::OpticalFlowInput::Ptr in;
      if (preload) {
        in = preloaded[k];
        preloaded[k].reset();
      } else {
        const auto t0 = Clock::now();
        in = LoadImage(image_files[k], image_ts[k] + cam_time_offset_ns);
        load_seconds += std::chrono::duration<double>(Clock::now() - t0).count();
      }
      if (realtime) {
        const auto due = rt_start + std::chrono::nanoseconds(image_ts[k] - image_ts[0]);
        std::this_thread::sleep_until(due);
        late_max_s = std::max(late_max_s, std::chrono::duration<double>(Clock::now() - due).count());
        release_wall[k] = std::chrono::duration<double>(Clock::now() - start).count();
        while (!flow->input_queue.try_push(in)) {
          granite::OpticalFlowInput::Ptr old;
          if (flow->input_queue.try_pop(old)) ++dropped;
        }
      } else {
        if (mono->metricInitialized() && k >= (size_t)max_backlog) {
          const int64_t need = image_ts[k - max_backlog] + cam_time_offset_ns;
          const auto t0 = Clock::now();
          while (last_state_ns.load() < need && Clock::now() - t0 < std::chrono::seconds(2))
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
        release_wall[k] = std::chrono::duration<double>(Clock::now() - start).count();
        flow->input_queue.push(in);
      }
      fed = k + 1;
    }
    flow->input_queue.push(nullptr);
    feed_done = true;
  });

  std::ofstream tum(out_prefix + "_traj_tum.txt"), csv(out_prefix + "_states.csv"),
      thr(out_prefix + "_threads.log");
  tum << std::setprecision(19);
  csv << "t_ns,wall_s,x,y,z,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,support,connected_finite,connected_infinite,"
         "landmarks,map_idx,prev_corr_rot,prev_corr_pos,healthy,latency_ms\n"
      << std::setprecision(12);
  size_t states = 0;
  double first_state_wall = -1;
  std::vector<double> latencies;
  std::string support = "init";
  auto report = start;
  auto consume = [&] {
    granite::VioStateData::Ptr r;
    while (state_queue.try_pop(r)) {
      if (!r) continue;
      const auto& s = r->state;
      const double wall = std::chrono::duration<double>(Clock::now() - start).count();
      if (first_state_wall < 0) first_state_wall = wall;
      ++states;
      last_state_ns = s.t_ns;
      double latency_ms = -1;
      auto it = index_of.find(s.t_ns);
      if (it != index_of.end() && release_wall[it->second] >= 0) {
        latency_ms = 1e3 * (wall - release_wall[it->second]);
        latencies.push_back(latency_ms);
      }
      const auto& p = s.T_w_i.translation();
      const auto q = s.T_w_i.unit_quaternion();
      tum << s.t_ns * 1e-9 << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' ' << q.x() << ' ' << q.y() << ' '
          << q.z() << ' ' << q.w() << '\n';
      int sup = static_cast<int>(r->mono_vio_support);
      support = sup == 1 ? "metric" : sup == 2 ? "bearing" : sup == 3 ? "inertial" : "n/a";
      csv << s.t_ns << ',' << wall << ',' << p.x() << ',' << p.y() << ',' << p.z() << ',' << q.x() << ',' << q.y()
          << ',' << q.z() << ',' << q.w() << ',' << s.vel_w_i.x() << ',' << s.vel_w_i.y() << ',' << s.vel_w_i.z()
          << ',' << s.bias_gyro.x() << ',' << s.bias_gyro.y() << ',' << s.bias_gyro.z() << ','
          << s.bias_accel.x() << ',' << s.bias_accel.y() << ',' << s.bias_accel.z() << ',' << support << ','
          << r->num_connected_finite_landmarks << ',' << r->num_connected_infinite_landmarks << ','
          << r->num_landmarks << ',' << r->map_idx << ',' << r->prev_correction_rot << ','
          << r->prev_correction_pos << ',' << r->healthy << ',' << latency_ms << '\n';
    }
  };
  name_new("feeder");
  auto sample_cpu = Clock::now();
  while (!vio->isFinished()) {
    consume();
    name_new("t");
    const auto now = Clock::now();
    if (now - sample_cpu >= std::chrono::milliseconds(500)) {
      sample_cpu = now;
      for (int tid : Tasks()) cpu[tid] = TaskCpu(tid);
    }
    if (now - report >= std::chrono::seconds(5)) {
      report = now;
      std::cout << std::fixed << std::setprecision(1) << "wall=" << std::chrono::duration<double>(now - start).count()
                << " fed=" << fed.load() << " dropped=" << dropped.load() << " states=" << states
                << " metric=" << mono->metricInitialized() << " support=" << support
                << " t_state=" << (last_state_ns ? (last_state_ns - image_ts.front()) * 1e-9 : -1.0) << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  const double wall_end = std::chrono::duration<double>(Clock::now() - start).count();
  {
    granite::OpticalFlowResult::Ptr res;
    auto t0 = Clock::now();
    while (Clock::now() - t0 < std::chrono::milliseconds(500)) {
      while (flow->output_queue.try_pop(res)) t0 = Clock::now();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  consume();
  double total_cpu = 0;
  thr << "tid name cpu_s\n";
  for (int tid : Tasks()) cpu[tid] = TaskCpu(tid);
  for (const auto& kv : cpu) {
    total_cpu += kv.second;
    thr << kv.first << ' ' << (names.count(kv.first) ? names[kv.first] : "?") << ' ' << kv.second << '\n';
  }
  thr << "sampled_threads_total " << total_cpu << " (exited threads: last 0.5 s sample)\n";
  thr.close();
  feeder.join();
  flow->join();
  vio->join();
  rusage ru{};
  getrusage(RUSAGE_SELF, &ru);
  std::cout << std::setprecision(6) << "SUMMARY mode=" << (realtime ? "realtime" : "max") << " wall_s=" << wall_end
            << " frames=" << n << " fed=" << fed.load() << " dropped_input=" << dropped.load()
            << " states=" << states << " first_state_wall_s=" << first_state_wall
            << " metric_initialized=" << mono->metricInitialized() << " image_load_s=" << load_seconds
            << " ms_per_frame=" << 1e3 * wall_end / n << " latency_ms_p50=" << Percentile(latencies, 0.5)
            << " latency_ms_p95=" << Percentile(latencies, 0.95) << " latency_ms_max=" << Percentile(latencies, 1.0)
            << " feeder_late_max_ms=" << 1e3 * late_max_s
            << " user_s=" << ru.ru_utime.tv_sec + ru.ru_utime.tv_usec * 1e-6
            << " sys_s=" << ru.ru_stime.tv_sec + ru.ru_stime.tv_usec * 1e-6 << " maxrss_mb=" << ru.ru_maxrss / 1024.0
            << std::endl;
  return 0;
}
