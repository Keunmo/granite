// Offline Granite mono VIO runner for an EuRoC folder (granite_vio rejects mono-inertial).
// Wiring follows src/pi_camera_imu_vio.cpp. Images are fed as fast as the pipeline takes them.
// Output: TUM trajectory of T_w_i, CSV of state/support per state, per-thread CPU time at the end.
#include <dirent.h>
#include <sys/resource.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <thread>

#include <CLI/CLI.hpp>
#include <cereal/archives/json.hpp>
#include <opencv2/core.hpp>
#include <tbb/concurrent_queue.h>
#include <tbb/global_control.h>

#include <granite/io/dataset_io.h>
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

// utime + stime [s] of a thread
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
}  // namespace

int main(int argc, char** argv) {
  std::string dataset, calib_path, config_path, out_prefix;
  int num_threads = 4, max_backlog = 8;
  double sigma_scale_acc = 1.0, sigma_scale_gyro = 1.0;
  int64_t cam_time_offset_ns = 0;
  CLI::App app{"Granite mono VIO on an EuRoC dataset"};
  app.add_option("--dataset-path", dataset)->required();
  app.add_option("--cam-calib", calib_path)->required();
  app.add_option("--config-path", config_path);
  app.add_option("--out", out_prefix)->required();
  app.add_option("--num-threads", num_threads);
  app.add_option("--max-backlog", max_backlog, "Max frames ahead of the metric estimator");
  app.add_option("--cam-time-offset-ns", cam_time_offset_ns, "Added to image timestamps");
  std::vector<double> init_ba{0, 0, 0}, init_bg{0, 0, 0};
  app.add_option("--init-ba", init_ba, "Initial accelerometer bias (3 values)")->expected(3);
  app.add_option("--init-bg", init_bg, "Initial gyroscope bias (3 values)")->expected(3);
  CLI11_PARSE(app, argc, argv);

  tbb::global_control threads(tbb::global_control::max_allowed_parallelism, num_threads);
  cv::setNumThreads(1);
  NameTask(gettid(), "main");

  granite::VioConfig config;
  if (!config_path.empty()) config.load(config_path);
  config.vio_enforce_realtime = false;  // dataset: process every frame
  granite::Calibration<double> calib;
  {
    std::ifstream input(calib_path);
    cereal::JSONInputArchive archive(input);
    archive(calib);
  }
  calib.main_cam_idx = {0};
  calib.stereo_pairs.clear();
  UNUSED(sigma_scale_acc);
  UNUSED(sigma_scale_gyro);

  auto io = granite::DatasetIoFactory::getDatasetIo("euroc");
  if (!io->read(dataset, 1)) return 1;
  auto data = io->get_data();
  const auto& image_ts = data->get_image_timestamps();
  std::cout << "frames=" << image_ts.size() << " imu=" << data->get_gyro_data().size() << std::endl;

  const auto t_launch = Clock::now();
  std::map<int, std::string> names;
  std::map<int, double> cpu;
  int unnamed = 0;
  auto name_new = [&](const std::string& label) {
    int k = 0;
    for (int tid : Tasks())
      if (!names.count(tid)) {
        const std::string n = label == "t" ? "t" + std::to_string(unnamed++) : label + (k ? std::to_string(k) : "");
        NameTask(tid, n);
        names[tid] = n;
        std::cout << "THREAD_NEW tid=" << tid << " name=" << n << " wall_s="
                  << std::chrono::duration<double>(Clock::now() - t_launch).count() << std::endl;
        ++k;
      }
  };
  name_new("main");

  tbb::concurrent_bounded_queue<granite::ImuData::Ptr> imu_queue;
  tbb::concurrent_bounded_queue<granite::VioStateData::Ptr> state_queue;
  state_queue.set_capacity(200);
  auto flow = granite::OpticalFlowFactory::getOpticalFlow(config, calib);
  flow->input_queue.set_capacity(4);
  name_new("flow");

  const auto& gyro = data->get_gyro_data();
  const auto& acc = data->get_accel_data();
  for (size_t i = 0; i < gyro.size(); ++i) {
    auto s = std::make_shared<granite::ImuData>();
    s->t_ns = gyro[i].timestamp_ns;
    s->gyro = gyro[i].data;
    s->accel = acc[i].data;
    imu_queue.push(s);
  }
  imu_queue.push(nullptr);

  auto vio = granite::VioEstimatorFactory::getVioEstimator(config, calib, granite::constants::g, true);
  auto mono = std::dynamic_pointer_cast<granite::MonoVioEstimator>(vio);
  if (!mono) { std::cerr << "not mono VIO" << std::endl; return 1; }
  vio->vision_data_queue = &flow->output_queue;
  vio->imu_data_queue = &imu_queue;
  vio->out_state_queue = &state_queue;

  std::atomic<int64_t> last_state_ns{0};
  std::atomic<size_t> fed{0};
  std::atomic<bool> feed_done{false};
  double load_seconds = 0;
  const auto start = Clock::now();
  vio->initialize(Eigen::Vector3d(init_bg[0], init_bg[1], init_bg[2]),
                  Eigen::Vector3d(init_ba[0], init_ba[1], init_ba[2]));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  name_new("wrap_vo");  // MonoVioEstimator thread (lower tid) and bootstrap KeypointVoEstimator thread
  std::thread feeder([&] {
    for (size_t k = 0; k < image_ts.size(); ++k) {
      // Keep the unbounded metric queue short: wait for the estimator after metric initialization.
      if (mono->metricInitialized() && k >= (size_t)max_backlog) {
        const int64_t need = image_ts[k - max_backlog] + cam_time_offset_ns;
        const auto t0 = Clock::now();
        while (last_state_ns.load() < need && Clock::now() - t0 < std::chrono::seconds(2))
          std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
      const auto t0 = Clock::now();
      auto in = std::make_shared<granite::OpticalFlowInput>();
      in->t_ns = image_ts[k] + cam_time_offset_ns;
      in->img_data = data->get_image_data(image_ts[k]);
      load_seconds += std::chrono::duration<double>(Clock::now() - t0).count();
      flow->input_queue.push(in);
      fed = k + 1;
    }
    flow->input_queue.push(nullptr);
    feed_done = true;
  });

  std::ofstream tum(out_prefix + "_traj_tum.txt"), csv(out_prefix + "_states.csv"),
      thr(out_prefix + "_threads.log");
  tum << std::setprecision(19);
  csv << "t_ns,wall_s,x,y,z,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,support,connected_finite,connected_infinite,"
         "landmarks,map_idx,prev_corr_rot,prev_corr_pos\n"
      << std::setprecision(12);
  size_t states = 0;
  double first_state_wall = -1;
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
          << r->num_landmarks << ',' << r->map_idx << ',' << r->prev_correction_rot << ',' << r->prev_correction_pos << '\n';
    }
  };
  bool flow_input_closed = false;
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
      std::cout << std::fixed << std::setprecision(1)
                << "wall=" << std::chrono::duration<double>(now - start).count() << " fed=" << fed.load()
                << " states=" << states << " metric=" << mono->metricInitialized() << " support=" << support
                << " t_state=" << (last_state_ns ? (last_state_ns - image_ts.front()) * 1e-9 : -1.0) << std::endl;
    }
    if (feed_done && !flow_input_closed) flow_input_closed = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  const double wall_end = std::chrono::duration<double>(Clock::now() - start).count();
  // Drain flow output if the estimator stopped first (see pi_camera_imu_vio.cpp)
  {
    granite::OpticalFlowResult::Ptr res;
    auto t0 = Clock::now();
    while (Clock::now() - t0 < std::chrono::milliseconds(500)) {
      while (flow->output_queue.try_pop(res)) t0 = Clock::now();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  consume();
  // per-thread CPU before joining
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
  std::cout << std::setprecision(6) << "SUMMARY wall_s=" << wall_end << " frames=" << image_ts.size()
            << " fed=" << fed.load() << " states=" << states << " first_state_wall_s=" << first_state_wall
            << " metric_initialized=" << mono->metricInitialized() << " image_load_s=" << load_seconds
            << " ms_per_frame=" << 1e3 * wall_end / image_ts.size()
            << " user_s=" << ru.ru_utime.tv_sec + ru.ru_utime.tv_usec * 1e-6
            << " sys_s=" << ru.ru_stime.tv_sec + ru.ru_stime.tv_usec * 1e-6 << " maxrss_mb=" << ru.ru_maxrss / 1024.0
            << std::endl;
  return 0;
}
