/**
MIT License

This file is part of the Granite project which is based on Basalt.
https://github.com/DLR-RM/granite

Copyright (c) Martin Wudenka, Deutsches Zentrum für Luft- und Raumfahrt

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
Original license of Basalt:
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "vio_app.h"

int main(int argc, char** argv) {
  bool show_gui = true;
  bool print_queue = false;
  bool step_by_step = false;
  bool use_imu = true;
  std::string cam_calib_path;
  std::string dataset_path;
  std::string dataset_type;
  std::string config_path;
  std::string result_path;
  std::string stats_path;
  std::string trajectory_fmt;
  std::string marg_data_path;
  int num_threads = 0;
  VioRunOptions run;
  bool resize_half = false;

  CLI::App app{"Granite VIO with the Pangolin GUI: a dataset, or the live Raspberry Pi camera and IMU"};

  app.add_option("--show-gui", show_gui, "Show GUI");
  app.add_option("--cam-calib", cam_calib_path,
                 "Camera and IMU calibration JSON (live: at the sensor mode or at 1/downscale of it)")
      ->required();

  app.add_option("--dataset-path", dataset_path, "Path to dataset.");

  app.add_option("--dataset-type", dataset_type, "Dataset type <euroc, bag>.");

  app.add_option("--marg-data", marg_data_path,
                 "Path to folder where marginalization data will be stored.");

  app.add_option("--print-queue", print_queue, "Print queue.");
  app.add_option("--config-path", config_path, "Path to config file.");
  app.add_option("--result-path", result_path,
                 "Path to result file where the system will write RMSE ATE.");
  app.add_option("--stats-path", stats_path,
                 "Path to stats file where the system will write some stats.");
  app.add_option("--num-threads", num_threads, "Number of threads.");
  app.add_option("--step-by-step", step_by_step, "Do not start automatically.");
  app.add_option("--save-trajectory", trajectory_fmt,
                 "Save trajectory. Supported formats <tum, euroc, kitti>");
  app.add_option("--use-imu", use_imu, "Use IMU.");

  // Realtime dataset run (eval/src/mono_vio_dataset.cpp --realtime) and outputs
  app.add_flag("--realtime", run.realtime,
               "Release the images at their data time; drop the oldest above --queue-limit");
  app.add_option("--queue-limit", run.queue_limit,
                 "Images that wait at the flow input (realtime and live)");
  app.add_option("--cam-time-offset-ns", run.cam_time_offset_ns, "Added to the image times of a dataset");
  app.add_option("--max-frames", run.max_frames, "Use only the first N images of a dataset (0: all)");
  app.add_option("--out", run.out_prefix, "Output prefix: <out>_traj_tum.txt and <out>_states.csv");

  // Live run
  app.add_flag("--live", run.live,
               "Use the Raspberry Pi camera and IMU instead of a dataset (camera.* keys of --config-path)");
  app.add_option("--record", run.record_path, "Live: record the images and the IMU as an EuRoC dataset");
  app.add_option("--duration", run.duration, "Live: stop after this many seconds (0: until Ctrl-C or window close)");
#ifdef GRANITE_WITH_PI_CAMERA
  granite::pi_live::addSensorOptions(app, run.pi, resize_half);
#endif

  // Visualization messages for a viewer on the ground (keys of the same names in --config-path)
  app.add_option("--send_vis_msg", run.vis.send_vis_msg,
                 "Send the GUI elements of each state as one UDP message (doc/granite_vis.proto)");
  app.add_option("--vis_msg_ip", run.vis.vis_msg_ip, "send_vis_msg: IPv4 address of the viewer");
  app.add_option("--vis_msg_port", run.vis.vis_msg_port, "send_vis_msg: UDP port of the viewer");
  app.add_option("--vis_msg_image_hz", run.vis.vis_msg_image_hz,
                 "send_vis_msg: camera images (JPEG) per second at most (0: no image)");
  app.add_option("--vis_msg_jpeg_quality", run.vis.vis_msg_jpeg_quality, "send_vis_msg: JPEG quality (1-100)");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  if (run.live == !dataset_path.empty() || (!run.live && dataset_type.empty())) {
    std::cerr << "Give --live, or --dataset-path and --dataset-type" << std::endl;
    return 2;
  }
  if (run.queue_limit < 1 || run.duration < 0) {
    std::cerr << "Invalid queue limit or duration" << std::endl;
    return 2;
  }

  // global thread limit is in effect until global_control object is destroyed
  std::unique_ptr<tbb::global_control> tbb_global_control;
  if (num_threads > 0) {
    tbb_global_control = std::make_unique<tbb::global_control>(
        tbb::global_control::max_allowed_parallelism, num_threads);
  }

  // The window needs a display; a headless SSH session has no DISPLAY.
  if (show_gui && !std::getenv("DISPLAY")) {
    std::cout << "No DISPLAY: running without the GUI" << std::endl;
    show_gui = false;
  }

  if (!show_gui) {
    step_by_step = false;
  }

  std::signal(SIGINT, granite_vio_interrupt);
  std::signal(SIGTERM, granite_vio_interrupt);

  try {
    if (!config_path.empty()) loadVisPublisherConfig(config_path, app, run.vis);
#ifdef GRANITE_WITH_PI_CAMERA
    if (run.live) {
      if (resize_half && app.count("--downscale") == 0) run.pi.downscale = 2;
      if (!config_path.empty()) granite::pi_live::loadCameraConfig(config_path, app, run.pi);
    }
#endif
    VioDataset vd(cam_calib_path, config_path, dataset_type, dataset_path,
                  marg_data_path, result_path, stats_path, trajectory_fmt,
                  show_gui, use_imu, step_by_step, print_queue, run);

    vd.run();

    std::cout << "Finish" << std::endl;
    return vd.liveFailed() ? 1 : 0;
  } catch (const std::exception& error) {
    std::cerr << "granite_vio: " << error.what() << std::endl;
    return 1;
  }
}
