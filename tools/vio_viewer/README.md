# UVIFY VIO Viewer

UVIFY VIO Viewer is a standalone ground viewer for the Granite VIO. It receives the visualization messages that the VIO sends over UDP (`send_vis_msg`, message `GraniteVis` in `doc/granite_vis.proto`) and shows them. It does not link libgranite.

## Structure

| File | Contents |
|---|---|
| `vio_viewer.cpp` | The receiver thread (UDP, JPEG decode) and the GUI (ImGui and ImPlot on GLFW and OpenGL 3) |
| `granite_vis_message.h` | The `GraniteVis` version 2 decoder (protobuf wire format, no protobuf library) |
| `camera_projection.h` | The projection of the granite camera models `pinhole`, `kb4`, `ds`, `eucm` and `ucm` |
| `h264_decoder.h` | The H.264 decoder (OpenH264) |
| `CMakeLists.txt` | The build of the viewer and of the ImGui and ImPlot static library |
| `build_appimage.sh` | The build and the AppImage package |
| `uvify-vio-viewer.desktop` | The desktop entry of the AppImage |
| `thirdparty/imgui`, `thirdparty/implot` | ImGui and ImPlot, the same copies as openvio (MIT licenses in the directories) |

## Build

```bash
sudo apt install -y libglfw3-dev libjpeg-dev
tools/vio_viewer/build_appimage.sh        # tools/vio_viewer/dist/UVIFY_VIO_Viewer-x86_64.AppImage
```

- The build directory is `build_viewer` in the repository (`BUILD_DIR` changes it). The build uses 6 jobs (`JOBS` changes it).
- `appimagetool` must be in `PATH`, or `APPIMAGETOOL` must give its path.
- The build needs OpenH264 (BSD): the Ubuntu 24.04 packages `libopenh264-dev` and `libopenh264-7` (2.4.1). Without root, extract them with `dpkg-deb -x` into `~/cross/openh264-amd64` (`OPENH264_ROOT` changes the folder). With the packages installed, use `OPENH264_ROOT=/`.
- The AppImage contains libglfw, libjpeg and libopenh264, so it decodes JPEG and H.264 images on another PC without more packages. OpenGL, X11, glibc and libstdc++ come from the host. The host must have glibc 2.38 or later (Ubuntu 24.04).
- OpenH264 decodes the constrained baseline profile. granite encodes with this profile.

## Use

```bash
./UVIFY_VIO_Viewer-x86_64.AppImage                    # listens on 0.0.0.0:5100
./UVIFY_VIO_Viewer-x86_64.AppImage --ip 192.168.168.102 --port 5100
```

- The **Listen (UDP)** box sets the IP address and the port. Type the values, then push **Apply** or Enter. 0.0.0.0 listens on all interfaces.
- **Apply** saves the address and the port in `~/.config/uvify-vio-viewer/listen.conf`. The next start uses them. `--ip` and `--port` override them for one run.
- `--snapshot FILE.jpg` writes the window as a JPEG after `--snapshot-delay` seconds (default 5).

## Panels

| Panel | Contents |
|---|---|
| Status | The listen address, the message rate [kbit/s], lost and skipped messages, the VIO health, position, speed and biases, the view switches |
| Image | The camera image of the newest image message (JPEG or H.264), the keypoints (green) and the landmark projections (as the granite GUI: from the nearest landmark of the image, red, to the farthest, blue) |
| 3D view | The trajectory, the states of the optimization window, the landmarks of the window, the current pose axes and the velocity. Left drag rotates, right drag pans, the wheel zooms |
| Plots | The speed and the x, y, z position over time |

- The status shows the time since the last message. After 1 s without a message, the text is red. After 1 s without a new image, the image panel shows a red bar.
- A smaller `seq` than the last message means a new VIO process. The viewer then clears the trajectory and the plots.
- The messages have no landmark projections. The viewer computes them: for each keypoint with a landmark of the same id, it moves the landmark into the camera frame with `(T_w_i * T_i_c)^-1` of the message and projects it with the camera model of the message. The landmarks come at `vis_msg_points_hz` (2 Hz), so a projection uses landmarks that are at most about 0.5 s old.
- H.264: after a lost image (a gap in `image_seq`), the viewer waits for the next IDR frame (at most 1 s).
- A large message comes in fragments of 1400 bytes. The viewer joins them. It keeps at most 4 incomplete messages; a lost fragment loses its message.

## Memory

The memory of the viewer does not grow with the run time.

- The viewer keeps only the newest image message (image, keypoints, projections) and the newest landmarks. The landmarks do not accumulate.
- The trajectory keeps at most 400000 positions (about 5.5 hours at 20 Hz). The plots keep at most 24000 samples (20 minutes at 20 Hz). When a buffer is full, the viewer removes the oldest 10 %.
- The 3D view draws at most 20000 trajectory segments.

Measured result [desktop, 1 run, the earlier limits of 200000 positions and 12000 samples]: a synthetic sender sent 5000 state messages per second for 120 s, with an image message at every 100th message. The trajectory buffer was full after about 40 s. From 45 s to 120 s, the RSS of the viewer stayed at 104.1 MB. After the sender stopped, the RSS stayed at 110.2 MB.
