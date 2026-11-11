# UVIFY VIO Viewer

UVIFY VIO Viewer is a standalone ground viewer for the Granite VIO. It receives the visualization messages that the VIO sends over UDP (`send_vis_msg`, message `GraniteVis` in `doc/granite_vis.proto`) and shows them. It does not link libgranite.

## Structure

| File | Contents |
|---|---|
| `vio_viewer.cpp` | The receiver thread (UDP, JPEG decode) and the GUI (ImGui and ImPlot on GLFW and OpenGL 3) |
| `granite_vis_message.h` | The `GraniteVis` decoder (protobuf wire format, no protobuf library) |
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
- The AppImage contains libglfw and libjpeg. libstdc++ is in the binary. OpenGL, X11 and glibc come from the host. The host must have glibc 2.38 or later (Ubuntu 24.04).

## Use

```bash
./UVIFY_VIO_Viewer-x86_64.AppImage                    # listens on 0.0.0.0:5100
./UVIFY_VIO_Viewer-x86_64.AppImage --ip 10.20.10.42 --port 5100
```

- The **Listen (UDP)** box sets the IP address and the port. Type the values, then push **Apply** or Enter. 0.0.0.0 listens on all interfaces.
- **Apply** saves the address and the port in `~/.config/uvify-vio-viewer/listen.conf`. The next start uses them. `--ip` and `--port` override them for one run.
- `--snapshot FILE.jpg` writes the window as a JPEG after `--snapshot-delay` seconds (default 5).

## Panels

| Panel | Contents |
|---|---|
| Status | The listen address, the message rate [kbit/s], lost and skipped messages, the VIO health, position, speed and biases, the view switches |
| Image | The camera image of the newest image message, the keypoints (green) and the landmark projections (red near, blue far) |
| 3D view | The trajectory, the states of the optimization window, the landmarks of the window, the current pose axes and the velocity. Left drag rotates, right drag pans, the wheel zooms |
| Plots | The speed and the x, y, z position over time |

- The status shows the time since the last message. After 1 s without a message, the text is red. After 1 s without a new image, the image panel shows a red bar.
- A smaller `seq` than the last message means a new VIO process. The viewer then clears the trajectory and the plots.

## Memory

The memory of the viewer does not grow with the run time.

- The viewer keeps only the newest image message (image, keypoints, projections, landmarks). The landmarks do not accumulate.
- The trajectory keeps at most 200000 positions. The plots keep at most 12000 samples (10 minutes at 20 Hz). When a buffer is full, the viewer removes the oldest 10 %.
- The 3D view draws at most 20000 trajectory segments.

Measured result [desktop, 1 run]: a synthetic sender sent 5000 state messages per second for 120 s, with an image message at every 100th message. The trajectory buffer was full after about 40 s. From 45 s to 120 s, the RSS of the viewer stayed at 104.1 MB. After the sender stopped, the RSS stayed at 110.2 MB.
