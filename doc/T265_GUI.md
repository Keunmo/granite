# T265 live GUI

The live GUI opens by default in all four T265 runners:

- `granite_rs_t265_vio`
- `granite_rs_t265_mono_vio`
- `granite_rs_t265_vo`
- `granite_rs_t265_mono_vo`

Use `--no-gui` for headless operation. Its default value is `false`.
`--no-gui=false` explicitly enables the GUI. These options also apply to
`--capture-only`.

The GUI uses OpenCV highgui. It works with `GRANITE_BUILD_GUI=OFF`, which
controls the separate Pangolin applications. A GUI-enabled OpenCV build and
an accessible desktop display are required to open the window. Headless
operation does not initialize highgui or create a window.

## Display

The upper panels show the selected fisheye images. Mono mode uses the first
panel for the selected left or right fisheye. Capture-only mode shows images
without estimator overlays.

- Green dots show tracked features.
- Colored rings show finite landmark projections. Blue indicates farther
  landmarks. Red indicates nearer landmarks.
- Blue squares show infinite landmark projections.

Colors use the estimated inverse distance to the camera. The color range
saturates at inverse distance 1. Metric VIO expresses distance in meters.
Mono VO uses its current visual scale.

The lower panels show XY and XZ trajectory projections. Green marks the
oldest displayed pose. Red marks the latest pose. The display retains up to
6000 poses. A map reset clears the trajectory.

The status panel shows the runner mode, tracking state, initialization,
input and output rates, image lag, dropped frames, landmark counts, and
position. Mono VIO also reports `metric`, `bearing only`, or `inertial only`
support. Connected finite and infinite counts describe the current frame.
A bearing-only update constrains rotation. An inertial-only update propagates
the state without a visual constraint. Metric initialization remains a
separate status.

The main loop drains bounded state and visualization queues. It retains
only the latest display image and visualization. The window refreshes at
most about 30 times per second. Trajectory CSV output retains each consumed
state, including states skipped by the display refresh.

## Stop

Press Escape or Q, or close the window, to stop the runner. SIGINT, SIGTERM,
and `--duration` also stop it. Shutdown stops SDK callbacks, drains estimator
output, and joins the workers before releasing the window.

## USB permission errors

On Linux, the runner checks USB event netlink access before it creates the
RealSense device. A sandbox can deny this access with `EPERM` or `EACCES`.
The installed librealsense 2.53.1 can crash after `libusb_init` fails in this
case. The runner reports `USB access blocked: netlink permission denied`
instead. Run outside the restricted sandbox or permit
`AF_NETLINK/NETLINK_KOBJECT_UEVENT` access. `--no-gui` disables the display;
it still requires USB access.
