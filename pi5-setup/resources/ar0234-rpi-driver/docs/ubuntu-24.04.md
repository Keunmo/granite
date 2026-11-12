# AR0234 on Raspberry Pi 5 with Ubuntu 24.04

This document tells you how to install the AR0234 driver on a Raspberry Pi 5
that runs Ubuntu 24.04 (arm64). It also tells you how to show a live image
without libcamera.

The main [README](../README.md) is for Raspberry Pi OS. Do not use its
install steps on Ubuntu. Use the steps in this document.

## Tested setup

| Item | Value |
|---|---|
| Board | Raspberry Pi 5 Model B Rev 1.0 |
| OS | Ubuntu 24.04.5 LTS (arm64), GNOME desktop |
| Kernel | `6.8.0-1065-raspi` |
| Camera | Kurokesu AR0234 color module, 2 lanes, on the CAM/DISP 1 port |
| Driver | `ar0234-rpi-dkms` 0.1.2 with the changes on this branch |

## Differences from the Raspberry Pi OS procedure

| README step | Ubuntu 24.04 | Reason |
|---|---|---|
| `apt full-upgrade` | Do not do it. | It is not necessary for the driver. It can change the kernel and other packages on the system. |
| Kurokesu apt archive | Do not add it. | The archive is for Raspberry Pi OS. It replaces the system `libcamera` and `rpicam-apps`. |
| `ar0234-rpi-dkms` package | Build from source with `setup.sh`. | The package is for Raspberry Pi OS. |
| `/boot/overlays/` | `/boot/firmware/overlays/` | Ubuntu has no `/boot/overlays`. `dkms.postinst` on this branch selects the correct directory. |
| `rpicam-hello` | Use `examples/ar0234_view.py`. | The Ubuntu `libcamera` 0.2 does not support the AR0234. |

## Changes on this branch

1. `ar0234.c`: The driver starts the sensor on kernels older than 6.11.
   Before 6.11, `v4l2_subdev_s_stream_helper()` makes the stream mask from
   the routing table only. This driver has no routing table, so the mask is
   zero. The sensor does not start, and no frames come in. `STREAMON`
   returns success, but the sensor stays in runtime suspend. The new
   `ar0234_s_stream()` calls the enable and disable functions directly.
   Kernel 6.11 and later use the original helper.
2. `dkms.postinst`: The script installs the overlay to
   `/boot/firmware/overlays/` when `/boot/overlays` does not exist.
   On Raspberry Pi OS, the behavior does not change.
3. `examples/ar0234_view.py`: A small live viewer. It uses only `numpy`,
   `tkinter` and `Pillow`.

## Install

Connect the camera to the CSI port when the Pi is off.

1. Install the kernel headers for the kernel that runs now:

   ```bash
   sudo apt update
   sudo apt install -y linux-headers-$(uname -r)
   ```

2. Install the build tools:

   ```bash
   sudo apt install -y git device-tree-compiler
   sudo apt install -y --no-install-recommends dkms
   ```

3. Install the tools and Python packages for the viewer:

   ```bash
   sudo apt install -y v4l-utils python3-numpy python3-tk python3-pil python3-pil.imagetk
   ```

4. Get the source on this branch:

   ```bash
   cd ~
   git clone -b rpi5-ubuntu2404 <repository-url> ar0234-rpi-driver
   cd ar0234-rpi-driver/
   ```

5. Build and install the driver and the overlay:

   ```bash
   sudo ./setup.sh
   ```

   Make sure that the output has this line:

   ```
   INSTALL /boot/firmware/overlays/ar0234.dtbo
   ```

6. Make a copy of the boot configuration:

   ```bash
   sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.bak-ar0234
   ```

7. Edit `/boot/firmware/config.txt`. Set `camera_auto_detect=0`.
8. In the last `[all]` section, add `dtoverlay=ar0234`.
   If the camera is on the CAM/DISP 0 port, add `dtoverlay=ar0234,cam0`.
9. If the file has an other camera overlay for the same port, put `#`
   at the start of that line. Example: `#dtoverlay=arducam-pivariety`.
10. Reboot:

    ```bash
    sudo reboot
    ```

DKMS builds the module again when a new kernel is installed. Install the
headers for the new kernel before you reboot into it.

## Verify the sensor

1. Make sure that the module is loaded:

   ```bash
   lsmod | grep ar0234
   ```

2. Make sure that the media graph has the sensor:

   ```bash
   for m in /dev/media*; do media-ctl -d $m -p | grep -q "ar0234" && echo $m; done
   ```

   The command shows the media device of the `rp1-cfe`. On the tested
   setup, it is `/dev/media0`.

3. Make sure that the kernel log has these lines:

   ```bash
   sudo dmesg | grep rp1-cfe
   ```

   ```
   rp1-cfe 1f00128000.csi: found subdevice /axi/pcie@120000/rp1/i2c@80000/ar0234@10
   rp1-cfe 1f00128000.csi: Using sensor ar0234 11-0010 for capture
   ```

## Show a live image

Run the viewer in a terminal on the Pi desktop:

```bash
python3 examples/ar0234_view.py                         # 960x600, 30 fps
python3 examples/ar0234_view.py --mode 1920x1200 --fps 15
python3 examples/ar0234_view.py --hflip --vflip            # rotate 180 deg
```

Modes: `960x600` (2x2 binning), `1280x720`, `1920x1080`, `1920x1200`.

`--hflip` mirrors the image left-right. `--vflip` mirrors the image up-down.
Use the two options together to rotate the image 180 degrees. The sensor
does the flip, so the CPU load does not change. The sensor and the driver
cannot rotate the image 90 degrees.

To start the viewer from SSH on the Pi desktop, set the display first:

```bash
export DISPLAY=:0
export XAUTHORITY=$(ls /run/user/$(id -u)/.mutter-Xwaylandauth.* | head -1)
```

The viewer does these steps:

1. It finds the media device that has the AR0234.
2. It sets the formats on the media pipeline.
3. It sets `vertical_blanking` for the frame rate that you select.
4. It reads RAW10 frames from `v4l2-ctl --stream-to=-`.
5. It applies a gray-world white balance to the RAW values. The gains
   change slowly.
6. It makes an RGB image of the visible area.

Use the mouse to examine the image:

- Turn the mouse wheel to zoom in or out at the cursor (1x to 16x).
- Hold the left button and move the mouse to move the image.
- Double-click to show the full image again.

Push `q` or `Esc` to stop the viewer.

When the zoom shows more than half of the sensor width, the viewer makes
a half-size image. At a higher zoom, the viewer does a full-size bilinear
demosaic of the visible area only. At 3x scale or more, each sensor pixel
shows as a block.

The viewer has two sliders:

- `exposure [ms]`: The default is 8.33 ms (1/120 s). With 60 Hz mains,
  lamps flicker at 120 Hz. An exposure of N x 8.33 ms gives the same
  brightness in each frame. With 50 Hz mains, use N x 10 ms.
- `analogue_gain`: The sensor register value (13 to 64).

The viewer does not apply lens shading correction, denoise or gamma.
The image quality is lower than the image from `rpicam-apps`.

## Capture RAW frames without the viewer

Use these commands to set the pipeline and capture frames to a file.
The example uses the 960x600 mode and `/dev/media0`.

```bash
M=/dev/media0
F="SGRBG10_1X10/960x600 field:none colorspace:raw xfer:none ycbcr:601 quantization:full-range"
media-ctl -d $M -l '"csi2":4 -> "rp1-cfe-csi2_ch0":0 [1]'
media-ctl -d $M -V '"ar0234 11-0010":0 [fmt:SGRBG10_1X10/960x600]'
media-ctl -d $M -V "\"csi2\":0 [fmt:$F]"
media-ctl -d $M -V "\"csi2\":4 [fmt:$F]"
v4l2-ctl -d /dev/v4l-subdev0 --set-subdev-fmt pad=1,width=2400,height=2,code=0x7002
v4l2-ctl -d /dev/video0 --set-fmt-video=width=960,height=600,pixelformat=pgAA
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=60 --stream-to=frames.raw
```

Get the subdevice and video node names from `media-ctl -d $M -p`. On the
tested setup, `csi2` is `/dev/v4l-subdev0`, the sensor is
`/dev/v4l-subdev2`, and `rp1-cfe-csi2_ch0` is `/dev/video0`.

Each frame is MIPI RAW10 packed: 4 pixels in 5 bytes, GRBG Bayer order.
A 960x600 frame is 720000 bytes.

Possible errors:

| Error | Cause | Fix |
|---|---|---|
| `VIDIOC_STREAMON returned -1 (Invalid argument)`, `Wrong width or height` in `dmesg` | The video node size is not the same as the `csi2` pad size. | Set the same size on all pads and on the video node. |
| `VIDIOC_STREAMON returned -1 (Broken pipe)` | The `csi2` pad 1 (embedded data) format is not the same as the sensor pad 1 format. | Set `csi2` pad 1 to `2400x2`, code `0x7002`. |
| `STREAMON` is successful, but no frames come in | The driver does not have the kernel 6.8 change from this branch. | Install the driver from this branch and reboot. |

## Flip

The driver has two controls on the sensor subdevice: `horizontal_flip`
and `vertical_flip`. The sensor changes its readout direction.

- Set the flip controls before you start the stream. The driver locks
  them during streaming. A change during streaming returns
  `Device or resource busy`.
- The Bayer order stays GRBG for all four flip combinations. The
  measured mean of each 2x2 Bayer position does not change.
- The overlay sets `rotation = <180>`. This value is information only
  (`camera_sensor_rotation`). It does not rotate the image.

```bash
v4l2-ctl -d /dev/v4l-subdev2 -c horizontal_flip=1,vertical_flip=1
```

## Timing

These values are measured on the tested setup.

- The line time is approximately 13.6 µs in all four modes.
- The frame period is `13.6 µs x (height + vertical_blanking)`.
- The maximum `exposure` value is `height + vertical_blanking - 6` lines.
- At 960x600 with `vertical_blanking=21`, the sensor gives 118 fps.
- At 960x600 with `vertical_blanking=1851`, the sensor gives 30.00 fps.

The `pixel_rate` and `horizontal_blanking` controls do not give the correct
line time. Do not calculate the frame rate from them.

## Remove the driver

1. Remove the DKMS module:

   ```bash
   dkms status | grep ar0234 | cut -d, -f1 | sort -u | xargs -rI{} sudo dkms remove {} --all
   ```

2. Remove the overlay:

   ```bash
   sudo rm /boot/firmware/overlays/ar0234.dtbo
   ```

3. Restore the boot configuration:

   ```bash
   sudo cp /boot/firmware/config.txt.bak-ar0234 /boot/firmware/config.txt
   ```

4. Reboot.
