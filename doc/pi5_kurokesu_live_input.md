# Pi 5 live 입력: Kurokesu AR0234와 PX4 IMU

이 문서는 `granite_vio --live`와 `granite_pi_cam_imu_vio`가 Raspberry Pi 5에서 카메라와 IMU를 읽는 방식을 설명합니다. 기본 입력은 Kurokesu AR0234 카메라와 PX4 flight controller(FC)의 IMU입니다. 이 입력은 xeye capture node와 같은 방식으로 이미지와 시각을 만듭니다. 그래서 xeye capture로 구한 보정을 live에 그대로 쓸 수 있습니다.

비교 기준은 multipleye `cross-pi5` 브랜치의 xeye capture node입니다(commit `b482d47f0`).

- 카메라: `camera_v4l2_gen2` node, `model: kurokesu_ar0234_pi` (`capturelib/capture/nodes/camera_v4l2_arducam_b0429_pi.cc`, `camera_v4l2_kurokesu_ar0234_pi.cc`)
- IMU: `imu_mavlink` node (`capturelib/capture/nodes/imu_mavlink.cc`)
- 설정: `xeye/configs/capture_kurokesu_ar0234_pi.yaml`

이전 입력(Arducam B0429 + I2C MPU, Pi 4 리그)은 남아 있습니다. 선택 방법은 8절에 있습니다. Pi 4 절차는 `doc/xeye_to_granite_guide.md`에 있습니다.

상태 표기는 `doc/xeye_to_granite_guide.md`와 같습니다.

- **[검증]**: 실행해서 확인한 내용입니다. 어디에서 확인했는지 같이 적습니다.
- **[코드 확인]**: 코드를 읽고, 컴파일러의 문법 검사(`-fsyntax-only`)를 통과한 내용입니다. 실행하지 않았습니다.
- **[미검증]**: 확인하지 않은 내용입니다.

Granite live 경로 전체는 Pi 5에서 아직 실행하지 않았습니다. 그래서 Granite 코드에 대한 내용은 모두 [코드 확인]입니다. [검증] 표기는 같은 처리를 하는 xeye node를 Pi 5(`uvify@10.20.10.60`)에서 실행한 결과입니다.

## 1. 구성

| 부분 | 장치 | 연결 | Granite 코드 |
|---|---|---|---|
| 카메라 | Kurokesu AR0234 (2 lane, RAW10, global shutter) | CAM/DISP 1, rp1-cfe | `src/device/pi_camera_imu.cpp` (`setupXeyeCamera`, `xeyeCameraLoop`, `processXeyeFrame`) |
| IMU | UVify V6X FMU (PX4) | USB, mavlink-router가 UDP로 전달 | `src/device/pi_camera_imu.cpp` (`setupMavlink`, `mavlinkLoop`) |
| 처리 함수 | - | - | `include/granite/device/pi_sensor_utils.h` (xeye node에서 옮긴 함수) |
| 설정 | - | - | `include/granite/device/pi_live_support.h` (명령줄 옵션, config 키) |

용어:

- **rp1-cfe**: Raspberry Pi 5의 CSI-2 수신기 driver입니다. media graph로 센서, `csi2`, video node를 연결합니다.
- **media graph**: V4L2 장치들의 연결 구조입니다. 각 장치를 entity라고 하고, entity의 입출력을 pad라고 합니다.
- **DMABUF**: 장치와 CPU가 같은 메모리를 file descriptor로 공유하는 Linux 방식입니다.
- **dma-heap**: DMABUF 메모리를 할당하는 Linux 장치입니다(`/dev/dma_heap/linux,cma`).
- **mavlink-router**: FC의 MAVLink 연결 하나를 여러 프로그램에 UDP로 나누어 주는 프로그램입니다.

## 2. 카메라 입력

### 2.1 시작할 때

`setupXeyeCamera()`가 다음 순서로 카메라를 준비합니다. [코드 확인]

1. rp1-cfe media graph를 찾습니다. 이 graph에는 이름이 `ar0234`로 시작하는 센서 entity와 `port`의 video node(기본 `/dev/video0`)가 있어야 합니다.
2. `csi2`에서 video node로 가는 link를 켭니다.
3. 센서 pad 0과 `csi2` pad 0을 `SGRBG10_1X10`(10 bit GRBG Bayer), 센서 모드 크기로 설정합니다.
4. `csi2`의 출력 pad를 `SGRBG16_1X16`으로 설정합니다. 이 설정에서 `csi2`는 RAW10을 16 bit word로 풉니다. 값은 word의 위 10 bit에 들어갑니다(GR16).
5. 센서의 다른 출력 pad(embedded data pad)의 형식을 그 pad와 연결된 `csi2` 입력 pad에 복사합니다. 두 형식이 다르면 STREAMON이 EPIPE로 실패합니다.
6. video node를 `V4L2_PIX_FMT_SGRBG16`으로 설정합니다. 한 행의 byte 수가 폭 × 2가 아니면 실패합니다.
7. 라인 시간을 정합니다. Kurokesu는 13.6 µs로 고정합니다. driver의 `pixel_rate`와 `horizontal_blanking` control로는 맞는 값이 나오지 않습니다.
8. `vertical_blanking`으로 frame rate를 맞춥니다. vblank [줄] = round(1 s / fps / 라인 시간) − 높이 [줄]입니다.
9. 노출, gain, 좌우 반전(`h_flip`), 상하 반전(`v_flip`)을 설정합니다. driver는 streaming 중에 반전 control을 잠그므로 STREAMON 전에 설정합니다.
10. 버퍼를 할당합니다(2.2절).
11. STREAMON 후 2초 안에 프레임이 오지 않으면 실패합니다.

라인 시간 13.6 µs는 Kurokesu driver 저장소의 `docs/ubuntu-24.04.md`에 적힌 측정값입니다. 960x600, vblank 1851줄에서 30.00 fps가 나왔습니다. [검증, Kurokesu driver 문서]

### 2.2 버퍼: DMABUF

rp1-cfe의 V4L2 MMAP 버퍼는 CPU cache를 거치지 않습니다. 그래서 Granite는 `/dev/dma_heap/linux,cma`에서 버퍼를 할당하고 `V4L2_MEMORY_DMABUF`로 넘깁니다. CPU는 이 버퍼를 cache로 읽습니다. 할당이 실패하면 경고를 내고 MMAP 버퍼를 씁니다. [코드 확인]

DMABUF 버퍼는 cache에 이전 데이터가 남을 수 있습니다. 그래서 프레임마다 읽기 전에 `DMA_BUF_IOCTL_SYNC`(START)를 호출하고, 읽은 뒤에 END를 호출합니다.

Pi 5(kernel 6.8.0-1065-raspi)에서 측정한 1.1 MB(960x600 GR16) 복사 시간입니다. [검증, 시험 프로그램 `~/v4l2_cache_test`]

| 버퍼 | memcpy | sync | 합계 |
|---|---|---|---|
| V4L2 MMAP | 0.95 ms (1.2 GB/s) | 없음 | 0.95 ms |
| DMABUF, `linux,cma` | 0.22 ms (5.2 GB/s) | 0.06 ms | 0.28 ms |
| 일반 heap 메모리, cache가 빈 상태 (기준값) | 0.27 ms (4.3 GB/s) | - | - |

이 kernel의 rp1-cfe는 MMAP cache 옵션(`V4L2_BUF_CAP_SUPPORTS_MMAP_CACHE_HINTS`)을 지원하지 않습니다. [검증]

### 2.3 프레임마다

`processXeyeFrame()`이 버퍼 하나를 gray8 이미지 하나로 만듭니다. 순서는 xeye node의 `FillFrame()`과 같습니다. [코드 확인]

1. 잘못된 프레임을 버립니다. 조건은 `V4L2_BUF_FLAG_ERROR`, `bytesused` < 폭 × 높이 × 2, 버퍼 시각 ≤ 0입니다.
2. DMABUF이면 sync START를 호출합니다.
3. crop 영역만 읽습니다. 시작 위치는 (crop x, crop y)이고, 행마다 crop 폭만큼 읽습니다. 행 단위로 먼저 memcpy합니다.
4. 10 bit를 8 bit로 줄입니다.
   - WB와 gamma가 모두 꺼져 있으면 word의 위 byte를 씁니다. 8 bit 값 = 10 bit 값 / 4입니다.
   - 아니면 LUT(look-up table)를 씁니다. LUT는 녹색 pixel, 짝수 행의 비녹색 pixel, 홀수 행의 비녹색 pixel에 각각 하나씩 있습니다.
5. DMABUF이면 sync END를 호출합니다.
6. 8 bit Bayer를 gray8로 바꿉니다(`bayer8ToGray8`).

LUT 식은 다음과 같습니다. v는 10 bit pixel 값 [DN, 0-1023]이고 b는 `black_level` [DN]입니다. g는 그 pixel 종류의 WB gain입니다(녹색은 1). DN(digital number)은 센서 출력의 정수 값입니다.

- gamma 꺼짐: 출력 [DN, 0-255] = clamp((b + (v − b) × g) / 4, 0, 255)
- gamma 켜짐: t = clamp((v − b) × g / (1023 − b), 0, 1), 출력 [DN] = round(255 × t^γ). γ는 `gamma_value`입니다.

WB gain은 비녹색 pixel의 합을 녹색 pixel의 합에 맞춥니다. 통계는 Bayer 행 쌍 4개 중 1개에서 모읍니다. 짝수 행 비녹색 gain = 0.5 × Σ(녹색 − b) / Σ(짝수 행 비녹색 − b)입니다. 홀수 행도 같은 식입니다. 녹색 pixel 수는 각 비녹색 pixel 수의 2배이므로 0.5를 곱합니다.

- `fused_wb: true`: 이전 프레임의 통계로 이번 프레임의 gain을 정합니다. 통계는 LUT를 적용하는 같은 읽기에서 모읍니다. 첫 프레임은 같은 프레임의 통계를 씁니다.
- `fused_wb: false`: 프레임마다 같은 프레임을 한 번 더 읽어 통계를 모읍니다.

gray8 변환은 xcv `DemosaicAndToGray8`의 결과와 byte 단위로 같습니다. 이 함수는 bilinear demosaic으로 R, G, B를 만들고 (R + G + B) / 3을 출력합니다. `bayer8ToGray8`은 같은 정수 나눗셈을 RGB 이미지 없이 한 번에 계산합니다. 경계 pixel은 원래 값을 유지합니다.

- 녹색 pixel: (C + (W + E)/2 + (N + S)/2) / 3
- 비녹색 pixel: (C + (N + S + E + W)/4 + (대각선 4개의 합)/4) / 3

C는 그 pixel 값이고 N, S, E, W는 위, 아래, 오른쪽, 왼쪽 pixel 값입니다. 모든 나눗셈은 정수 나눗셈입니다. [검증, host에서 무작위 입력 1188만 pixel, Bayer 순서 4개, 불일치 0]

xeye node에서 측정한 처리 시간입니다(960x600 gray8, 30 fps, 카메라 스레드). [검증, xeye node, .60]

| 조건 | 프레임당 CPU 시간 |
|---|---|
| WB + `fused_wb` + AE/AG, DMABUF | 3.13 ms |
| WB, gamma, AE, AG 끔 | 2.70 ms |

Granite의 처리 시간은 측정하지 않았습니다. [미검증]

### 2.4 자동 노출과 gain (AE/AG)

`runXeyeAutoExposure()`가 출력 gray8 이미지를 측광하고 노출과 gain을 바꿉니다. 식은 xeye node(`multi_camera_v4l2_arducam_b0429`의 `UpdateExposureGain`)와 같습니다. [코드 확인]

1. 측광 영역을 정합니다. `auto_exposure_roi`가 있으면 그 영역을 씁니다. 없으면 `auto_exposure_metering`을 씁니다: `average`(전체), `spot`(가운데 면적 3 %), `partial`(가운데 면적 11 %). `auto_exposure_circle`이 있으면 원 밖의 sample을 뺍니다.
2. 8번째 행과 열마다 2x2 칸 하나를 sample로 씁니다. m [%] = sample 평균 / 255 × 100입니다. gamma가 켜져 있으면 gamma의 역함수로 선형 값으로 되돌린 뒤 평균합니다.
3. 오차를 계산합니다. 목표 T [%] = min(99, `target_gray_value` [%] × 2^`auto_exposure_ev_compensation`)입니다. e = ln(T / max(m, 0.1))입니다. |e| < 0.1이면 바꾸지 않습니다.
4. 한 번의 변화량을 정합니다. s = clamp((1 − `auto_exposure_damping`) × e, −0.693, 0.693)입니다. 0.693은 ln 2이므로 한 번에 밝기가 최대 2배 바뀝니다.
5. 목표 밝기 B = 노출 [ms] × gain [배] × e^s입니다.
6. `auto_exp_prior: exposure`이면 노출을 먼저 바꿉니다. 새 노출 [ms] = clamp(B / gain 최솟값, `auto_exposure_range`)이고, 새 gain [배] = clamp(B / 새 노출, gain 범위)입니다. `gain`이면 순서가 반대입니다.
7. 값을 바꾸면 다음 2 프레임은 AE를 쉽니다. pivariety driver의 control은 프레임 N + 2부터 적용됩니다. Kurokesu driver의 지연은 측정하지 않았습니다. [미검증]

노출 상한은 1000 / fps − 0.1 [ms]입니다. 이보다 긴 노출은 frame rate를 낮춥니다. `auto_exposure_range`의 최댓값이 이 값보다 크면 Granite가 경고를 내고 줄입니다.

gain 모델은 센서마다 다릅니다.

| model | analogue_gain 값 v | 선형 gain [배] | 상태 |
|---|---|---|---|
| `kurokesu_ar0234_pi` | 13-64 (R0x3060 register 값) | 2^⌊v / 16⌋ × 32 / (32 − v mod 16) | [미검증] AR0234 register 문서의 식 |
| `b0429_pi` | 100-1122 | 1 + (v − 100) / 408.5 | [검증] openvio에서 Pi 5 측정, 오차 3 % 이하 |

`auto_gain_range` [dB]는 선형 gain의 dB입니다(20 × log10(gain)). 비우면 driver control 범위를 씁니다. Kurokesu는 4.53-24.08 dB입니다. [검증, xeye node 로그]

### 2.5 binning과 host 축소 (`downscale`)

`downscale` N(또는 `--resize-half`, N = 2)은 출력 이미지를 crop의 1/N 크기로 만듭니다. [코드 확인]

- **Kurokesu, N = 2, 센서 모드 1920x1200 (`sensor_binning: true`, 기본값):** Granite가 센서 모드를 960x600 2x2 binning 모드로 바꿉니다. crop의 각 값을 2로 나눕니다. 예: `[288, 0, 1344, 1200]` → `[144, 0, 672, 600]`. host 평균은 하지 않습니다. 출력은 binning 모드의 crop입니다.
- **crop 값이 4의 배수가 아닐 때:** 2로 나눈 값이 홀수가 되면 Bayer 위상이 바뀝니다. 이때 Granite는 경고를 내고 binning을 쓰지 않습니다. host가 축소합니다.
- **`sensor_binning: false`, N = 3 같은 다른 값, 다른 센서 모드:** host가 축소합니다. 8 bit Bayer에 `grbgToGray`를 적용합니다(짝수 N은 N x N 평균).

`auto_exposure_roi`와 `auto_exposure_circle`은 설정한 센서 모드의 crop 좌표로 해석합니다. binning에서는 2로 나누고, host 축소에서는 N으로 나눕니다. AE는 출력 이미지를 측광하기 때문입니다.

binning 출력 pixel i는 전체 해상도 pixel 2i와 2i + 1을 덮는다고 가정합니다. 그래서 보정 식은 host 2x2 축소와 같습니다(7.2절). binning과 host 2x2 평균의 pixel 기하가 같다는 가정은 측정하지 않았습니다. [미검증]

## 3. IMU 입력

`mavlinkLoop()`이 PX4의 `HIGHRES_IMU` 메시지를 읽습니다. 동작은 xeye `imu_mavlink` node와 같습니다. [코드 확인]

1. mavlink-router의 UDP endpoint 주소(기본 `127.0.0.1:14601`)에 bind합니다. `imu.port`를 주면 UDP 대신 serial port를 엽니다(8N1, flow control 없음).
2. `HEARTBEAT`에서 autopilot의 system id와 component id를 찾습니다.
3. `MAV_CMD_SET_MESSAGE_INTERVAL`로 `HIGHRES_IMU`를 `imu.freq_hz` [Hz](기본 400)로 요청합니다. ACK가 올 때까지 2초마다 다시 보냅니다. `imu.freq_hz: 0`이면 요청하지 않고 FC의 설정 rate를 씁니다.
4. FC가 요청을 거부하면 경고를 내고 FC의 rate를 씁니다.
5. 3초 동안 `HIGHRES_IMU`가 없으면 rate를 다시 요청합니다. FC 시각이 1초보다 많이 뒤로 가면 FC가 재부팅했다고 보고 시각 매핑을 다시 시작합니다.
6. 각 sample을 `ImuData`로 넘깁니다. gyro [rad/s]와 가속도 [m/s²]는 FC body frame(FRD, PX4 `SENS_BOARD_ROT` 적용 후)입니다. PX4는 EKF2의 bias 추정값을 이미 뺀 값을 보냅니다.

xeye node 시험 결과입니다. [검증, xeye node, .60, 15초]

- `HIGHRES_IMU` 수신 193.6 Hz
- FC가 400 Hz 요청을 거부함(`MAV_RESULT 2`). 그래서 FC의 기본 rate를 씀.
- 15초 동안 IMU sample 2735개

### 3.1 시각 매핑 (`FcClockMapper`)

FC sample 시각을 host 시각으로 바꿉니다. [코드 확인]

- t_fc [ns]: `HIGHRES_IMU.time_usec` × 1000. FC가 sample을 만든 시각입니다.
- t_rx [ns]: host가 UDP datagram을 받은 시각(CLOCK_MONOTONIC)입니다.
- d [ns] = t_rx − t_fc: 전송 지연과 두 시계의 차이를 합한 값입니다.

1초 bucket 5개(5초 창)에서 d의 최솟값 d_min을 구합니다. 가장 빨리 도착한 sample의 d가 실제 시계 차이에 가장 가깝기 때문입니다. 적용하는 offset o는 sample마다 d_min 쪽으로 움직입니다. 한 번에 움직이는 양은 FC sample 간격의 1 % 이하입니다. host 시각 t_host [ns] = t_fc + o입니다. 처음 1초의 sample은 창을 채우기만 하고 넘기지 않습니다.

## 4. timestamp 기준

| 데이터 | 시각 | 시계 |
|---|---|---|
| 이미지 | V4L2 버퍼 시각 + `camera_time_offset_ns` | CLOCK_MONOTONIC |
| IMU | FC sample 시각을 host 수신 시각에 맞춘 값 (3.1절) | CLOCK_MONOTONIC |

이미지 시각은 노출 중간으로 보정하지 않습니다. xeye node와 같은 기준이므로 xeye capture로 구한 카메라-IMU time offset을 live에 그대로 씁니다. `camera_time_offset_ns`는 보정 파일의 `cam_time_offset_ns`입니다(t_imu = t_cam + offset).

xeye node는 이 두 시각에 CLOCK_REALTIME 기준점을 더합니다. Granite는 CLOCK_MONOTONIC을 그대로 씁니다. 두 시각에 같은 상수가 더해지므로 카메라와 IMU의 상대 시각은 같습니다. [코드 확인]

rp1-cfe가 버퍼에 찍는 시각이 프레임의 어느 순간(시작 또는 끝)인지는 kernel 코드에서 확인하지 않았습니다. [미검증] AE가 노출을 바꾸면 노출 중간과 버퍼 시각의 차이도 바뀝니다. 이 차이는 보정하지 않습니다.

## 5. xeye capture 형식

### 5.1 capture.yaml

같은 장치로 xeye capture를 기록하는 설정입니다. 값은 Granite live의 기본값과 같습니다(6절).

```yaml
sensor_nodes: [cam0, imu_fc]

cam0:
  type: camera_v4l2_gen2
  port: /dev/video0
  model: kurokesu_ar0234_pi
  property:
    pixel_format: gray8
    width: 1920
    height: 1200
    crop: [288, 0, 1344, 1200]
    frame_rate: 30
    exposure_time: 8.0
    gain: 14
    h_flip: true
    v_flip: true
    auto_white_balance: true
    fused_wb: true
    black_level: 42
    enable_gamma: false
    auto_exposure: true
    auto_gain: true
    auto_exposure_range: [0.1, 10.0]
    auto_exp_prior: exposure
    target_gray_value: 20.0

imu_fc:
  type: imu_mavlink
  udp: 127.0.0.1:14601
  freq_hz: 400
```

- `cam0`은 단일 카메라 node입니다. `camera_v4l2_gen2`가 `model`로 처리 module을 고릅니다.
- `imu_fc`는 `udp` 또는 `port`와 `baudrate` 중 하나를 씁니다.
- 보정 뒤에는 `calib.cam0`(카메라 모델, intrinsic, `cam2rig_pose`)과 `calib.imu_fc`(`imu2rig_pose`, `time_offset`, bias, noise)가 추가됩니다.

### 5.2 기록 파일

capture 폴더의 파일입니다. [검증, xeye node, .60]

| 파일 | 내용 |
|---|---|
| `capture.yaml` | 기록할 때의 설정 사본, `play_record: true` |
| `capture.manifest.yaml` | 기록 정보 |
| `cam0.csv` | 프레임마다 `Timestamp (ns), fidx`. 첫 줄은 `# camera_frame 1 <폭> <높이> GRAY8` |
| `cam0.xidx`, `cam0000.bin` | 프레임 index와 이미지 데이터. gray8 프레임 하나는 폭 × 높이 byte |
| `imu_fc.csv`, `imu_fc.xidx` | IMU sample |

960x600 gray8 30 fps에서 프레임 간격은 33.334 ms, 표준편차 0.002 ms였습니다. xeye node가 V4L2 버퍼 시각을 쓰기 때문입니다. [검증, xeye node]

## 6. 설정 키

Granite config 파일에서는 `camera.<키>`와 `imu.<키>`로 씁니다. 명령줄에서는 `--<키>`와 `--imu_<키>`로 씁니다. 명령줄 옵션이 config 키보다 우선합니다. 키 이름은 xeye 키 이름과 같습니다. 값의 범위와 오류 메시지도 xeye와 같습니다. [코드 확인]

### 6.1 카메라

| 키 | 단위 | 기본값 | 범위와 검사 |
|---|---|---|---|
| `model` | - | `kurokesu_ar0234_pi` | `kurokesu_ar0234_pi`, `b0429_pi` (`arducam_b0429_pi`도 받음), `b0429` (8절) |
| `port` | - | `/dev/video0` | rp1-cfe의 video node. 비우면 센서가 있는 graph의 `rp1-cfe-csi2_ch0` |
| `width`, `height` | px | 0 → Kurokesu 1920x1200, B0429 1280x720 | Kurokesu: 1920x1200, 1920x1080, 1280x720, 960x600. `b0429_pi`: 앞의 3개 |
| `fps` (`frame_rate`) | Hz | 30 | > 0 |
| `pixel_format` | - | `gray8` | `gray8`만 받음 (VIO는 gray 이미지를 씀) |
| `exposure_time` | ms | 8.0 | 0.002-65.535. 범위 밖이면 경고 후 1.0. AE의 시작값 |
| `gain` | driver 값 | −1 → Kurokesu 14, `b0429_pi` 100 | Kurokesu 13-64, `b0429_pi` 100-1200. 범위 밖이면 경고 후 100 |
| `num_v4l_buffer` | 개 | 4 | 2 이상 |
| `h_flip`, `v_flip` | - | true, true | 센서가 반전함. Bayer 순서는 GRBG로 유지됨 |
| `crop` | px | Kurokesu 기본 모드: `[288, 0, 1344, 1200]`. 그 밖: 전체 프레임 | `[x, y, w, h]`, 짝수로 내림, 센서 모드 안 |
| `auto_white_balance` | - | true | |
| `fused_wb` | - | true | `auto_white_balance`가 꺼지면 무시 |
| `black_level` | DN (10 bit) | 42 | 0-255 |
| `enable_gamma` | - | false | |
| `gamma_value` | - | 0.45 | gamma가 켜지면 0.25-1 |
| `auto_exposure`, `auto_gain` | - | true, true | |
| `auto_exposure_range` | ms | `[0.1, 10.0]` | 0 < 최솟값 ≤ 최댓값. 최솟값이 노출 상한보다 크면 오류 |
| `auto_gain_range` | dB | 비움 → driver 범위 | 최솟값 ≤ 최댓값, driver 범위 안 |
| `auto_exp_prior` | - | `exposure` | `exposure`, `gain` |
| `target_gray_value` | % | 20 | 1-99 |
| `auto_exposure_ev_compensation` | EV | 0 | −3 이상 3 이하 |
| `auto_exposure_damping` | - | 0.5 | 0 이상 1 미만 |
| `auto_exposure_metering` | - | `average` | `average`, `spot`, `partial` |
| `auto_exposure_roi` | px | 비움 | `[x, y, w, h]`, crop 좌표 |
| `auto_exposure_circle` | px | 비움 | `[cx, cy, r]`, r > 0, crop 좌표 |
| `downscale` | - | 1 | Granite 옵션. 1 이상 (2.5절) |
| `sensor_binning` | - | true | Granite 옵션. Kurokesu, `downscale` 2, 1920x1200에서 binning 모드를 씀 (2.5절) |

`gain`의 범위 13-64와 100-1200은 xeye의 센서 profile 값입니다. driver는 이 값을 자기 control 범위로 다시 자릅니다.

### 6.2 IMU

| 키 | 단위 | 기본값 | 범위와 검사 |
|---|---|---|---|
| `type` | - | `imu_mavlink` | `imu_mavlink`, `mpu_i2c` (8절) |
| `udp` | - | `udp`와 `port`가 모두 비면 `127.0.0.1:14601` | `<주소>:<port>`, port 1-65535 |
| `port` | - | 비움 | serial device. `udp`와 같이 쓰면 오류 |
| `baudrate` | baud | 921600 | 57600, 115200, 230400, 460800, 500000, 921600, 1000000, 1500000 |
| `freq_hz` | Hz | 400 | 0 이상. 0이면 FC의 rate를 유지 |

## 7. capture_to_granite.py 변환

`eval/scripts/capture_to_granite.py`는 xeye capture의 `capture.yaml`(보정 포함)을 Granite 보정 JSON과 VIO config JSON으로 바꿉니다. 이 절은 새 node 형식에 대한 내용입니다. 나머지 내용(카메라 모델 fit, IMU noise, VIO 설정)은 `doc/xeye_to_granite_guide.md` 2절에 있습니다.

### 7.1 키 매핑

| capture.yaml | Granite config |
|---|---|
| `camera_v4l2_gen2` node의 `model` (`kurokesu_ar0234_pi`, `b0429_pi`) | `camera.model` |
| 그 node의 `port` | `camera.port` |
| 그 node의 `property.<키>` (6.1절의 xeye 키) | `camera.<키>` |
| 그 node의 `property.frame_rate` | `camera.fps` (`--fps`를 주면 그 값) |
| `--downscale` | `camera.downscale` |
| `imu_mavlink` node의 `udp`, `port`, `baudrate`, `freq_hz` | `imu.type: imu_mavlink`, `imu.udp`, `imu.port`, `imu.baudrate`, `imu.freq_hz` |
| 다른 카메라 node (multi-camera node 등) | `camera.model: b0429`와 이전 키 |
| 다른 IMU node (`imu_mpu6050` 등) | `imu.type: mpu_i2c`, `camera.i2c_device`, `camera.imu_rate` |

capture의 `pixel_format`이 `gray8`이 아니면 스크립트가 경고를 내고 `gray8`을 씁니다. `imu_mavlink`이면 보정의 `imu_update_rate`는 200 Hz입니다. PX4는 새 sample을 `IMU_INTEG_RATE`(기본 200 Hz)로 만듭니다.

### 7.2 intrinsic 변환식

보정 파일의 크기와 intrinsic을 live 출력 이미지에 맞춥니다. 기호는 다음과 같습니다.

- f [px]: 보정의 focal length
- c [px]: 보정의 principal point (x 또는 y 성분)
- x₀ [px]: crop 원점의 같은 성분(센서 모드 좌표)
- N: `downscale` (binning을 쓰면 2)
- W × H [px]: crop 크기

변환 순서는 다음과 같습니다.

1. **crop:** 보정 크기가 센서 모드 크기이고 crop이 있으면 c_crop [px] = c − x₀, 크기 = W × H입니다. 보정 크기가 이미 crop 크기이면 그대로 씁니다. 두 크기 모두 아니면 오류입니다.
2. **downscale:** f' [px] = f / N, c' [px] = (c_crop + 0.5) / N − 0.5, 크기 = (W / N) × (H / N)입니다. 출력 pixel u가 입력 pixel N·u부터 N·u + N − 1까지를 덮는다는 가정의 식입니다. pixel 중심 좌표를 쓰므로 0.5를 더하고 뺍니다.
3. **binning:** Kurokesu, N = 2, 1920x1200, crop 값이 모두 4의 배수이면 live가 binning 모드를 씁니다. 출력 크기와 식은 2번과 같습니다. 스크립트는 live가 binning을 쓰는지 출력합니다.

예: 1920x1200 보정, crop `[288, 0, 1344, 1200]`, `--downscale 2`일 때 c_x = 932.1112 px이면 c_crop = 644.1112 px, c' = (644.1112 + 0.5) / 2 − 0.5 = 321.8056 px입니다. 크기는 672x600입니다. live는 binning 모드의 crop `[144, 0, 672, 600]`을 씁니다. [검증, 합성 capture dry-run]

live 앱은 시작할 때 같은 검사를 합니다(`fitCalibration`). 보정 크기가 crop 크기이면 2번 식으로 줄이고, 출력 크기이면 그대로 씁니다. [코드 확인]

### 7.3 time_offset

| node | `--target live` | `--target capture` |
|---|---|---|
| `camera_v4l2_gen2` (`kurokesu_ar0234_pi`, `b0429_pi`) | capture의 `time_offset` | capture의 `time_offset` |
| 다른 node | 0 | capture의 `time_offset` |

새 node에서는 live 이미지 시각과 IMU 시각이 xeye capture와 같은 기준입니다(4절). 그래서 live에도 capture의 `time_offset`을 씁니다. 다른 node의 live 경로(`b0429`)는 노출 중간을 이미지 시각으로 씁니다. 이 경우 capture의 `time_offset`은 맞지 않습니다.

### 7.4 실행 예

```bash
python3 eval/scripts/capture_to_granite.py <capture 폴더> --downscale 2 \
    --calib-out calib.json --config-out config.json
granite_vio --live --use-imu 1 --cam-calib calib.json --config-path config.json
```

`auto_exposure_circle`이 없는 capture이고 보정이 uwfisheye이면, 스크립트가 어안 원 안쪽을 측광하는 `--auto_exposure_circle` 값을 제안합니다. 이 값은 출력 크기 좌표입니다.

## 8. 이전 경로 선택 (B0429 + I2C MPU)

| 키 | 값 | 내용 |
|---|---|---|
| `camera.model` / `--model` | `b0429` | Arducam B0429, rp1-cfe 또는 Pi 4 unicam. packed RAW10, `grbgToGray`, 노출 중간 이미지 시각, Granite의 AE (`exposure_ms`, `gain_db`, `max_exposure_ms`, `max_gain`, `brightness_target`, `metering_circle`) |
| `imu.type` / `--imu_type` | `mpu_i2c` | MPU-6500/9250, I2C adapter (`--i2c`, `--imu-address`, `--imu-rate`) |

이전 경로의 수동 gain 옵션은 `--gain-db`입니다. `--gain`은 이제 xeye 모델의 analogue_gain 값입니다. config 키 `camera.gain_db`는 바뀌지 않았습니다.

`scripts/pi4_live_mono_vio.sh`와 `data/uvify_config_mono_vio.json`은 이전 경로를 선택합니다. 다른 B0429 설정 파일에는 `camera.model`과 `imu.type`을 추가하거나 명령줄 옵션을 주십시오.

## 9. 검증 상태 요약

| 내용 | 상태 |
|---|---|
| Granite live 경로 전체 (Pi 5 실행) | [미검증] |
| Granite 코드의 컴파일 (`-Wall -Wextra -Werror -fsyntax-only`, x86) | [코드 확인] |
| Pi 교차 컴파일 | [미검증] |
| DMABUF 복사 시간, MMAP이 cache를 거치지 않음 | [검증] 시험 프로그램, .60 |
| `bayer8ToGray8` = xcv `DemosaicAndToGray8` | [검증] host 비교 |
| xeye node의 gray8 출력, 프레임 간격, 처리 시간 | [검증] xeye node, .60 |
| `imu_mavlink` 수신 rate, FC의 rate 요청 거부 | [검증] xeye node, .60 |
| Kurokesu 라인 시간 13.6 µs | [검증] Kurokesu driver 문서의 측정값 |
| Kurokesu gain 모델 | [미검증] |
| Kurokesu의 `black_level` 42 | [미검증] Helios B0429에서 측정한 값 |
| Kurokesu control 지연 (AE 대기 2 프레임) | [미검증] |
| binning과 host 2x2 평균의 pixel 기하가 같다는 가정 | [미검증] 측정하지 않음 |
| rp1-cfe 버퍼 시각의 의미 | [미검증] |
| `capture_to_granite.py`의 키 매핑, crop·downscale 식, 이전 형식 호환 | [검증] 합성 capture와 기존 capture의 dry-run |
