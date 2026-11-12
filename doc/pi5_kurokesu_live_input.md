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

`mavlinkLoop()`이 FC의 `HIGHRES_IMU` 메시지를 읽습니다. 동작은 xeye `imu_mavlink` node와 같습니다. [코드 확인]

1. mavlink-router의 UDP endpoint 주소(기본 `127.0.0.1:14601`)에 bind합니다. `imu.port`를 주면 UDP 대신 serial port를 엽니다(8N1, flow control 없음).
2. `HEARTBEAT`에서 autopilot의 system id와 component id를 찾습니다.
3. `MAV_CMD_SET_MESSAGE_INTERVAL`로 `HIGHRES_IMU`를 `imu.freq_hz` [Hz](기본 400)로 요청합니다. ACK가 올 때까지 2초마다 다시 보냅니다. `imu.freq_hz: 0`이면 요청하지 않고 FC의 설정 rate를 씁니다.
4. FC가 요청을 거부하면 경고를 내고 FC의 rate를 씁니다.
5. 3초 동안 `HIGHRES_IMU`가 없으면 rate를 다시 요청합니다. FC 시각이 1초보다 많이 뒤로 가면 FC가 재부팅했다고 보고 시각 매핑을 다시 시작합니다.
6. 각 sample을 `ImuData`로 넘깁니다. gyro [rad/s]와 가속도 [m/s²]는 FC body frame(FRD)입니다. ArduPilot은 IMU 값을 `AHRS_ORIENTATION`으로 회전해서 보냅니다.

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
| `send_odometry` | - | false | Granite 옵션. VIO 상태를 FC에 `ODOMETRY`로 보냄 (9절). `imu_mavlink`이 아니면 오류 |

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

## 9. ODOMETRY 출력

`imu.send_odometry: true`(명령줄 `--imu_send_odometry 1`)이면 live 앱이 VIO 상태마다 MAVLink `ODOMETRY`(#331) 메시지 하나를 FC에 보냅니다. 기본값은 false입니다. 메시지 내용은 ArduPilot 4.7.2(UVify V6X)의 `GCS_MAVLINK::handle_odometry`에 맞춥니다. [코드 확인]

### 9.1 송신 경로

| 부분 | 코드 |
|---|---|
| 상태 하나를 보냄 | `src/vio_app.h` (`VioDataset::sendOdometry`). state thread가 `writeState()` 다음에 호출합니다 |
| 메시지를 만들고 씀 | `src/device/pi_camera_imu.cpp` (`PiCameraImuDevice::sendOdometry`, `writeMavlink`) |

- `granite_vio --live`와 `granite_pi_cam_imu_vio`(VIO)가 같은 코드를 씁니다.
- IMU 입력과 같은 MAVLink 연결을 씁니다. 새 socket을 열지 않습니다.
  - UDP: router가 보낸 마지막 datagram의 송신 주소로 보냅니다. router가 이 메시지를 FC의 UART로 전달합니다. router의 datagram을 받기 전에는 보내지 않습니다.
  - serial: IMU를 읽는 같은 port에 씁니다.
- system id와 component id는 `HIGHRES_IMU` rate 요청과 같습니다: 1, `MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`(197).
- IMU thread와 state thread는 mutex 하나(`mavlink_mutex`)를 같이 씁니다. 이 mutex는 fd 쓰기와 close, 송신 주소, `MAVLINK_COMM_0`의 sequence 번호를 보호합니다.
- 송신이 실패하면 경고를 냅니다. 실패가 이어지는 동안에는 경고를 한 번만 냅니다. VIO는 멈추지 않습니다.
- 종료할 때 `LIVE` 줄에 `odometry_sent`(보낸 메시지 수)와 `odometry_errors`(실패 수)를 씁니다.

보내지 않는 경우:

| 조건 | 동작 |
|---|---|
| `imu.type`이 `imu_mavlink`이 아님 | 시작할 때 오류 |
| `--use-imu 0` (VO) | 경고 후 보내지 않음. VO의 world는 중력 정렬이 아니고 scale이 없습니다 |
| 초기화 전 | 보내지 않음. estimator는 metric 초기화 뒤에만 상태를 냅니다 |
| 상태 시각이 이전에 보낸 시각보다 크지 않음 | 경고 후 그 상태를 보내지 않음 |
| 위치, 자세, 속도에 NaN 또는 Inf가 있음 | 보내지 않음 |
| `stop()` 뒤 | 보내지 않음. 종료할 때 estimator가 남은 상태를 내도 보내지 않습니다 |

### 9.2 필드

| 필드 | 값 |
|---|---|
| `time_usec` | 상태 시각 [ns] / 1000. 이미지 시각과 같은 CLOCK_MONOTONIC 기준입니다(4절) |
| `frame_id` | `MAV_FRAME_LOCAL_FRD` (20) |
| `child_frame_id` | `MAV_FRAME_BODY_FRD` (12) |
| `x`, `y`, `z` [m] | LOCAL_FRD의 위치 (9.3절) |
| `q` | LOCAL_FRD에 대한 body FRD의 자세. 순서는 w, x, y, z입니다 |
| `vx`, `vy`, `vz` [m/s] | body FRD의 속도 |
| `rollspeed`, `pitchspeed`, `yawspeed` | NaN. ArduPilot은 이 값을 쓰지 않습니다 |
| `pose_covariance`, `velocity_covariance` | 모든 원소가 NaN |
| `reset_counter` | 9.4절 |
| `estimator_type` | `MAV_ESTIMATOR_TYPE_VIO` (3) |
| `quality` | 0(unknown). health 검사가 실패하면 ODOMETRY를 보내지 않습니다. 아래의 health 검사를 보십시오 |

ArduPilot은 다음과 같이 처리합니다.

- `frame_id`가 LOCAL_FRD가 아니거나 `child_frame_id`가 BODY_FRD가 아니면 메시지를 버립니다.
- `pose_covariance[0]`이 NaN이면 위치 오차와 각도 오차로 `VISO_POS_M_NSE`와 `VISO_YAW_M_NSE`를 씁니다.
- 속도에 `q`를 곱해 body FRD에서 LOCAL_FRD로 회전합니다.
- `quality`가 `VISO_QUAL_MIN`(기본 0)보다 작으면 쓰지 않습니다. 그래서 기본 설정에서 −1인 상태는 쓰지 않습니다.

**health 검사:** `sendOdometry()`는 `VioStateData::healthy`가 false이면 ODOMETRY를 보내지 않습니다. 두 초기화 방법(`vo_alignment`, `inertial`)은 같은 규칙(`HealthMonitor`, `src/vi_estimator/mono_vio.cpp`)으로 이 값을 정합니다.

`quality` −1을 보내지 않는 이유: ArduPilot은 −1인 메시지를 EKF에 쓰지 않지만, 수신한 것으로 기록합니다. 그래서 VisualOdom이 healthy로 남고, arming 검사가 arming을 막지 않습니다. 송신이 멈추면 0.3 s 뒤 FC가 "VisualOdom: not healthy"로 arming을 거부합니다. FC 담당이 추가할 예정인 VISO 손실 처리기도 송신 중단을 감지합니다(armed에서 `VISO_LOSS_MS`, 기본 1 s). [FC 담당 답변, 2026-11-12]

| 규칙 | 조건 | 설정 (기본값) |
|---|---|---|
| 회전 보정 | 초기화 뒤 settle + window가 지나면 검사합니다. 최근 window 동안 직전 출력 상태의 rms 보정이 회전으로 기준보다 크고, 이동으로 기준보다 작으면 실패입니다(나쁜 초기화 뒤 거의 정지) | `vio_mono_health_settle_s` (3 s), `vio_mono_health_window_s` (2 s), `vio_mono_health_rot_deg` (0.07°), `vio_mono_health_pos_m` (5 mm) |
| bias 크기 | 초기화 뒤 settle이 지나면 검사합니다. bias 추정값이 기준보다 크면 실패입니다(발산) | `vio_mono_health_max_accel_bias` (2.0 m/s²), `vio_mono_health_max_gyro_bias` (0.1 rad/s) |

- 한 번 실패하면 그 estimator가 끝날 때까지 송신하지 않습니다. live 설정의 `inertial` 초기화는 다시 초기화하지 않습니다(9.4절). 그래서 그 실행에서는 송신이 다시 시작되지 않습니다. 실패할 때와 healthy로 돌아올 때 stderr에 `ODOMETRY:` 줄을 씁니다.
- 검사는 상태 시각으로 합니다. 초기화 뒤 처음 3 s는 실패를 알리지 않습니다.
- 확인한 결과 [데스크톱, 각 1회]:
  - 오검출: Unreal Cam0, Cam3 전체(각 6008 상태)와 EuRoC MH_01-05에서 0회입니다.
  - 발산 검출: 이동 시작으로 발산한 Unreal Cam3 실행에서 초기화 7.0 s 뒤에 실패로 바꿨습니다. 그때 위치는 출발점에서 117 m였고, 실행이 끝날 때 6.9 km였습니다.
- 규칙은 Unreal 데이터에서 정했습니다. 실제 기체의 진동과 영상에서 오검출하는지는 확인하지 않았습니다 [미검증]. 오검출하면 비행 중에 FC가 VIO를 버립니다.

**최종 후보와 live 설정은 bias 크기 규칙만 씁니다.** 두 설정은 `vio_mono_health_rot_deg`를 180°로 둡니다. 회전 보정 rms는 이 값을 넘지 않으므로 회전 보정 규칙은 실패를 내지 않습니다.

- 저속 비행에서는 이동 보정이 거의 항상 5 mm보다 작습니다(EuRoC: 창의 56-89 %). 그래서 회전 보정 규칙은 "회전 보정 rms > 0.07°" 하나가 됩니다.
- 정상 실행의 회전 보정 rms 최대는 Unreal 0.041-0.049°, EuRoC 0.021-0.036°입니다. 기준과의 여유가 1.4-3배뿐이고, 실제 기체의 값은 모릅니다.
- bias 크기 규칙은 정상 실행과의 여유가 4배 이상입니다(\|ba\| 최대 0.47 m/s², \|bg\| 최대 0.024 rad/s). 대신 늦게 잡습니다. 위의 발산 실행에서 회전 보정 규칙은 7.0 s, bias 크기 규칙은 9.8 s에 실패를 냈습니다.
- 실제 기체의 상태 CSV(`prev_corr_rot`, `prev_corr_pos`)로 여유를 확인하면 회전 보정 규칙을 다시 켤 수 있습니다. 다시 켜려면 이 키를 지웁니다(기본값 0.07°).

### 9.3 좌표 변환

기호는 다음과 같습니다.

- w: VIO world. 중력 정렬이고 z축이 위쪽입니다. 중력 벡터는 (0, 0, −9.81) m/s²입니다(`constants::g`, `include/granite/utils/imu_types.h`). 초기화가 정렬한 중력을 이 벡터로 회전합니다(`src/vi_estimator/keypoint_vio.cpp`).
- i: VIO의 IMU frame. `mavlinkLoop()`이 `HIGHRES_IMU`의 x, y, z 값을 축을 바꾸지 않고 넘깁니다. 그래서 i는 FC body frame(FRD)입니다.
- a: ODOMETRY world. w를 z축으로 회전한 좌표계입니다. x축은 첫 상태에서 기체 앞쪽(i의 x축)을 수평면에 투영한 방향입니다. 원점은 첫 상태의 위치입니다.
- frd: LOCAL_FRD. z축이 아래쪽입니다.
- T_w_i: VIO 상태의 자세 R_w_i와 위치 p_w [m]. v_w [m/s]: VIO 상태의 world 속도.

w의 x축은 VIO 초기화가 정합니다. FC IMU는 z축이 아래쪽이므로 초기화의 `FromTwoVectors()`는 약 180° 회전을 고릅니다. 이 회전의 축은 기체의 작은 기울기가 정합니다. 그래서 w의 x축과 기체 앞쪽 사이의 각은 실행마다 다릅니다. `sendOdometry()`(`src/vio_app.h`)는 첫 상태에서 이 각을 없앱니다.

- 첫 상태의 기체 앞쪽 [w]: f = R_w_i₀ · (1, 0, 0)
- yaw [rad]: ψ = atan2(f_y, f_x)
- T_a_w = (R_z(−ψ), −R_z(−ψ) · p_w₀). 첫 상태의 기체 앞쪽은 a의 xz 평면에 있습니다. 그래서 첫 상태의 yaw는 0입니다.

`map_idx`가 바뀌면 `sendOdometry()`는 T_a_w를 다시 계산합니다. 새 metric estimator(`metric_run`)는 이전 estimator의 위치와 yaw에서 시작하므로 T_a_w를 그대로 씁니다.

R_frd_a = diag(1, −1, −1)입니다. 이것은 x축에 대한 180° 회전입니다. a의 y축은 기체 왼쪽이고, LOCAL_FRD의 y축은 기체 오른쪽입니다. body = i이므로 R_i_body = I입니다.

- 위치 [m]: p = R_frd_a · T_a_w · p_w
- 자세: R_frd_body = R_frd_a · R_a_w · R_w_i
- 속도 [m/s]: v_body = R_frd_bodyᵀ · (R_frd_a · R_a_w · v_w). 이 값은 body frame의 속도 R_w_iᵀ · v_w와 같습니다.

### 9.4 reset_counter

시작 값은 0입니다. 연속한 두 상태의 estimate가 다르면 1 올립니다. 255 다음은 0입니다.

| 조건 | 코드 |
|---|---|
| mono VIO가 새 metric estimator로 바뀜 (`VioStateData::metric_run`) | `src/vi_estimator/mono_vio.cpp` (`MetricRun::index`) |
| `map_idx`가 바뀜 | `src/vio_app.h` (`sendOdometry`) |

- mono VIO(`vio_mono_init_method: vo_alignment`)는 metric estimator의 health 검사가 실패하면 새 estimator를 초기화합니다(`vio_mono_reinit_max` > 0일 때, 기본값 0). 새 estimator는 이전 estimator가 낸 위치와 yaw에서 시작합니다. 그래서 원점은 바뀌지 않습니다. 그러나 속도, roll, pitch, scale, bias는 새로 추정한 값입니다. MAVLink 정의는 position, velocity, attitude 중 하나가 reset되어도 counter를 올리라고 합니다. 그래서 이때 1 올립니다.
- live VIO의 metric estimator(`KeypointVioEstimator`)는 `map_idx`가 항상 0입니다. `map_idx`가 바뀌는 estimator는 VO(`KeypointVoEstimator`)이고, VO는 ODOMETRY를 보내지 않습니다. `map_idx` 검사는 다른 estimator를 위해 남깁니다.
- `vio_mono_init_method: inertial`은 estimator를 다시 초기화하지 않습니다. counter는 0으로 유지됩니다.

### 9.5 시각과 FC 설정

`time_usec`는 측정 시각입니다(이미지 시각 + `camera_time_offset_ns`). 상태는 이 시각보다 VIO 처리 시간만큼 늦게 나갑니다. 이 지연은 `LIVE` 줄의 `latency_ms_p50`, `latency_ms_p95`입니다.

ArduPilot은 `correct_offboard_timestamp_usec_to_ms()`로 `time_usec`를 FC 시각으로 바꿉니다. 이 함수는 jitter correction으로 두 시계의 차이를 추정합니다. 이 추정은 가장 빨리 도착한 메시지의 지연을 0으로 봅니다. 그러면 최소 VIO 처리 시간도 0으로 보입니다. 이 경우 `VISO_DELAY_MS`를 VIO 지연에 맞추어야 합니다. [미검증] ArduPilot 코드와 FC 동작으로 확인하지 않았습니다.

FC 설정(ArduPilot 4.7.2)입니다. [미검증]

| 파라미터 | 값 |
|---|---|
| `VISO_TYPE` | 1 (MAVLink) |
| `VISO_QUAL_MIN` | 0 (기본). −1인 상태는 쓰지 않습니다 |
| `VISO_POS_M_NSE`, `VISO_YAW_M_NSE` | covariance를 NaN으로 보내므로 이 값이 위치 오차와 yaw 오차입니다 |
| `VISO_DELAY_MS` | 위의 시각 설명을 보십시오 |
| `EK3_SRC1_POSXY`, `EK3_SRC1_VELXY`, `EK3_SRC1_YAW` 등 | ExternalNav (6) |

## 10. 지상 시각화 메시지 (send_vis_msg)

드론에는 화면이 없으므로 VIO는 GUI 없이 실행합니다. `send_vis_msg`를 켜면 GUI가 그리는 요소를 지상의 PC로 보냅니다. GUI가 켜져 있어도 같이 보냅니다.

### 10.1 설정

| 키 (`--config-path`의 JSON) | 명령줄 | 기본값 (코드) | Pi 5 live 설정 | 뜻 |
|---|---|---|---|---|
| `send_vis_msg` | `--send_vis_msg` | false | true | 보낼지 |
| `vis_msg_ip` | `--vis_msg_ip` | 192.168.168.102 | 192.168.168.102 | 받는 PC의 IPv4 주소 |
| `vis_msg_port` | `--vis_msg_port` | 5100 | 5100 | 받는 PC의 UDP port |
| `vis_msg_image_hz` | `--vis_msg_image_hz` | 20 | 20 | 이미지 메시지의 최대 비율 [Hz]. 카메라 20 fps에서는 모든 frame입니다. 0이면 이미지 메시지가 없습니다 |
| `vis_msg_points_hz` | `--vis_msg_points_hz` | 2 | 2 | landmark 3D 점의 최대 비율 [Hz]. 0이면 보내지 않습니다 |
| `vis_msg_encode` | `--vis_msg_encode` | h264 | h264 | 이미지 압축. `h264`(OpenH264가 있는 빌드만) 또는 `mjpeg`(이미지마다 회색 JPEG) |
| `vis_msg_jpeg_quality` | `--vis_msg_jpeg_quality` | 80 | 80 | mjpeg의 JPEG 품질 (1-100) |
| `vis_msg_max_kbps` | `--vis_msg_max_kbps` | 2000 | 2000 | 평균 전송률의 상한 [kbit/s]. 상태 메시지는 항상 갑니다. 0이면 상한이 없습니다 |
| `vis_msg_h264_rate_control` | `--vis_msg_h264_rate_control` | bitrate_cap | bitrate_cap | h264 rate control. `quality`는 QP 고정이고 전송률 상한이 없습니다. `bitrate_cap`은 QP를 `vis_msg_h264_qp` 이상으로 두고 `vis_msg_max_kbps` 상한을 지킵니다 |
| `vis_msg_h264_qp` | `--vis_msg_h264_qp` | 24 | 24 | h264 QP (0-51, 작을수록 좋음). `bitrate_cap`에서는 QP의 하한입니다 |

- 명령줄 옵션이 JSON 키보다 우선합니다. 서비스(`granite-vio.service`)에서는 `/etc/default/granite-vio`의 `GRANITE_VIO_ARGS`에 옵션을 넣습니다. 예: `--vis_msg_ip 10.20.10.77`, JPEG로 바꾸려면 `--vis_msg_encode mjpeg`, 끄려면 `--send_vis_msg 0`.
- `capture_to_granite.py --target live`는 이 키들을 live 설정에 씁니다. 주소와 port는 `--vis-msg-ip`, `--vis-msg-port`로 바꿉니다.
- `vis_msg_encode h264`는 OpenH264(BSD)와 링크한 빌드에서만 됩니다. CMake가 `wels/codec_api.h`와 `libopenh264`를 찾으면 링크합니다. 없는 빌드는 경고를 출력하고 mjpeg로 보냅니다. 시각화 때문에 VIO가 멈추지 않게 하려는 것입니다. Pi에는 `libopenh264-7`이 있어야 합니다(`cross/pi5/build.sh` 머리말).

### 10.2 메시지

- **전송:** 메시지 하나가 1400 B 이하이면 UDP datagram 하나로 갑니다. 더 크면 1400 B씩 나눈 조각(fragment)으로 갑니다. 조각 하나는 Ethernet MTU(1500 B)보다 작으므로 IP가 다시 나누지 않습니다. 받는 쪽이 없어도 VIO에 영향이 없습니다.
- **조각:** 각 조각은 `fragment_seq`(메시지 번호), `fragment_index`, `fragment_count`, `fragment_data`만 가진 `GraniteVis`입니다. 받는 쪽은 같은 `fragment_seq`의 조각을 번호 순서로 이어 붙여 메시지 하나로 읽습니다. 조각 하나를 잃으면 그 메시지 전체를 잃습니다. 1 MB를 넘는 메시지는 보내지 않고 `skipped`에 셉니다.
- **형식:** protobuf 메시지 `GraniteVis` version 2(`doc/granite_vis.proto`)입니다. Pi에 protobuf와 zmq 개발 패키지가 없으므로, granite는 protobuf wire format을 직접 씁니다(`src/utils/vis_publisher.cpp`의 `ProtoWriter`). 받는 쪽은 어떤 protobuf 라이브러리로도 읽을 수 있습니다. id 필드는 앞 값과의 차이로 씁니다.
- **좌표:** VIO world(중력 정렬, z 위쪽, m)입니다. FC로 보내는 ODOMETRY의 world(LOCAL_FRD, yaw 0 정렬)와 다릅니다.

| 메시지에 들어가는 것 | 비율 | 내용 | 크기 (Unreal 640x640) |
|---|---|---|---|
| 상태 | 상태마다 (20 Hz) | 현재 pose, 속도, gyro와 accel bias, health, keyframe 여부, 최적화 창의 pose들 | 수백 byte |
| 이미지 | 최대 `vis_msg_image_hz` | 이미지(JPEG 또는 H.264 frame 하나), keypoint(1/8 px 단위 uint16)와 id, 카메라 모델과 parameter, `T_i_c` | JPEG q80 약 77 kB, H.264 bitrate_cap 평균 약 10 kB, keypoint 약 2 kB |
| landmark | 최대 `vis_msg_points_hz` | 창의 landmark 3D 점(float)과 id | 약 9 kB |

- **투영:** 메시지에 landmark 투영이 없습니다. 뷰어가 계산합니다. keypoint id와 같은 id의 landmark가 이 이미지에서 관측된 landmark입니다. 뷰어는 그 점을 `(T_w_i * T_i_c)^-1`로 카메라 좌표로 옮기고 카메라 모델로 투영합니다. landmark는 최대 0.5 s 전의 값이므로 granite GUI의 투영과 조금 다를 수 있습니다.
- **GUI가 없는 실행:** 추정기는 landmark 투영을 계산하지 않습니다(`vis_projections`). VIO thread의 일이 그만큼 줄어듭니다.
- **이미지 비율:** 이미지 시간으로 제한합니다. 시간 간격이 주기의 0.75 이상이면 이미지를 넣습니다. 그래서 카메라가 `vis_msg_image_hz`로 돌면 frame마다 이미지가 갑니다.
- **mjpeg 전송률 상한:** 보낸 byte(IP와 UDP header 포함)를 빚으로 셉니다. 빚은 `vis_msg_max_kbps`의 속도로 0까지 줄어듭니다. 이미지는 빚이 0일 때만 갑니다. 그래서 평균 전송률은 상한을 넘지 않고, 이미지 수가 줄어듭니다.
- **h264:** OpenH264를 thread 1개, constrained baseline profile(CAVLC), 낮은 복잡도로 씁니다. B frame이 없으므로 frame마다 바로 나옵니다. IDR frame은 1초마다이고, 그 앞에 SPS와 PPS가 있습니다. frame을 버리면 다음 frame을 디코드할 수 없으므로, h264는 전송률 상한 때문에 frame을 버리지 않습니다. 인코더의 frame skip도 끕니다.
- **h264 bitrate_cap:** OpenH264의 목표 bitrate를 1초마다 `0.95 × (vis_msg_max_kbps − 다른 데이터의 전송률)`로 맞춥니다. QP 하한은 `vis_msg_h264_qp`이므로 쉬운 장면은 상한보다 적게 씁니다. 움직임이 크면 QP를 올려 화질을 낮춥니다. 짧은 시간 동안은 상한을 넘을 수 있습니다.
- **송신 실패:** 조각 하나라도 보내지 못하면(소켓 buffer가 찬 경우 등) 그 메시지를 잃은 것으로 세고, h264는 다음 frame을 IDR로 만듭니다.
- **CPU 우선순위:** publisher thread는 `SCHED_IDLE`입니다(H.264 인코딩도 이 thread에서 합니다). 그래서 VIO thread가 CPU를 쓰는 동안 publisher는 기다립니다. 그 사이에 새 데이터가 오면 이전 데이터를 버리고 `skipped`에 셉니다.

전송률과 화질 측정 [검증, 데스크톱, Unreal Cam3 640x640 앞 200 프레임, 실시간 15.4 fps, 상한 2000 kbit/s, 각 1회]:

| 설정 | 받은 전송률 (조각과 header 포함) | 이미지 | PSNR 평균 (최소) |
|---|---|---|---|
| h264 bitrate_cap, QP 24 이상 (기본) | 1400 kbit/s | 74/74 | 30.6 dB (26.9) |
| h264 quality, QP 24 | 4748 kbit/s | 74/74 | 38.4 dB (37.8) |
| mjpeg q80 | 1608 kbit/s | 13/74 | 35.8 dB (34.5) |

- PSNR은 받은 이미지를 디코드해서 데이터셋의 원본 이미지와 비교한 값입니다.
- 상태 메시지 74개는 모든 경우에 다 갔습니다(skipped 0, `image_seq` 빈칸 0).
- 조각 전송 전에는 QP 24의 IDR frame(640x640에서 70-93 kB)이 UDP 한계(65507 B)를 넘어 이미지가 하나도 가지 않았습니다.
- Unreal Cam3는 빠르게 움직이는 장면입니다. Pi의 448x400 이미지는 화소 수가 44%이므로 frame이 더 작습니다.
- Pi 5에서 OpenH264의 CPU 사용량과 VIO 지연에 주는 영향은 측정하지 않았습니다 [미검증].

### 10.3 받는 쪽

**UVIFY VIO Viewer (`tools/vio_viewer`):** 받는 PC의 GUI입니다. mjpeg와 h264를 모두 보여 주고(AppImage에 OpenH264 포함), landmark 투영을 계산합니다. `tools/vio_viewer/README.md`에 사용법이 있습니다.

```bash
tools/vio_viewer/build_appimage.sh                        # tools/vio_viewer/dist/UVIFY_VIO_Viewer-x86_64.AppImage
./UVIFY_VIO_Viewer-x86_64.AppImage                        # 0.0.0.0:5100에서 받습니다
```

**`eval/scripts/vis_receiver.py`:** 의존성이 없는 확인용 스크립트입니다. JPEG 이미지만 보여 주고, 투영을 그리지 않습니다.

```bash
python3 eval/scripts/vis_receiver.py --port 5100          # 1초마다 수신 통계
python3 eval/scripts/vis_receiver.py --port 5100 --show   # OpenCV 창: JPEG 이미지와 위에서 본 궤적
```

- 통계: 초당 메시지, kB/s, 이미지 수, 네트워크 손실(`seq`의 빈칸), publisher가 건너뛴 수(`skipped`).
- `--show`: 왼쪽은 카메라 이미지와 keypoint(초록)입니다. 오른쪽은 위에서 본 궤적(흰색), 창의 상태(노랑), landmark(하늘색)입니다. q는 끝, c는 궤적 지우기입니다.
- 스크립트는 protobuf 라이브러리 없이 메시지를 읽습니다. numpy와 OpenCV는 `--show`에만 필요합니다.

### 10.4 장시간 실행의 메모리

publisher와 live 실행은 실행 시간에 비례해 커지는 데이터를 갖지 않습니다.

- **publisher:** 가장 최근의 시각화 데이터 하나만 갖습니다. 보내기 전에 새 데이터가 오면 이전 것을 버리고 `skipped`에 셉니다. 송신은 `MSG_DONTWAIT`이므로 네트워크가 막혀도 기다리지 않습니다. 추정기가 넣는 queue는 publisher 쪽 thread가 바로 비웁니다.
- **GUI가 없는 live 실행:** 상태 이력(궤적, 속도, bias 등 10개 vector)과 프레임 번호 표를 쌓지 않습니다. 궤적과 상태 CSV는 상태마다 파일에 씁니다. 지연 통계는 크기가 일정한 histogram(0.1 ms 구간, 2 s까지)입니다.
- **초기화 전:** 초기화기는 정지 bias용 가속도와 영상 이동을 최근 120 s만 남깁니다. 그래서 전원을 켜고 오래 기다려도 커지지 않습니다.
- GUI 실행과 데이터셋 실행은 그림과 평가에 쓰려고 이력을 계속 쌓습니다(Unreal 15.4 Hz에서 시간당 약 20 MB).

확인한 결과 [검증, 데스크톱, Unreal Cam3 전체 6118 프레임, 397 s 실시간, send_vis_msg 켬, 각 1회]:

| 실행 | 초기화 뒤 RSS | 증가율 |
|---|---|---|
| 데이터셋 모드 (이력 쌓음) | 105.5 → 107.8 MB | 시간당 약 20 MB. 상태 이력(상태당 약 280 B)과 지연 측정 표의 계산값과 같습니다 |
| 이력 없음 (GUI 없는 live와 같은 조건, 시험용 binary) | 104.8 → 105.2 MB | 100 s 구간마다 시간당 4.7, 1.8, 2.3, 1.3 MB로 줄어듭니다. 할당기가 안정되는 모양이고, 일정한 누수는 보이지 않습니다 |

- 두 실행 모두 상태 5992개를 모두 보냈습니다(이미지 메시지 약 1220개, skipped 0, 송신 오류 0). 수신은 평균 15.3 msg/s, 179 kB/s(약 1.4 Mbit/s)였습니다(그때의 설정: `vis_msg_image_hz` 5, q60, 전송률 상한 없음).
- send_vis_msg를 켜도 궤적은 바뀌지 않습니다(결정론 최대 속도, Cam3 1000 프레임, md5 같음). version 2에서도 같습니다(Cam3 800 프레임, md5 같음).
- Pi 5에서의 장시간 실행(수 시간)과 CPU 부하는 측정하지 않았습니다 [미검증].

## 11. 검증 상태 요약

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
| ODOMETRY 송신 코드 (9절) | [코드 확인] |
| FC가 ODOMETRY를 받아 EKF에 쓰는지, 축 방향이 맞는지 | [미검증] |
| ArduPilot의 시각 변환과 `VISO_DELAY_MS` | [미검증] |
