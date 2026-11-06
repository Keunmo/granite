# xeye 보정으로 Granite 단안 VIO 실행하기

이 문서는 Raspberry Pi 4에서 Granite 단안 VIO(mono VIO)를 실행하는 절차입니다. 실행 방법은 두 가지입니다.

- **데이터셋 실행:** 녹화된 EuRoC 데이터셋을 Pi 4에서 재생합니다. 기준 데이터셋은 Unreal `unreal_data_city_sky`의 Cam1입니다(4절).
- **live 실행:** Pi 4에 연결된 카메라와 IMU로 실행합니다. 기준 리그는 Arducam B0429(AR0234)와 MPU-9250(CP2112)입니다(5절).

두 방법 모두 `granite_vio`로 실행합니다. 화면은 Granite 기본 GUI(Pangolin)입니다. 화면이 없으면(`--show-gui 0` 또는 SSH) 결과 파일만 씁니다.

두 방법 모두 xeye capture의 보정(`capture.yaml`)을 Granite 입력으로 바꾸어서 씁니다(2절). 구조는 openvio의 `doc/xeye_to_openvio_guide.md`를 따릅니다. Pi 4의 주소는 `uvify@10.20.10.50`입니다.

capture.yaml은 데이터셋마다 다릅니다. 각 데이터셋에는 그 데이터셋의 capture.yaml만 씁니다.

| 데이터셋 또는 리그 | capture.yaml | 이 문서의 Granite 파일 |
|---|---|---|
| Pi 4 live 리그 (B0429 + MPU-9250) | Pi 4 `/home/uvify/workspace/openvio/config/capture.yaml` (데스크톱 사본 `~/workspace/openvio/config/capture.yaml`, md5 `d2d36cf7`) | `eval/config/live/b0429_pi_640x400_*.json` |
| unreal_data_city_sky | Pi 4 `/media/uvify/uvify_ssd/Data/unreal_data_city_sky/capture.yaml` (데스크톱 사본 `~/multipleye_docker/data/unreal_data_city_sky/capture.yaml`, md5 `38643b4e`) | `eval/config/raw_cam0..3_kb4_calib.json` |
| 260625_163357_smrig_ar0234 (다른 리그, 손으로 듦) | 그 데이터셋 폴더의 `capture.yaml` | `eval/config/live/smrig_cam1_672x600_*.json` |

상태 표기는 세 가지입니다.

- **[검증]**: 이 문서를 쓸 때 실행해서 확인한 내용입니다.
- **[코드 확인]**: 코드를 읽고 확인했지만 실행하지 않은 내용입니다.
- **[미검증]**: 확인하지 않은 내용입니다. 이 표기가 붙은 단계는 결과를 직접 확인하십시오.

## 빠른 시작

바이너리, 설정, 데이터는 Pi 4에 있습니다. 바이너리 폴더는 `~/granite_eval/bin/granite/`입니다. 이 폴더에는 가장 최근에 배포한 바이너리만 있습니다(3절).

### A. Pi 4에서 데이터셋 실행 (Unreal Cam1)

```bash
ssh 10.20.10.50
cd ~/granite_eval
B=bin/granite
# 실시간(15.4 fps) 실행, 앞 1000 프레임. 전체 구간은 --max-frames를 뺍니다(약 400 s).
$B/granite_vio --dataset-path data/city_sky_cam1 --dataset-type euroc --use-imu 1 \
    --cam-calib config/city_sky_cam1_kb4_calib.json --config-path config/oi_g40_det_gauge_gate_rest.json \
    --num-threads 4 --realtime --max-frames 1000 --out results/c1_rt
```

- 결과: `results/c1_rt_traj_tum.txt`(TUM, T_w_i), `results/c1_rt_states.csv`(상태 CSV). 끝의 `SUMMARY` 줄에 지연과 버린 이미지 수가 있습니다.
- SSH에는 `DISPLAY`가 없습니다. 그래서 GUI 없이 실행합니다. Pi 4 모니터에 GUI를 띄우는 방법은 7.3절에 있습니다.
- GUI가 열려 있으면 데이터가 끝나도 프로그램이 끝나지 않습니다. 창을 닫으면 끝납니다.
- 평가는 데스크톱에서 합니다(4.4절). Pi 4에는 numpy가 없습니다.

### B. Pi 4 live 실행

가장 쉬운 방법은 Pi 4 바탕화면의 **Granite Live Mono VIO** 아이콘입니다. 아이콘을 두 번 누르면 터미널 창과 GUI가 열립니다. 같은 실행을 터미널에서 할 수도 있습니다.

```bash
~/granite_eval/pi4_live_mono_vio.sh              # GUI와 녹화
~/granite_eval/pi4_live_mono_vio.sh --no-record  # 녹화 없음
~/granite_eval/pi4_live_mono_vio.sh --no-gui     # SSH: GUI 없음
```

- 스크립트는 CP2112의 I2C 번호를 스스로 찾습니다. 다른 번호를 쓰려면 `GRANITE_I2C=/dev/i2c-N`을 앞에 붙입니다.
- 결과는 `~/granite_eval/live/<날짜_시각>/`에 있습니다: `run_traj_tum.txt`, `run_states.csv`, `run.log`, `euroc/`(녹화).
- Ctrl+C를 누르거나 GUI 창을 닫으면 실행이 끝나고 결과를 씁니다. 터미널 창을 먼저 닫으면 결과의 끝부분을 잃을 수 있습니다.
- 다른 `bin/granite` 프로그램이 센서를 쓰고 있으면 스크립트는 실행하지 않습니다.
- 스크립트, 아이콘 파일, 바탕화면 파일의 원본은 저장소의 `scripts/pi4_live_mono_vio.*`입니다.
- 바탕화면의 **Live VIO** 아이콘은 openvio의 live VIO입니다. Granite와 다른 프로그램입니다.

직접 실행하는 명령은 다음과 같습니다.

```bash
ssh 10.20.10.50
cd ~/granite_eval
B=bin/granite
C=config/live
# 1. CP2112의 I2C 번호를 확인합니다. "CP2112 SMBus Bridge"인 줄의 번호를 씁니다.
for d in /sys/bus/i2c/devices/i2c-*; do echo $d $(cat $d/name); done | grep CP2112
# 2. VIO와 녹화를 같이 실행합니다. 리그를 2초 이상 정지한 다음 움직입니다. Ctrl-C 또는 창 닫기로 끝냅니다.
$B/granite_vio --live --use-imu 1 --cam-calib $C/b0429_pi_640x400_calib.json \
    --config-path $C/b0429_pi_640x400_handheld.json --i2c /dev/i2c-23 --out live/run1 --record live/run1_euroc
```

- 결과: `live/run1_traj_tum.txt`(TUM, T_w_i), `live/run1_states.csv`(상태 CSV), `live/run1_euroc/`(EuRoC 녹화).
- 끝에 나오는 `LIVE` 줄에서 fps, 손실, IMU, 지연을 확인합니다(6절).
- `granite_pi_cam_imu_vio`도 같은 GUI와 같은 실행 경로를 씁니다(5.2절).

## 1. 전체 흐름

| 단계 | 도구 | 입력 | 출력 |
|---|---|---|---|
| A. 변환 | `eval/scripts/capture_to_granite.py`, `eval/scripts/capture_to_euroc.py` | xeye `capture.yaml`, capture 데이터 | Granite 보정 JSON, VIO 설정 JSON, EuRoC 데이터셋 |
| B. 크로스 빌드 | 데스크톱 `~/cross/granite-pi4` | Granite 소스 | `granite_vio`, `granite_pi_cam_imu_vio`, `granite_mono_vio_dataset`, `libgranite.so` |
| C. 데이터셋 실행 | Pi 4 `granite_vio --realtime` | A의 EuRoC 데이터셋과 보정 | TUM 궤적, 상태 CSV |
| D. live 실행 | Pi 4 `granite_vio --live` | A의 보정과 설정 | TUM 궤적, 상태 CSV, EuRoC 녹화 |
| E. 녹화 재생 | `granite_vio --realtime` | D의 녹화 | live와 비교할 궤적 |
| F. 평가 | 데스크톱 `eval/scripts/eval_pose.py` | C 또는 E의 궤적, GT | ATE, 스케일, 판정 |

`granite_vio`의 세 모드(최대 속도, `--realtime`, `--live`)는 같은 추정기 경로를 씁니다 [코드 확인, `src/vio_app.h`]. 평가 러너 `eval/src/mono_vio_dataset.cpp`도 같은 경로입니다.

- 같은 설정 파일, `vio_mono_init_method: "inertial"`, `vio_enforce_realtime` 켬.
- `initialize()`에 바이어스 0을 줍니다. 보정 JSON의 바이어스는 IMU 샘플에서 뺍니다.
- flow 입력 큐는 4장입니다. 큐가 차면 가장 오래된 이미지를 버립니다.
- 상태 CSV의 열이 같습니다. live는 `latency_ms`에 노출 중심부터 상태 출력까지의 시간을 씁니다.

## 2. 변환

### 2.1 Pi 4 live용 변환

데스크톱의 Granite 저장소에서 실행합니다 [검증].

```bash
cd ~/workspace/granite_reliability
python3 eval/scripts/capture_to_granite.py ~/workspace/openvio/config/capture.yaml --downscale 3 --handheld \
    --i2c-device /dev/i2c-23 \
    --calib-out eval/config/live/b0429_pi_640x400_calib.json \
    --config-out eval/config/live/b0429_pi_640x400_handheld.json
scp eval/config/live/b0429_pi_640x400_* 10.20.10.50:granite_eval/config/live/
```

- 입력은 1920x1200 보정입니다. `--downscale 3`은 640x400 보정을 씁니다. 초점거리는 1/3이고, 주점은 `(c + 0.5) / 3 - 0.5`입니다.
- 출력에서 카메라 모델과 맞춤 오차를 확인합니다(2.2절).
- 여러 카메라 capture는 `--camera cam1`처럼 카메라를 고릅니다.

데이터셋 실행용 변환(Unreal)은 4.1절에 있습니다.

### 2.2 카메라 모델

xeye의 uwfisheye는 Granite에 없습니다. 변환은 kb4, ds, eucm을 시야 전체(θ ≤ max_theta)에 최소제곱으로 맞춥니다. 그리고 두 방향을 Granite 공식으로 검사합니다. kb4의 역투영은 Newton 3회만 합니다. 그래서 역투영도 따로 검사합니다. kb4의 최대 오차가 `--max-error-px`(0.5 px) 이하이면 kb4를 씁니다. 아니면 최대 오차가 가장 작은 모델을 씁니다.

| 카메라 | 크기 | 선택 | RMS | 최대 | 비고 |
|---|---|---|---|---|---|
| B0429 Pi 리그 | 640x400 | kb4 | 0.013 px | 0.062 px | eucm 최대 0.12 px, ds 2.9 px |
| B0429 Pi 리그 | 1920x1200 | kb4 | 0.038 px | 0.19 px | |
| smrig_ar0234 cam1 | 672x600 | eucm | 0.27 px | 0.77 px (경고) | kb4 투영은 0.19 px, 3회 역투영은 279 px |

[검증, 2026-11-06]

Unreal의 카메라는 계수가 한 개인 등거리 uwfisheye입니다. 이 모델은 k1..k4 = 0인 kb4와 정확히 같습니다. 그래서 `capture_to_euroc.py --calib-out`이 보정을 바로 씁니다 [코드 확인].

### 2.3 변환이 쓰는 값

| 값 | 출처 |
|---|---|
| `T_imu_cam` | `(imu2rig_pose)⁻¹ · cam2rig_pose` |
| `calib_accel_bias`, `calib_gyro_bias` | `bias_acc`, `bias_gyro`. 원시 샘플일 때만 씁니다(live). `calibrated: true` capture의 재생은 0입니다. |
| IMU 노이즈 | Allan 결과(`calib.<imu>.intrinsic.noise` 또는 `--imu-intrinsics`). 없으면 openvio `config/pi5_b0429.yaml` 값(MPU-9250 데이터시트 값 × 2, 미보정): accel 6e-3, gyro 3.5e-4, accel walk 3e-3, gyro walk 2.2e-5 |
| `cam_time_offset_ns` | live: 0. capture 재생(`--target capture`): `time_offset` |

시간 오프셋: live 앱의 이미지 시각은 노출 중심입니다(`CLOCK_MONOTONIC`). xeye의 `time_offset`은 xeye 이미지 시각(DQBUF 뒤)에 맞는 값입니다. Granite는 오프셋을 실행 중에 추정하지 않습니다. 그래서 live에는 0을 씁니다. live의 실제 오프셋은 측정하지 않았습니다 [미검증].

### 2.4 VIO 설정

설정은 최종 후보 `eval/config/oi_g40_det_gauge_gate_rest.json`을 base로 씁니다. 데이터셋 실행(4절)은 이 파일을 그대로 씁니다. live 변환은 다음 값만 바꿉니다.

| 키 | 값 (Pi 4 640x400) | 이유 |
|---|---|---|
| `vio_use_deterministic` | false | live에서는 결정론 합산이 필요 없습니다(약 7% 느림). |
| `vio_mono_init_max_still_disparity` | 8.7 → 6.05 px | 움직임 기준입니다. 평가 카메라 178.9 px/rad 대 이 카메라 124.5 px/rad의 비율로 줄입니다. |
| `vio_mono_init_max_rest_disparity` | 1.0 → 2.0 (`--handheld`), 다음에 해상도 비례 (640x400: 1.392 px) | 2.5절 |
| `vio_mono_init_excitation_accel_std` | 0.3 (최종 후보와 같음) | 2.5절 |
| `camera.*` | 1920x1200, downscale 3, 15 fps, 자동 노출 5 ms 한도, 측광 원 `[310.2, 202.5, 203.7]`, `/dev/i2c-23` | live 센서 옵션 |

- 픽셀 노이즈와 셀 크기(`optical_flow_detection_grid_size` 40 px, `vio_obs_std_dev` 등)는 그대로 둡니다. openvio 변환과 같은 규칙입니다.
- `--handheld` 없이 만든 설정(`b0429_pi_640x400_config.json`)은 최종 후보의 시작 규칙(정지 이동 1.0을 해상도 비례로 줄인 값, 가속 0.3)을 씁니다. 손으로 드는 리그에는 권장하지 않습니다.

### 2.5 정지 시작 요구와 `--handheld`

최종 후보는 정지 상태에서만 초기화합니다. 이동 중에는 초기화하지 않고 정지를 기다립니다. 초기화 조건은 다음과 같습니다 [코드 확인, `mono_inertial_initializer.cpp`].

1. 2 s 창의 앞쪽 반(1 s)은 정지입니다. 특징점의 평균 이동이 `max_still_disparity` 이하이고, 가속도 표준편차가 `excitation_accel_std` 이하입니다.
2. 뒤쪽 반(1 s)은 움직입니다. 평균 이동이 `max_still_disparity`보다 크고, 가속도 표준편차가 `excitation_accel_std` 이상입니다.
3. `max_rest_disparity`가 0보다 크면 앞쪽 반의 평균 이동이 이 값 이하여야 합니다. 이 조건은 등속 운동을 정지로 판단하지 않게 합니다(Unreal에서 4 m/s 등속은 2.8 px, 정지는 0.004 px).

smrig_ar0234 cam1(손으로 든 리그, 0~8.3 s 정지 후 천천히 움직임) 재생 결과입니다 [검증, 데스크톱].

| 설정 | 초기화 | 결과 (rovins2 대비) |
|---|---|---|
| 최종 후보 값(정지 이동 0.77 px, 가속 1.0) | 350 s 동안 없음 | 앞쪽 반의 이동이 0.77 px를 넘은 다음에야 뒤쪽 반이 6.7 px를 넘습니다. |
| 정지 이동 끔, 가속 1.0 | 8.65 s | 앞쪽 반에 움직임이 들어감(이동 3.99 px, bg 0.03 rad/s) |
| 정지 이동 끔, 가속 0.3 | 7.93 s (openvio도 7.93 s) | SE3 ATE 0.66 m, Sim3 스케일 1.019, 길이 204.7 m 대 202.7 m, 발산 없음 |
| 정지 이동 1.5 (Unreal 기준), 가속 0.3 | 8.2 s | 길이 207.3 m, 발산 없음 |
| `--handheld` (정지 이동 2.0, Unreal 기준), 가속 0.3 | 7.93 s | 정지 이동을 끈 경우와 같은 시각, 길이 204.7 m, 발산 없음 |

정지 이동 기준을 끄면 안 됩니다. Unreal(capture.yaml: `/media/uvify/uvify_ssd/Data/unreal_data_city_sky/capture.yaml`, Pi 4)에서 확인한 결과입니다 [검증, 데스크톱 결정론 실행].

| 설정 | 정지 시작 Cam0–3 (1000 프레임) | 정지 시작 Cam0–3 (전체 구간) | 이동 중 시작 17개 |
|---|---|---|---|
| 최종 후보 (정지 이동 1.0, 가속 1.0) | 4/4 PASS | 4/4 PASS | 모두 초기화 안 함 |
| 가속 0.3만 바꿈 | 4/4 PASS | 4/4 PASS, ATE 같음 | 모두 초기화 안 함 |
| 정지 이동 끔, 가속 0.3 | 4/4 PASS | — | 17/17 발산 (등속을 정지로 판단) |
| 정지 이동 1.5, 가속 0.3 | 4/4 PASS | — | 모두 초기화 안 함 |
| `--handheld` (정지 이동 2.0, 가속 0.3) | 4/4 PASS | 4/4 PASS, ATE 같음 | 모두 초기화 안 함 |

- 이동 중 시작 17개는 Cam0–3의 시작 프레임 1000, 1150, 2550, 3200과 Cam1의 5800입니다.
- 기준 2.0은 Unreal 4 m/s 등속(2.8 px)보다 작습니다. 더 느린 등속(약 2.9 m/s 이하)은 정지로 판단할 수 있습니다 [미검증].
- 2026-11-06부터 최종 후보 설정은 가속 0.3을 명시합니다. 코드 기본값은 1.0 그대로입니다. 그래서 이 키가 없는 이전 설정의 결과는 바뀌지 않습니다. 가속 0.3은 Cam1과 Cam2의 1000 프레임 결정론 궤적이 1.0과 바이트 단위로 같습니다 [검증, 데스크톱].
- 최종 후보의 정지 이동 기준은 1.0 그대로입니다. live 변환의 `--handheld`만 2.0을 씁니다.

리그를 움직일 때 다음을 지킵니다.

- 시작 전에 리그를 2초 이상 완전히 정지합니다. 손으로 들고 있지 말고 바닥이나 책상에 둡니다.
- 그 다음 1초 안에 분명하게 움직입니다. 들어 올리거나 옮기는 움직임(가속)이 필요합니다. 제자리 회전만으로는 가속도 조건을 못 넘을 수 있습니다.
- 초기화가 되지 않으면 리그를 다시 2초 이상 정지한 다음 다시 움직입니다. 초기화 전에는 상태 출력이 없습니다.
- Pi 4 live 리그의 정지 상태 가속도 표준편차는 0.017~0.032 m/s²입니다 [검증]. 0.3보다 충분히 작습니다.
- Pi 4 live 리그를 책상에 두었을 때 Granite 트랙의 앞쪽 반 이동은 중앙값 0.38 px, 최대 1.23 px입니다 [검증, 60 s]. 최종 후보 값을 비율로 줄인 기준(0.70 px)보다 클 때가 있습니다. `--handheld` 기준(1.392 px)보다는 작지만 여유가 작습니다. 정지해 있어도 초기화가 늦어질 수 있습니다 [미검증].
- 가속도 기준은 앞쪽 반에도 쓰입니다. 앞쪽 반의 가속도 표준편차가 기준 이하여야 정지입니다. 기준 0.3에서는 모터 진동처럼 작은 흔들림도 정지를 막을 수 있습니다. 드론은 모터를 켜기 전에 초기화합니다 [미검증, 드론 진동 미측정].

## 3. 크로스 빌드와 배포

데스크톱에서 빌드합니다. Pi 4에서는 빌드하지 않습니다 [검증].

```bash
cd ~/cross/granite-pi4/build_live      # GRANITE_SRC=~/workspace/granite_reliability
make -j14
F="granite_vio granite_pi_cam_imu_vio granite_mono_vio_dataset libgranite.so"
ssh 10.20.10.50 "mkdir -p ~/granite_eval/bin/granite"
for f in $F; do scp $f 10.20.10.50:granite_eval/bin/granite/.$f.new; done
SHA=$(git -C ~/workspace/granite_reliability rev-parse --short HEAD)
ssh 10.20.10.50 "cd ~/granite_eval/bin/granite && for f in $F; do mv -f .\$f.new \$f; done && echo $SHA \$(date -Is) > DEPLOYED"
```

- live 스크립트와 바탕화면 아이콘이 바뀌었으면 같이 복사합니다.

  ```bash
  cd ~/workspace/granite_reliability
  scp scripts/pi4_live_mono_vio.sh scripts/pi4_live_mono_vio.svg 10.20.10.50:granite_eval/
  scp scripts/pi4_live_mono_vio.desktop 10.20.10.50:Desktop/
  ssh 10.20.10.50 'chmod +x ~/granite_eval/pi4_live_mono_vio.sh ~/Desktop/pi4_live_mono_vio.desktop &&
    DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus gio set ~/Desktop/pi4_live_mono_vio.desktop metadata::trusted true'
  ```

- 파일을 임시 이름으로 복사한 다음 이름을 바꿉니다. 같은 이름으로 바로 덮어쓰면 실행 중인 프로그램이 쓰는 `libgranite.so`가 바뀝니다. 그러면 그 실행이 비정상 종료할 수 있습니다. 실행 중인 바이너리는 바로 덮어쓰기가 실패합니다(`dest open ... Failure`).

- 배포는 항상 같은 폴더 `~/granite_eval/bin/granite/`를 덮어씁니다. 이 폴더에는 가장 최근에 배포한 바이너리만 있습니다.
- 배포한 커밋은 `~/granite_eval/bin/granite/DEPLOYED`에 있습니다.

- 새 빌드 폴더를 만들 때: `cmake .. -DCMAKE_TOOLCHAIN_FILE=$HOME/sysroots/pi4-ubuntu2404.toolchain.cmake -DGRANITE_SRC=$HOME/workspace/granite_reliability`
- 실행 파일은 같은 폴더의 `libgranite.so`를 씁니다(RUNPATH `$ORIGIN`).
- 2026-11-06에 배포한 커밋은 `355cc87`입니다.
- `bin/cross_*` 폴더와 `bin/granite_mono_vio_dataset`은 평가 보고서의 바이너리입니다. 평가 보고서와 Pi 4 배치 스크립트(`scripts/batch_*.sh`, `scripts/run_pi4.sh`)가 이 경로를 씁니다.
- `granite_vio`는 Pi 4용 Pangolin 정적 라이브러리를 씁니다. 이 라이브러리는 `~/cross/pangolin-pi4/build/src/libpangolin.a`입니다. 다시 만드는 방법은 다음과 같습니다.
  - 소스: `thirdparty/Pangolin`.
  - GLEW 헤더: Pi 4 sysroot에 없습니다. 데스크톱 `/usr/include/GL/glew.h`(같은 2.2.0)를 `~/cross/pangolin-pi4/glew_include`에 복사해서 씁니다.
  - 옵션: `-include cstdint -DGLEW_NO_GLU`, video, python, 도구는 끕니다. 명령은 `~/cross/pangolin-pi4/cmake.log`의 맨 앞에 있습니다.
- Pi 4 크로스 빌드에는 rosbag이 없습니다(`GRANITE_NO_ROSBAG`). 그래서 `--dataset-type bag`은 쓸 수 없습니다. `euroc`을 씁니다.
- 데스크톱에서 `granite_vio`의 최대 속도 궤적은 평가 러너와 바이트 단위로 같습니다. 실시간 결과도 러너와 같습니다(상태 872개, 버림 0, Sim3 스케일 0.9967, PASS) [검증, Cam1 1000 프레임].
- Pi 4에서는 `granite_vio --help` 실행과 공유 라이브러리 연결만 확인했습니다. Pi 4의 데이터셋 실행과 live 실행은 확인하지 않았습니다 [미검증].

## 4. Pi 4에서 데이터셋 실행

### 4.1 데이터셋 만들기 (데스크톱)

Pi 4에는 Unreal Cam1 데이터셋이 이미 있습니다. 다른 카메라나 다른 데이터셋을 쓸 때만 이 절차를 따릅니다.

1. 데스크톱에서 capture를 EuRoC로 바꿉니다. 같은 명령이 kb4 보정 JSON도 씁니다 [검증].

   ```bash
   cd ~/workspace/granite_reliability/eval
   R=~/multipleye_docker/data/unreal_data_city_sky
   python3 scripts/capture_to_euroc.py $R data/cam1 --camera Cam1 --jobs 5 --calib-out config/raw_cam1_kb4_calib.json
   ```

2. 데이터셋과 보정을 Pi 4에 복사합니다.

   ```bash
   rsync -a data/cam1/ 10.20.10.50:granite_eval/data/city_sky_cam1/
   scp config/raw_cam1_kb4_calib.json 10.20.10.50:granite_eval/config/city_sky_cam1_kb4_calib.json
   ```

- 출력은 640x640 회색 PNG입니다. RGB 1280x1280을 회색으로 바꾸고 2x2 평균으로 줄입니다. openvio의 `capture.resize_half`와 같은 조건입니다.
- 앞부분을 잘라 낸 부분 집합(예: 1000 프레임부터 시작)은 `eval/scripts/make_subset.py`로 만듭니다. Pi 4의 `data/cam1_s1000`이 그 예입니다.
- xeye의 BAYER 또는 GRAY capture(예: smrig)는 `--node`, `--imu`, `--downscale`을 줍니다. 보정은 `capture_to_granite.py --target capture`로 씁니다(7.2절).

Pi 4에 있는 파일입니다 [검증].

| 파일 | 내용 |
|---|---|
| `~/granite_eval/data/city_sky_cam1/` | Unreal Cam1 전체, 6118 프레임, 640x640, 15.4 fps, 1.2 GB |
| `~/granite_eval/data/cam1_s1000/` | Cam1, 1000 프레임부터 시작하는 부분 집합 |
| `~/granite_eval/config/city_sky_cam1_kb4_calib.json` | Cam1 보정. 데스크톱 `eval/config/raw_cam1_kb4_calib.json`과 같음(md5 `7a8791b6`) |
| `~/granite_eval/config/oi_g40_det_gauge_gate_rest.json` | 최종 후보 설정(가속 0.3). 데스크톱 파일과 같음(md5 `0a495589`) |
| `~/granite_eval/config/oi_g40_det_gauge_gate_rest_exc1.0.json` | 이전 최종 후보 설정(가속 1.0, md5 `b5dcdd6f`). 4.3절과 4.4절의 실측은 이 설정의 결과입니다. |

### 4.2 실행

```bash
cd ~/granite_eval
B=bin/granite
D="--dataset-path data/city_sky_cam1 --dataset-type euroc --cam-calib config/city_sky_cam1_kb4_calib.json"
C=config/oi_g40_det_gauge_gate_rest.json
# 실시간, 전체 구간
$B/granite_vio $D --config-path $C --use-imu 1 --num-threads 4 --realtime --out results/c1_rt_full
# 최대 속도(모든 이미지 처리), 앞 1000 프레임
$B/granite_vio $D --config-path $C --use-imu 1 --num-threads 4 --max-frames 1000 --out results/c1_max_1k
```

| 옵션 | 뜻 |
|---|---|
| `--use-imu 1` | 단안 VIO입니다. 카메라 1대와 IMU입니다. `0`은 단안 VO입니다. |
| `--realtime` | 이미지를 데이터 시각에 공급합니다. flow 입력에 4장보다 많이 쌓이면 가장 오래된 이미지를 버립니다. live와 같은 규칙입니다. 추정기도 `vio_enforce_realtime`을 씁니다. |
| (없음) | 최대 속도입니다. 모든 이미지를 처리합니다. 결정론 설정이면 실행마다 궤적이 같습니다. |
| `--out PREFIX` | `PREFIX_traj_tum.txt`와 `PREFIX_states.csv`를 씁니다. 평가 러너와 같은 형식입니다. |
| `--num-threads N` | TBB 스레드 수입니다. 데이터셋 실행은 4, live는 3을 씁니다. |
| `--max-frames N` | 앞 N 프레임만 씁니다. 0은 모두입니다. |
| `--show-gui 0` | GUI 없이 실행합니다. `DISPLAY`가 없으면 자동으로 GUI 없이 실행합니다. |
| `--cam-time-offset-ns N` | 이미지 시각에 더합니다. 보정 JSON의 오프셋은 읽지 않습니다. |
| `--queue-limit N` | 실시간 모드의 flow 입력 큐 크기입니다(기본 4). |
| `--save-trajectory tum` | 끝에 Granite 형식의 궤적 파일(`trajectory.txt`)도 씁니다. 평가에는 `--out`을 씁니다. |

- 한 번에 한 실행만 합니다. 동시에 실행하면 실시간 결과가 나빠집니다.
- 실시간 모드에서는 스레드 타이밍 때문에 실행마다 궤적이 조금 다릅니다.
- 평가 러너 `granite_mono_vio_dataset`도 같은 폴더에 있습니다. 옵션은 같고, `--dataset-type`, `--use-imu`가 없습니다. `--preload`(이미지를 미리 읽음)는 러너에만 있습니다.

### 4.3 결과 확인

끝의 `SUMMARY` 줄의 값입니다. `granite_vio`와 평가 러너가 같은 키를 씁니다.

| 값 | 뜻 | Cam1 전체 구간 실시간 실측 [검증, 평가 러너 `cross_9788391`] |
|---|---|---|
| `frames`, `fed`, `dropped_input` | 이미지 수, 공급한 수, 버린 수 | 6118, 6118, 0 |
| `states` | 출력한 상태 수 | 5990 |
| `metric_initialized` | 초기화 성공(1) | 1 |
| `first_state_wall_s` | 첫 상태까지의 실행 시간 | 8.4 s |
| `latency_ms_p50/p95/max` | 이미지 공급부터 상태 출력까지 | 40 / 50 / 1069 ms (최대는 초기화 직후 한 번) |
| `maxrss_mb` | 최대 메모리 | 86 MB |

### 4.4 평가 (데스크톱)

1. 궤적을 데스크톱으로 가져옵니다.

   ```bash
   cd ~/workspace/granite_reliability
   scp 10.20.10.50:granite_eval/results/c1_rt_full_traj_tum.txt /tmp/
   ```

2. GT와 비교합니다.

   ```bash
   python3 eval/scripts/eval_pose.py /tmp/c1_rt_full_traj_tum.txt \
       ~/multipleye_docker/data/unreal_data_city_sky/gt_poses.txt --abs-window --json /tmp/c1.json
   ```

- 판정은 `verdict_abs`를 읽습니다. 기준은 `eval/granite_pi4_status_report.md`의 "판정 기준"에 있습니다.
- 기준 결과 [검증, Pi 4 Cam1 실시간]: 앞 1000 프레임은 ATE 0.35 m로 PASS입니다. 전체 구간은 ATE 5.87 m이고, 정지 직전 구간(382.3 s, 389.8 s)의 창 오차로 FAIL입니다. 자세한 내용은 같은 보고서에 있습니다.

## 5. Pi 4 live 실행

### 5.1 준비

- `/boot/firmware/config.txt`: `camera_auto_detect=0`, `dtoverlay=arducam-pivariety` [검증, 설정 확인].
- 카메라: `/sys/class/video4linux/video0/name`이 `unicam-image`입니다 [검증].
- IMU: CP2112 번호는 다시 꽂거나 재부팅하면 바뀔 수 있습니다. 빠른 시작 B의 1번 명령으로 확인합니다. 2026-11-06에는 `/dev/i2c-23`입니다 [검증].
- 카메라와 IMU는 한 프로세스만 씁니다. openvio나 `imu_fifo_test`를 같이 실행하지 않습니다.

### 5.2 명령

```bash
cd ~/granite_eval; B=bin/granite; C=config/live
# VIO + 녹화 (빠른 시작 B와 같음)
$B/granite_vio --live --use-imu 1 --cam-calib $C/b0429_pi_640x400_calib.json \
    --config-path $C/b0429_pi_640x400_handheld.json --i2c /dev/i2c-23 --out live/run1 --record live/run1_euroc
# 같은 실행을 live 앱의 옵션으로
$B/granite_pi_cam_imu_vio --calib $C/b0429_pi_640x400_calib.json --config-path $C/b0429_pi_640x400_handheld.json \
    --i2c /dev/i2c-23 --out live/run1 --record live/run1_euroc
# 센서만 확인 (VIO 없음, 10 s)
$B/granite_pi_cam_imu_vio --config-path $C/b0429_pi_640x400_handheld.json --i2c /dev/i2c-23 --capture-only --duration 10
# 녹화만 (VIO 없음)
$B/granite_pi_cam_imu_vio --config-path $C/b0429_pi_640x400_handheld.json --i2c /dev/i2c-23 --record live/rec1 --duration 30
```

- `granite_vio --live`와 `granite_pi_cam_imu_vio`(VIO)는 같은 코드(`src/vio_app.h`)와 같은 Pangolin GUI를 씁니다. 옵션 이름만 다릅니다(`--cam-calib` 대 `--calib`, `--show-gui 0` 대 `--no-gui`).
- VIO가 없는 두 모드(`--capture-only`, 보정 없는 `--record`)는 `granite_pi_cam_imu_vio`에만 있습니다. 이 두 모드는 Pangolin GUI 대신 이미지만 보이는 OpenCV 창을 씁니다.
- 센서 옵션(`--fps`, `--downscale`, `--exposure`, `--max-exposure`, `--i2c` 등)은 두 프로그램이 같습니다. 명령줄 옵션은 설정 파일의 `camera.*` 키보다 우선합니다.
- `--duration`을 주지 않으면 Ctrl-C 또는 창 닫기까지 실행합니다.
- `--num-threads`는 3을 권장합니다. 카메라와 IMU 스레드가 나머지 코어를 씁니다.
- GUI는 최근 이미지 300장(약 20 s)만 보관합니다. 더 오래된 프레임을 고르면 이미지가 바뀌지 않습니다.
- 실행 중에는 매초 상태 줄(`t=... input_fps=... imu_hz=... tracking=... support=...`)이 나옵니다.

### 5.3 프레임률

기본은 센서 15 fps입니다. Granite는 15 fps에서 검증했습니다. 30 fps는 시험하지 않았습니다 [미검증]. 근거는 다음과 같습니다 [검증, Pi 4].

- 카메라 스레드: 1920x1200 RAW10 변환과 1/3 축소에 프레임당 약 37 ms(버퍼 시각부터 큐까지)가 걸립니다.
- 센서는 30 fps도 손실 없이 줍니다(29.75 fps, 손실 0).
- live 프로세스 CPU(초기화 전, 카메라 + flow + 초기화기): 15 fps 0.61 코어, 30 fps 1.09 코어.
- 추정기: smrig cam1(672x600, 약 18 fps) 실시간 재생에서 입력 손실 0, 지연 p50 37 ms, p95 45 ms, 최대 1021 ms(초기화 직후 한 번)입니다. 이때 CPU는 평균 1.76 코어입니다. 최대 속도에서는 프레임당 19 ms입니다.
- 30 fps로 비례 계산하면 추정기만 약 2.9 코어가 필요합니다. 카메라 변환까지 더하면 4코어 Pi 4의 한계에 가깝습니다. 그래서 15 fps를 기본으로 합니다. 이 계산은 추정입니다.

## 6. live 결과 확인

끝의 `LIVE` 줄의 값입니다.

| 값 | 뜻 | Pi 4 정지 상태 실측 [검증, 2026-11-06, 이전 live 앱, 커밋 `1af850d`] |
|---|---|---|
| `fps` | 받은 이미지/초 | 14.90 (센서 15.00) |
| `sensor_drops` | 센서가 버린 프레임(sequence 빈칸) | 0 |
| `queue_drops` | flow 입력 큐와 장치 큐가 버린 이미지 | 0 |
| `imu_hz` | 받은 IMU 샘플/초 | 191~192 |
| `imu_i2c_errors` | CP2112 전송 실패 수 | 30 s에 12, 60 s에 28 (`doc/mpu6500-cp2112-issue.md`) |
| `image_delay_ms_mean/max` | 버퍼 시각부터 큐까지 | 37 / 53 ms |
| `latency_ms_p50/p95` | 노출 중심부터 상태 출력까지 | 정지 상태에서는 초기화가 없어서 값이 없습니다 |
| `brightness_mean` | 측광 원 안의 평균 밝기 | 99.7 (목표 100) |
| `first_state_data_s` | 첫 상태의 데이터 시각(첫 이미지 기준) | 정지 상태에서는 -1 (초기화 없음, 정상) |

- 매초 줄의 `tracking`과 `support`를 봅니다. 초기화 전에는 `initializing`입니다. 초기화 뒤에는 `support=metric`이 정상입니다.
- 정지 상태로만 두면 초기화하지 않습니다. 이것은 설계대로의 동작입니다 [검증, 60 s].
- 속도(`vx, vy, vz`, 상태 CSV)가 손으로 드는 속도(0.1~1.5 m/s)보다 계속 커지면 발산입니다.
- 같은 자리로 돌아오는 경로로 시험하면 끝점과 시작점의 거리로 drift를 볼 수 있습니다.
- 움직이는 live 시험은 아직 하지 않았습니다 [미검증].

## 7. 녹화 재생과 GUI

### 7.1 live 녹화를 Pi 4에서 재생

`--record`는 VIO와 같은 실행의 이미지와 IMU를 EuRoC로 씁니다. 재생은 4절의 `granite_vio --realtime`을 씁니다. 재생은 live와 같은 실시간 규칙을 씁니다 [검증, 이전 live 앱과 평가 러너, 정지 녹화 60 s: live와 재생 모두 상태 0개, 초기화 시도의 이유와 이동 분포가 같음(중앙값 0.38 px)].

```bash
cd ~/granite_eval; B=bin/granite; C=config/live
$B/granite_vio --dataset-path live/run1_euroc --dataset-type euroc --use-imu 1 --cam-calib $C/b0429_pi_640x400_calib.json \
    --config-path $C/b0429_pi_640x400_handheld.json --out live/run1_replay --num-threads 3 --realtime
```

- 실시간 모드에서는 스레드 타이밍 때문에 live와 재생의 궤적이 정확히 같지 않습니다.
- `--realtime` 없이 실행하면 모든 이미지를 처리합니다(최대 속도).

움직이는 녹화의 Pi 4 재생 예: smrig cam1(7.2절의 데이터를 Pi 4 `data/smrig_cam1`에 복사, `--realtime --preload`)은 7.93 s에 초기화하고 끝까지 추적합니다. rovins2 대비 SE3 ATE 0.58 m, Sim3 스케일 1.022, 길이 204.8 m 대 202.6 m입니다 [검증].

### 7.2 xeye capture를 데스크톱에서 재생

예: smrig_ar0234 cam1(BAYERGR8, 1344x1200 → 672x600) [검증].

```bash
cd ~/workspace/granite_reliability
D=~/multipleye_docker/data/260625_163357_smrig_ar0234
python3 eval/scripts/capture_to_euroc.py $D eval/data/smrig_cam1 --camera cam1 --node multicam_lux --imu imu_lux --downscale 2
python3 eval/scripts/capture_to_granite.py $D/capture.yaml --camera cam1 --downscale 2 --target capture --handheld \
    --calib-out eval/config/live/smrig_cam1_672x600_calib.json --config-out eval/config/live/smrig_cam1_672x600_handheld.json
sh eval/src/build.sh live      # 처음 한 번
eval/bin/live/granite_mono_vio_dataset --dataset-path eval/data/smrig_cam1 \
    --cam-calib eval/config/live/smrig_cam1_672x600_calib.json --config-path eval/config/live/smrig_cam1_672x600_handheld.json \
    --out eval/results/smrig_c1 --cam-time-offset-ns 2653760
```

- `--cam-time-offset-ns`는 변환 출력에 나오는 값입니다. 러너와 `granite_vio`는 보정 JSON의 오프셋을 읽지 않습니다.
- GUI로 보려면 같은 옵션으로 `build_gui/granite_vio --dataset-path eval/data/smrig_cam1 --dataset-type euroc --use-imu 1 ... --cam-time-offset-ns 2653760`을 실행합니다(7.3절).
- 이 capture의 IMU는 바이어스를 뺀 값(`calibrated: true`)입니다. 그래서 `--target capture`는 바이어스 0을 씁니다.

### 7.3 GUI (`granite_vio`)

`granite_vio`는 카메라 1대와 IMU(`--use-imu 1`)를 받으면 단안 VIO 추정기(`MonoVioEstimator`)를 씁니다. 화면은 기존 스테레오 VIO와 단안 VO의 Pangolin 화면과 같습니다. 데이터셋 실행, 실시간 실행, live 실행이 모두 이 화면을 씁니다.

데스크톱 빌드:

1. GUI용 빌드 폴더를 만듭니다. 저장소의 `build/`는 GUI를 끈 설정이므로 쓰지 않습니다.

   ```bash
   cd ~/workspace/granite_reliability
   mkdir -p build_gui && cd build_gui
   cmake .. -DCMAKE_BUILD_TYPE=Release -DGRANITE_BUILD_GUI=ON -DGRANITE_BUILD_PI_CAMERA=ON
   make -j14 granite_vio granite_pi_cam_imu_vio
   ```

2. 데이터셋으로 실행합니다. 예는 Unreal Cam1 1000 프레임의 실시간 실행입니다.

   ```bash
   cd ~/workspace/granite_reliability
   build_gui/granite_vio --dataset-path eval/data/cam1_1000 --dataset-type euroc --use-imu 1 \
       --cam-calib eval/config/raw_cam1_kb4_calib.json --config-path eval/config/oi_g40_det_gauge_gate_rest.json \
       --num-threads 4 --realtime
   ```

Pi 4 모니터에 GUI 띄우기 [미검증]:

1. Pi 4의 모니터에서 `uvify`로 데스크톱에 로그인합니다. 2026-11-06에는 로그인 화면 상태입니다.
2. Pi 4의 터미널에서 실행합니다. SSH에서 실행하려면 앞에 `DISPLAY=:0`을 붙입니다. 이때 화면 세션의 X 권한이 필요합니다.
3. 창을 닫거나 Ctrl-C를 누르면 실행이 끝나고 결과 파일을 씁니다.

- 단안 VIO는 메트릭 결과입니다. 그래서 GT 정렬은 SE3입니다. 스케일 오차가 화면에 그대로 보입니다. 단안 VO(IMU 없음)는 Sim3로 정렬합니다.
- Unreal EuRoC 폴더에는 GT가 없습니다. 그래서 화면에 GT 궤적이 없습니다. 평가는 4.4절로 합니다.
- Pi 4에서 GUI가 실시간 처리(15 fps)에 주는 부하는 측정하지 않았습니다 [미검증]. 판정용 실행은 `--show-gui 0`으로 합니다. GUI가 있을 때 `dropped`(데이터셋) 또는 `queue_drops`(live)가 0보다 크면 GUI 부하를 의심합니다.

## 8. 문제 해결

| 증상 | 확인할 것 |
|---|---|
| `No rp1-cfe or unicam media graph contains sensor` | 카메라 케이블, `dtoverlay=arducam-pivariety` |
| `Unexpected MPU WHO_AM_I value`, `Cannot open /dev/i2c-N` | `--i2c` 번호(5.1절), 사용자의 `i2c` 그룹 |
| `imu_i2c_errors`가 0보다 큼 | Pi 4 리그의 CP2112는 약 150~200회에 1회 전송 시간 초과(`errno 110`, 약 54 ms 뒤)가 납니다. CP2112가 상태 질문에 응답하지 않는 것이 원인입니다. 앱은 FIFO를 유지하고 샘플 경계만 다시 맞춥니다. 그래서 손실은 약 0.3%입니다. 자세한 내용은 `doc/mpu6500-cp2112-issue.md`에 있습니다. |
| 초기화되지 않음 | 2.5절의 조건을 확인합니다. 매초 상태 줄이 `tracking=initializing`을 계속 보이면, 리그를 2초 이상 정지한 다음 1초 안에 분명하게 들어 올리거나 옮깁니다. |
| `queue_drops` 또는 `dropped_input` 증가 | 추정기가 카메라보다 느립니다. 다른 프로세스가 CPU를 쓰는지 확인합니다. GUI가 켜져 있으면 `--show-gui 0`으로 다시 확인합니다. live는 `--fps`를 낮추거나 `--num-threads`를 확인합니다. |
| 밝기가 목표와 다름 | `camera.metering_circle`이 렌즈 원 안에 있는지 확인합니다. 노출은 `--max-exposure`(5 ms) 다음에 gain을 씁니다. |
| `Calibration resolution ... is neither the sensor mode nor the output` | 보정 해상도는 센서 모드 또는 그 1/N이어야 합니다. 변환의 `--downscale`과 live의 `--downscale`을 맞춥니다. |
| `ModuleNotFoundError: No module named 'numpy'` (Pi 4) | Pi 4에는 numpy가 없습니다. 평가와 변환은 데스크톱에서 합니다. |
| `No DISPLAY: running without the GUI` | 화면이 없는 실행입니다(SSH). 결과 파일은 정상으로 씁니다. GUI가 필요하면 7.3절을 따릅니다. |
| `Give --live, or --dataset-path and --dataset-type` | `granite_vio`에는 `--live` 또는 데이터셋 두 옵션 중 하나가 필요합니다. |
| `Dataset type bag is not supported` (Pi 4) | Pi 4 빌드에는 rosbag이 없습니다. EuRoC 폴더를 씁니다. |

## 9. 에이전트용 요약

- 실행 프로그램: `granite_vio`. 데이터셋은 `--dataset-path --dataset-type euroc --use-imu 1 [--realtime] --out`. live는 `--live --use-imu 1 --cam-calib --config-path --i2c --out [--record]`.
- 코드: `src/vio_app.h`(GUI, 실시간 공급, live 모드, `--out`, SUMMARY와 LIVE 줄), `src/vio.cpp`(`granite_vio` 옵션), `src/pi_camera_imu_vio.cpp`(live 앱 옵션, VIO 없는 확인 모드), `include/granite/device/pi_live_support.h`(녹화, `camera.*` 키, 센서 옵션), `src/device/pi_camera_imu.cpp`(카메라와 IMU, IMU 오류 복구), `eval/src/mono_vio_dataset.cpp`(평가 러너), `eval/scripts/capture_to_granite.py`, `eval/scripts/capture_to_euroc.py`, `eval/src/imu_fifo_test.cc`(CP2112 시험).
- 설정: 데이터셋 실행은 `eval/config/oi_g40_det_gauge_gate_rest.json` + `eval/config/raw_cam1_kb4_calib.json`. Pi 4 live는 `eval/config/live/b0429_pi_640x400_calib.json` + `b0429_pi_640x400_handheld.json`.
- 빌드: 데스크톱 `build_gui/`. Pi 4 크로스 빌드 `~/cross/granite-pi4/build_live`(Pangolin `~/cross/pangolin-pi4`).
- Pi 4: 바이너리 `~/granite_eval/bin/granite/`, 데이터 `~/granite_eval/data/city_sky_cam1/`, 결과 `~/granite_eval/results/`, live 출력 `~/granite_eval/live/`.
- 아직 없는 것: Pi 4의 `granite_vio` 실행 확인(데이터셋, live, GUI 부하), 움직이는 live 시험, 30 fps 실시간 시험, live 시간 오프셋 측정, IMU Allan 보정, CP2112 응답 손실의 위치(`doc/mpu6500-cp2112-issue.md` 5절).
