# Raspberry Pi 5 mono VIO 설치 매뉴얼

이 문서는 Raspberry Pi 5에서 Kurokesu AR0234 카메라와 FC IMU로 mono VIO를 실행하기 위한 준비 절차를 설명한다. 2단계(카메라 드라이버), 3단계(mavlink-router와 FC UART), 9단계(부팅 때 VIO 자동 실행)는 이 폴더의 스크립트로 실행한다.

이 폴더에는 2단계, 3단계, 9단계의 설치에 필요한 파일이 있다. 9단계의 granite는 Pi 5용으로 빌드한 실행 파일, live 설정, 실행 스크립트를 `resources/vio/granite/`에 함께 갖고 있다. 그래서 PC에서 granite를 따로 빌드하거나 배포하지 않아도 된다. 이 폴더를 Pi에 복사한 뒤 폴더 안에서 작업한다. 외부에서 받는 것은 Ubuntu apt 패키지(커널 헤더, `dkms`, `device-tree-compiler`, `v4l-utils`, granite의 실행 라이브러리)뿐이다.

## 빠른 시작

Pi의 OS는 Ubuntu 24.04이어야 한다. 아래 순서대로 실행한다. 각 단계의 설명은 4장부터 6장에 있다.

1. Pi를 끈다. 카메라를 CAM/DISP 1 포트에 연결한다.
2. FC와 Pi를 UART로 배선한다. 배선은 [docs/fc-v6x-rpi5-uart-wiring.md](docs/fc-v6x-rpi5-uart-wiring.md)의 4장을 따른다.
3. Pi를 켠다. Pi를 인터넷에 연결한다.
4. PC에서 이 폴더를 Pi로 복사한다. `uvify@192.168.168.101`은 Pi의 `user@ip`로 바꾼다.

   ```bash
   rsync -a ~/workspace/ddp-drone/pi5-setup/ uvify@192.168.168.101:~/pi5-setup/
   ```

5. Pi의 터미널에서 이 폴더로 이동한다.

   ```bash
   cd ~/pi5-setup
   ```

6. 카메라 드라이버를 설치한다.

   ```bash
   sudo bash install_ar0234_driver.sh
   ```

7. FC를 USB로 Pi에 연결한다.
8. FC 파라미터를 쓰고 FC를 재부팅한다. 확인 질문에 `y`를 입력한다.

   ```bash
   sudo python3 fc_mav.py --conn /dev/ttyACM0 apply --reboot
   ```

9. Pi의 UART를 켜고 mavlink-router를 설치한다.

   ```bash
   sudo bash setup_fc_uart_router.sh
   ```

10. Pi를 재부팅한다.

    ```bash
    sudo reboot
    ```

11. 카메라 드라이버를 확인한다. 마지막 줄은 `RESULT OK`이어야 한다.

    ```bash
    cd ~/pi5-setup
    bash install_ar0234_driver.sh --check
    ```

12. UART와 mavlink-router를 확인한다. 마지막 줄은 `RESULT OK`이어야 한다.

    ```bash
    bash setup_fc_uart_router.sh --check
    ```

13. FC IMU의 수신 비율을 확인한다. 결과는 약 200 Hz이어야 한다.

    ```bash
    python3 fc_mav.py --conn udpin:127.0.0.1:14601 imu
    ```

RTS/CTS 선(FC 4번, 5번 핀)을 연결하지 않았으면 8단계와 9단계를 아래 명령으로 바꾼다.

```bash
sudo python3 fc_mav.py --conn /dev/ttyACM0 --no-flow-control apply --reboot
sudo bash setup_fc_uart_router.sh --no-flow-control
```

`RESULT FAIL`이 나오면 출력의 `FAIL` 줄을 확인한다. 각 항목의 설명은 5장과 6장에 있다.

드론에서 부팅 때 VIO를 자동으로 실행하려면 이어서 아래 명령을 실행한다. 설명은 7장에 있다.

```bash
sudo bash setup_vio_service.sh      # granite 설치, 서비스 등록 (지금 시작하지는 않는다)
bash setup_vio_service.sh --check   # 마지막 줄은 RESULT OK
```

## 1. 대상 장비

| 항목 | 내용 |
|---|---|
| 컴패니언 | Raspberry Pi 5 |
| OS | **Ubuntu 24.04 LTS (arm64)**. 시험 커널 `6.8.0-1065-raspi`. Raspberry Pi OS가 아니다 |
| 카메라 | Kurokesu AR0234 (2 lane), **CAM/DISP 1** 포트 |
| FC | UVify V6X, ArduPilot 4.7.2 |
| FC 연결 | FC UART2(ArduPilot `SERIAL3`) ↔ Pi UART0(`/dev/ttyAMA0`), 921600 baud, RTS/CTS |

> **주의:** 이 문서와 스크립트는 Ubuntu 24.04 기준이다. Raspberry Pi OS에는 쓰지 않는다. Ubuntu의 부트 파일은 `/boot/firmware/`에 있다.

## 2. 폴더 구조

```text
pi5-setup/
├── README.md                    이 문서
├── install_ar0234_driver.sh     2단계: 카메라 드라이버 설치
├── setup_fc_uart_router.sh      3단계: Pi UART, mavlink-router 설치
├── fc_mav.py                    3단계: FC 파라미터 설정, IMU 수신 확인
├── setup_vio_service.sh         9단계: 부팅 때 granite VIO를 실행하는 systemd 서비스 설치
├── renew-config.sh              9단계: 새 xeye 캘리브레이션으로 서비스의 캘리브와 config 갱신
├── lib/bootcfg.sh               config.txt, cmdline.txt 편집 함수
├── docs/
│   └── fc-v6x-rpi5-uart-wiring.md   1단계: 배선
└── resources/
    ├── SOURCES.md               각 파일의 출처, 버전, SHA256
    ├── ar0234-rpi-driver/       드라이버 소스(브랜치 rpi5-ubuntu2404)
    ├── mavlink-router/          install.sh, 서비스 파일, Pi용 main.conf, aarch64 바이너리
    ├── vio/                     granite-vio.service(틀), granite-vio.default(실행 옵션), capture_to_granite.py(캘리브 변환)
    │   └── granite/             granite Pi 5 빌드(bin/, lib/), live 설정(config/), 실행 스크립트(scripts/), SHA256SUMS
    └── python/                  pymavlink wheel(CPython 3.12, aarch64)
```

`resources/vio/granite/bin/`의 실행 파일 세 개는 모두 `lib/libgranite.so`를 쓴다.

| 파일 | 소스 (granite 레포) | 하는 일 | 쓰는 곳 |
|---|---|---|---|
| `granite_vio` | `src/vio.cpp`, `src/vio_app.h` | `--live`: Kurokesu 카메라와 FC IMU로 실시간 VIO를 실행한다. `ODOMETRY` 송신, 지상 시각화 송신, EuRoC 녹화, Pangolin GUI를 지원한다. `--live`가 없으면 데이터셋을 재생한다 | 9단계의 서비스, 바탕화면의 granite 아이콘(둘 다 `pi5_live_kurokesu_vio.sh`로 실행) |
| `granite_pi_cam_imu_vio` | `src/pi_camera_imu_vio.cpp` | Arducam B0429 카메라와 I2C MPU-6500/9250 IMU로 실시간 VIO를 실행한다. `--capture-only`(추정 없이 이미지만 표시)와 `--record`(EuRoC 녹화)가 있다 | 이전 B0429 리그. Kurokesu와 FC IMU 리그에서는 쓰지 않는다 |
| `granite_mono_vio_dataset` | `eval/src/mono_vio_dataset.cpp` | GUI 없이 EuRoC 폴더를 재생하는 평가용 러너다. TUM 궤적과 상태 CSV를 쓴다. `--realtime`은 실시간 처리 조건을 흉내 낸다 | 녹화를 Pi에서 다시 실행하는 평가 |

드론 운용에는 `granite_vio`만 필요하다. `setup_vio_service.sh --check`도 `bin/`에서는 `granite_vio`만 검사한다.

## 3. 전체 순서와 레포·브랜치

| 단계 | 작업 | 레포 | 브랜치 (커밋) | 쓰는 파일 |
|---|---|---|---|---|
| 1 | 배선 | - | - | [docs/fc-v6x-rpi5-uart-wiring.md](docs/fc-v6x-rpi5-uart-wiring.md) |
| 2 | 카메라 드라이버 설치 | ar0234-rpi-driver | `rpi5-ubuntu2404` (`f2d4538`) | `install_ar0234_driver.sh`, `resources/ar0234-rpi-driver/` |
| 3 | mavlink-router 설치, FC UART 설정 | device-setup | `master` (`3447c02`) | `setup_fc_uart_router.sh`, `fc_mav.py`, `resources/mavlink-router/` |
| 4 | xeye 캡처 | multipleye | `cross-pi5` (`fdaf25cbc`) | `xeye/configs/capture_kurokesu_ar0234_pi.yaml` |
| 5 | xeye 캘리브레이션 | multipleye | `cross-pi5` (`fdaf25cbc`) | `calibrator`, `refine-calib` |
| 6 | granite 배포 | granite | `mavlink` (`341a11e`) 또는 `vio-good-io` (`568c7dc`) | `cross/pi5/build.sh`, `cross/pi5/deploy.sh`. 9단계를 하면 이 단계는 필요 없다 |
| 6 | openvio 배포 | openvio | `mavlink` (`8f08196`) 또는 `feature/xeye-capture-io` (`8867887`) | `cross/pi5/build.sh`, `cross/pi5/deploy.sh` |
| 7 | 캡처를 config로 변환 | granite | 6단계와 같다 | `eval/scripts/capture_to_granite.py`, `doc/xeye_to_granite_guide.md` |
| 7 | 캡처를 config로 변환 | openvio | 6단계와 같다 | `scripts/python/capture_to_config.py` |
| 8 | 실행 | granite, openvio | 6단계와 같다 | `cross/pi5/pi5_live_kurokesu_vio.sh` |
| 9 | granite 설치와 부팅 때 VIO 자동 실행 (드론) | granite | `mavlink` (`341a11e`) | `setup_vio_service.sh`, `renew-config.sh`, `resources/vio/` (7장) |

2단계, 3단계, 9단계의 파일은 `resources/`에 있다. 출처와 버전은 [resources/SOURCES.md](resources/SOURCES.md)에 있다. 4단계부터 8단계까지는 이 폴더에 파일이 없다. 각 레포의 해당 브랜치를 쓴다. 단, 9단계의 granite 빌드는 6단계의 결과와 같다.

브랜치 선택:

- `mavlink` 브랜치는 `vio-good-io`(granite)와 `feature/xeye-capture-io`(openvio)를 모두 포함한다. 추가로 VIO pose를 `ODOMETRY`로 FC에 보낸다. 이 기능은 실행 옵션(`--imu_send_odometry 1`)으로 켠다. 9단계의 서비스는 이 옵션을 켠다.
- granite `mavlink`는 드론 시나리오의 초기화 파라미터(모터 진동에 강한 정지 시작)와 health 검사(bias 규칙)도 포함한다. 드론에는 granite `mavlink`를 쓴다.
- VIO pose를 FC로 보내지 않으면 `vio-good-io`와 `feature/xeye-capture-io`로 충분하다.

원격 상태(2026-11-11 확인):

| 레포 | 브랜치 | 원격 |
|---|---|---|
| ar0234-rpi-driver | `rpi5-ubuntu2404` | **원격에 없다.** origin은 Kurokesu 상류이다. 이 폴더의 `resources/ar0234-rpi-driver/`가 사본이다 |
| device-setup | `master` | `uvify/device-setup`에 있다 |
| multipleye | `cross-pi5` | `uvify/multipleye`에 있다. 단, `xeye/build_pi.sh`는 레포에 없다(추적되지 않는 로컬 파일) |
| granite | `mavlink`, `vio-good-io` | `uvify/granite`에 있다 |
| openvio | `feature/xeye-capture-io`, `mavlink` | `uvify/openvio`에 있다 |

## 4. 준비: Pi로 폴더 복사

이 폴더를 Pi의 홈 폴더에 복사한다. 아래 예는 PC에서 실행한다. `PI`는 Pi의 `user@ip`이다.

```bash
PI=uvify@192.168.168.101
rsync -a ~/workspace/ddp-drone/pi5-setup/ $PI:~/pi5-setup/
```

USB 메모리나 zip으로 복사해도 된다. 이때 실행 권한이 빠질 수 있다. 그래서 아래 절차는 셸 스크립트를 `sudo bash`로, Python 스크립트를 `python3`로 실행한다.

## 5. 2단계: 카메라 드라이버 설치

`install_ar0234_driver.sh`는 다음 일을 한다.

1. 실행 중인 커널의 헤더, `dkms`, `device-tree-compiler`, `v4l-utils`를 apt로 설치한다. 인터넷 연결이 필요하다.
2. `resources/ar0234-rpi-driver/setup.sh`로 모듈(DKMS)과 overlay를 설치한다. overlay는 `/boot/firmware/overlays/ar0234.dtbo`에 들어간다.
3. `config.txt`에 `camera_auto_detect=0`과 `dtoverlay=ar0234`를 넣는다.
4. 같은 포트의 다른 카메라 overlay(예: `dtoverlay=arducam-pivariety`)에 `#`을 붙인다.

스크립트는 `apt full-upgrade`를 실행하지 않는다. Kurokesu apt 저장소도 추가하지 않는다. 두 가지는 Raspberry Pi OS용이다.

스크립트는 드라이버 소스에 `ar0234_s_stream()`이 없으면 멈춘다. 이 함수가 없으면 커널 6.8에서 `STREAMON`은 성공하지만 프레임이 오지 않는다.

> **경고:** 스크립트는 `/boot/firmware/config.txt`를 바꾼다. 처음 바꿀 때 원본을 `config.txt.bak-ar0234`로 저장한다. 다음 실행부터는 `config.txt.bak-ar0234.<시각>`으로 저장한다.

절차(Pi에서 실행):

1. Pi를 끈 상태에서 카메라를 CAM/DISP 1 포트에 연결한다.
2. 스크립트를 실행한다.

   ```bash
   cd ~/pi5-setup
   sudo bash install_ar0234_driver.sh
   ```

   - 카메라가 CAM/DISP 0 포트에 있으면 `--port cam0`을 추가한다.
   - 예제 뷰어(`resources/ar0234-rpi-driver/examples/ar0234_view.py`)의 패키지가 필요하면 `--with-viewer`를 추가한다.
   - 다른 드라이버 소스를 쓰려면 소스 폴더를 마지막 인자로 준다.
3. 출력에 `INSTALL /boot/firmware/overlays/ar0234.dtbo` 줄이 있는지 확인한다.
4. Pi를 재부팅한다.

   ```bash
   sudo reboot
   ```

5. 설치를 확인한다. 마지막 줄은 `RESULT OK`이어야 한다.

   ```bash
   bash ~/pi5-setup/install_ar0234_driver.sh --check
   ```

   `--check`는 모듈, overlay, `config.txt`, media graph를 확인한다. 카메라 stream은 시작하지 않는다. 시험 장비에서 `rp1-cfe-csi2_ch0`은 `/dev/video0`이었다. 이 값은 xeye config의 `cam0.port`와 같아야 한다.

새 커널을 설치하면 DKMS가 모듈을 다시 빌드한다. 새 커널로 재부팅하기 전에 그 커널의 헤더를 설치한다.

## 6. 3단계: mavlink-router 설치와 FC UART 설정

이 단계는 세 부분이다. 순서대로 한다.

1. FC 파라미터를 바꾼다(USB 연결, `fc_mav.py`).
2. Pi의 UART와 mavlink-router를 설정한다(`setup_fc_uart_router.sh`).
3. 재부팅 뒤 IMU 수신을 확인한다(`--check`, `fc_mav.py imu`).

이 단계는 인터넷 연결이 필요 없다.

### 6.1 pymavlink

`fc_mav.py`는 pymavlink를 쓴다. 설치는 필요 없다. 시스템에 pymavlink가 없으면 `fc_mav.py`가 `resources/python/`의 wheel을 `~/.cache/pi5-setup/python/`에 풀고 그 위치에서 import한다. wheel은 CPython 3.12, aarch64용이다. Ubuntu 24.04의 `python3`는 3.12이다.

### 6.2 FC 파라미터

V6X의 `SERIAL3`(UART2 커넥터)은 RC 입력으로 설정되어 있을 수 있다. 아래의 "변경 전 값"은 2026-11-11에 .60의 FC에서 읽은 값이다. 다른 FC는 값이 다를 수 있다.

| 파라미터 | 변경 전 값 (.60) | 목표 값 |
|---|---|---|
| `SERIAL3_PROTOCOL` | 23 (RCIN) | 2 (MAVLink2) |
| `SERIAL3_BAUD` | 115 | 921 |
| `SERIAL3_OPTIONS` | 4 (half-duplex) | 0 |
| `BRD_SER3_RTSCTS` | 2 (auto) | 1 (RTS/CTS 선이 없으면 0) |

> **경고:** 아래 절차는 FC의 파라미터를 바꾸고 FC를 재부팅한다. 기체가 비행 중이 아니고 모터가 꺼진 상태에서만 실행한다.

절차:

1. FC를 USB로 Pi에 연결한다.
2. mavlink-router가 `/dev/ttyACM0`을 쓰고 있으면 router를 멈춘다.

   ```bash
   sudo systemctl stop mavlink-router
   ```

3. 현재 값을 읽는다. 바꿀 값에는 `CHANGE`가 표시된다.

   ```bash
   python3 ~/pi5-setup/fc_mav.py --conn /dev/ttyACM0 params
   ```

   사용자가 `dialout` 그룹에 없으면 `/dev/ttyACM0`을 열 수 없다. 이때는 명령 앞에 `sudo`를 붙인다.
4. 값을 쓰고 FC를 재부팅한다. 스크립트는 쓰기 전에 확인을 묻는다.

   ```bash
   python3 ~/pi5-setup/fc_mav.py --conn /dev/ttyACM0 apply --reboot
   ```

   RTS/CTS 선을 연결하지 않았으면 `--no-flow-control`을 `apply` 앞에 추가한다.
5. FC가 재부팅된 뒤 3번 명령을 다시 실행한다. 모든 줄이 `OK`이어야 한다.

FC 파라미터는 QGroundControl이나 Mission Planner로 바꿔도 된다. 목표 값은 위 표와 같다.

### 6.3 Pi UART와 mavlink-router

`setup_fc_uart_router.sh`는 다음 일을 한다.

1. `config.txt`에 `dtoverlay=uart0-pi5,ctsrts`를 넣는다. `--no-flow-control`을 주면 `dtparam=uart0=on`을 넣는다.
2. `cmdline.txt`에서 `ttyAMA0`의 kernel console을 지운다. `serial-getty@ttyAMA0`을 끄고 mask한다.
3. `resources/mavlink-router/main.conf`를 `/etc/mavlink-router/main.conf`에 쓴다. 기존 파일은 `main.conf.bak-<시각>`으로 저장한다.
4. `resources/mavlink-router/mavlink-routerd-v4-aarch64`의 SHA256을 확인한다.
5. `resources/mavlink-router/install.sh`로 바이너리를 `/usr/local/bin/mavlink-routerd`에, 서비스를 `/etc/systemd/system/mavlink-router.service`에 설치한다. 서비스를 켜고 시작한다.

Pi 5에서 `serial0`은 기본으로 디버그 커넥터(`ttyAMA10`)이다(Raspberry Pi 문서 기준). 그래서 스크립트는 `console=serial0`을 지우지 않는다. 단, `/dev/serial0`이 `ttyAMA0`을 가리키면 지운다.

`main.conf`의 endpoint는 다음과 같다.

| endpoint | 주소 | 사용자 |
|---|---|---|
| `fc` | `/dev/ttyAMA0`, 921600, `FlowControl = true` | FC |
| `xeye_bridge` | UDP 127.0.0.1:14600 | xeye bridge |
| `xeye_imu` | UDP 127.0.0.1:14601 | xeye capture(`imu_mavlink`), granite·openvio live |
| `gcs` | UDP 0.0.0.0:14550 (server) | 네트워크의 GCS |

capture, granite, openvio는 모두 14601 포트를 쓴다. 그래서 세 프로그램은 동시에 실행할 수 없다. 9단계의 VIO 서비스(granite)도 같다.

> **경고:** `gcs` endpoint에 접근하는 모든 장비는 FC에 명령을 보낼 수 있다. 신뢰할 수 없는 네트워크에서는 `resources/mavlink-router/main.conf`에서 `gcs` 절을 지운 뒤 스크립트를 실행한다.

> **경고:** 스크립트는 `/boot/firmware/config.txt`와 `cmdline.txt`를 바꾼다. 처음 바꿀 때 원본을 `config.txt.bak-uart0`, `cmdline.txt.bak-uart0`으로 저장한다.

절차(Pi에서 실행):

1. 스크립트를 실행한다.

   ```bash
   cd ~/pi5-setup
   sudo bash setup_fc_uart_router.sh
   ```

   - RTS/CTS 선이 없으면 `--no-flow-control`을 추가한다.
   - 다른 `mavlink-routerd` 바이너리를 쓰려면 `--router-binary <파일>`을 추가한다.
   - 기존 `/etc/mavlink-router/main.conf`를 그대로 두려면 `--keep-conf`를 추가한다.
2. 스크립트가 재부팅을 요청하면 Pi를 재부팅한다.

   ```bash
   sudo reboot
   ```

3. Pi 쪽 설정을 확인한다. 마지막 줄은 `RESULT OK`이어야 한다.

   ```bash
   bash ~/pi5-setup/setup_fc_uart_router.sh --check
   ```

   router 로그에는 `Opened UART` 줄과 `/dev/ttyAMA0`이 보여야 한다.

### 6.4 IMU 수신 확인

`fc_mav.py imu`는 router의 14601 endpoint에서 `HIGHRES_IMU`를 받는다. FC에 200 Hz를 요청하고 10초 동안 수신 비율을 잰다.

1. capture와 VIO를 멈춘다. 14601 포트를 한 프로그램만 쓸 수 있다.
2. 수신 비율을 잰다.

   ```bash
   python3 ~/pi5-setup/fc_mav.py --conn udpin:127.0.0.1:14601 imu
   ```

3. 결과를 확인한다. `COMMAND_ACK`는 0이어야 한다. 수신 비율은 약 200 Hz이어야 한다.

2026-11-11의 실기 결과(capture 10초, router 경유)는 평균 200.0 Hz, 최대 간격 8.0 ms였다.

## 7. 9단계: 부팅 때 VIO 자동 실행 (systemd)

드론은 전원을 켜면 VIO가 자동으로 시작되어야 한다. `setup_vio_service.sh`는 다음 일을 한다.

1. granite의 실행 라이브러리(OpenCV, TBB, GLEW, GL, OpenH264 등 Ubuntu 패키지 13개) 중 빠진 것을 apt로 설치한다. 인터넷 연결이 필요하다. `apt upgrade`는 하지 않는다.
2. `resources/vio/granite/`의 granite를 `~/deploy/granite`에 설치한다. 먼저 각 파일의 SHA256을 확인한다. `runs/`(이전 실행 결과)는 그대로 둔다. 바탕화면이 있으면 granite 아이콘도 설치한다.
   - `~/deploy/granite/config/service/`가 없으면 번들의 672x600 캘리브와 config를 이 폴더에 복사한다. 서비스는 이 폴더의 파일을 쓴다. 폴더가 이미 있으면 그대로 둔다(7.4절).
3. `granite-vio.service`와 `/etc/default/granite-vio`를 설치하고, 부팅 때 시작하도록 켠다.

### 7.1 서비스의 동작

| 항목 | 내용 |
|---|---|
| 실행 | `~/deploy/granite/scripts/pi5_live_kurokesu_vio.sh --no-gui` + `/etc/default/granite-vio`의 `GRANITE_VIO_ARGS` |
| 기본 옵션 | `--no-record --imu_send_odometry 1`: 녹화하지 않고, VIO 상태를 FC에 `ODOMETRY`로 보낸다 |
| 캘리브와 config | `~/deploy/granite/config/service/kurokesu_672x600_calib.json`, `kurokesu_672x600_handheld.json` (unit의 `GRANITE_CALIB`, `GRANITE_CONFIG`). 이미지는 672x600이다: 센서가 2x2 binning(960x600)을 하고, granite가 crop한다 |
| 시작 순서 | `mavlink-router.service` 다음에 시작한다. `/dev/video0`이 생길 때까지 최대 30 s 기다린다. FC의 `HIGHRES_IMU`가 올 때까지 최대 60 s 기다린다(FC가 Pi와 같이 부팅하는 경우). 기다리는 동안 granite는 FC에 송신을 다시 요청한다 |
| 실행 사용자 | granite를 배포한 사용자(`--user`). `video` 그룹을 추가로 받는다 |
| 재시작 | 하지 않는다(`Restart=no`). 부팅 한 번에 VIO 한 번이다. 다시 시작하면 원점과 yaw 0이 새로 정해지기 때문이다 |
| 정지 | `systemctl stop`은 SIGINT를 보낸다. `granite_vio`는 결과를 쓰고 끝난다 |
| 결과 | `~/deploy/granite/runs/<날짜_시각>/`: `run_traj_tum.txt`, `run_states.csv`(`healthy`, `prev_corr_rot` 포함), `run.log` |
| 로그 | `journalctl -u granite-vio` |
| 지상 시각화 | GUI 요소(pose, 궤적용 상태, 이미지, keypoint, landmark)를 UDP로 `192.168.168.102:5100`에 보낸다. 이미지는 H.264다(frame마다, 움직임이 크면 화질이 낮아짐). 평균 전송률은 1.7 Mbit/s 이하다(`vis_msg_max_kbps`). 받는 PC에서 `resources/vio/UVIFY_VIO_Viewer-x86_64.AppImage`를 실행한다 |

- 시각화 메시지의 주소를 바꾸려면 `/etc/default/granite-vio`의 `GRANITE_VIO_ARGS`에 `--vis_msg_ip <IP> --vis_msg_port <port>`를 추가한다. 끄려면 `--send_vis_msg 0`을 추가한다. 형식은 granite `doc/granite_vis.proto`와 `doc/pi5_kurokesu_live_input.md` 10절에 있다.
- 이미지를 JPEG(q80)로 보내려면 `GRANITE_VIO_ARGS`에 `--vis_msg_encode mjpeg`를 추가한다. 화질은 일정하고, 1.7 Mbit/s 안에서 이미지 수가 줄어든다. H.264의 전송률 상한 없이 화질을 고정하려면 `--vis_msg_h264_rate_control quality`를 추가한다(빠른 장면에서 약 4.7 Mbit/s). H.264는 Pi CPU로 인코딩한다(OpenH264). 시각화 thread는 가장 낮은 우선순위(`SCHED_IDLE`)라서 VIO가 먼저 CPU를 쓴다. Pi에서 CPU 사용량은 측정하지 않았다.
- 뷰어 AppImage는 H.264 디코더(OpenH264)를 갖고 있다. 받는 PC에 패키지를 더 설치하지 않아도 된다(Ubuntu 24.04 이상).
- 서비스는 카메라와 FC IMU 포트(UDP 14601)를 쓴다. 그래서 서비스가 실행 중이면 xeye capture, 캘리브레이션 캡처, 바탕화면의 granite 아이콘을 실행할 수 없다.
- VIO는 정지 상태 다음의 움직임에서 초기화한다. 드론을 바닥에 둔 채 전원을 켜고, 그다음 들어 올리거나 이륙하면 초기화한다.
- FC가 `ODOMETRY`를 쓰려면 FC 쪽 설정(`VISO_TYPE` 등)이 필요하다. 이 설정은 FC 담당자가 한다. 설정 전에는 FC가 메시지를 무시한다.

### 7.2 설치

> **경고:** 2번 단계는 `~/deploy/granite`의 `bin/`, `lib/`, `config/`, `scripts/`를 이 폴더의 파일로 바꾼다. Pi에서 직접 고친 live 설정이 있으면 먼저 복사해 둔다. 이전 실행 결과(`runs/`)와 서비스용 `config/service/`는 바뀌지 않는다.

1. Pi에서 스크립트를 실행한다. 부팅 때 시작하도록 켠다. 지금 시작하지는 않는다.

   ```bash
   cd ~/pi5-setup
   sudo bash setup_vio_service.sh
   ```

   - granite를 실행할 사용자가 sudo를 실행한 사용자와 다르면 `--user <사용자>`를 추가한다.
   - 설치 폴더가 `~/deploy/granite`가 아니면 `--deploy <폴더>`를 추가한다.
   - PC에서 `cross/pi5/deploy.sh`로 이미 배포한 granite를 쓰려면 `--use-deployed`를 추가한다. 이때 2번 단계를 건너뛴다.
   - 인터넷이 없고 패키지가 이미 있으면 `--skip-apt`를 추가한다.
   - 부팅 때 시작하지 않으려면 `--no-enable`을 추가한다.
2. 설치를 확인한다. 마지막 줄은 `RESULT OK`이어야 한다. 실행 라이브러리가 빠졌으면 `FAIL` 줄이 나온다.

   ```bash
   bash ~/pi5-setup/setup_vio_service.sh --check
   ```

3. 시험으로 시작한다. 카메라를 쓰는 다른 프로그램이 없어야 한다.

   ```bash
   sudo systemctl start granite-vio
   journalctl -u granite-vio -f
   ```

4. 로그에서 `TBB threads`와 `Output:` 줄을 확인한다. 드론을 2 s 이상 정지한 뒤 들어 올리면 상태 출력이 시작된다.

5. 새 번들을 설치했으면 7.4절의 `renew-config.sh`를 다시 실행한다. 그래야 서비스가 새 번들의 config를 쓴다.

### 7.3 일상 운용

| 작업 | 명령 |
|---|---|
| 상태 확인 | `systemctl status granite-vio` |
| 정지 (capture, 캘리브레이션 전에) | `sudo systemctl stop granite-vio` |
| 시작 | `sudo systemctl start granite-vio` |
| 부팅 때 시작 끄기 | `sudo systemctl disable granite-vio` |
| 옵션 바꾸기 | `/etc/default/granite-vio`를 고친 뒤 `sudo systemctl restart granite-vio` |
| 새 캘리브레이션 적용 | `~/pi5-setup/renew-config.sh <capture.yaml> --restart` (7.4절) |

> **주의:** 비행 중에 `systemctl restart`나 `stop`을 하지 않는다. VIO가 멈추면 FC는 `ODOMETRY`를 받지 못한다. 다시 시작하면 새 원점에서 초기화하는데, 비행 중에는 정지 상태가 없어서 초기화하지 못한다.

### 7.4 캘리브레이션을 다시 한 뒤

`renew-config.sh`는 xeye의 새 캘리브레이션(`capture.yaml`)으로 서비스의 캘리브와 config를 만든다. `capture.yaml`의 카메라는 1920x1200이고, crop은 `[288, 0, 1344, 1200]`이어야 한다.

| 파일 | 내용 |
|---|---|
| `config/service/kurokesu_672x600_calib.json` | 새 캘리브레이션(intrinsics, 카메라-IMU 변환, 시간 차이, IMU bias). `capture_to_granite.py --downscale 2`로 만든다 |
| `config/service/kurokesu_672x600_handheld.json` | 번들의 config. 정지 판정의 픽셀 기준 2개만 새 캘리브의 image scale [px/rad]에 맞춰 바꾼다 |
| `config/service/SOURCE` | 사용한 `capture.yaml`의 경로, SHA256, 번들 커밋, 날짜 |
| `config/service/backup/<날짜_시각>/` | 바꾸기 전의 파일 |

1. 서비스를 멈춘다. 캘리브레이션 캡처는 카메라와 IMU를 쓴다.

   ```bash
   sudo systemctl stop granite-vio
   ```

2. xeye로 캡처하고 캘리브레이션한다.
3. Pi에서 스크립트를 실행한다. sudo 없이 실행한다.

   ```bash
   ~/pi5-setup/renew-config.sh ~/xeye-deploy/config/capture.yaml --restart
   ```

   - 출력의 fx, fy, cx, cy, `cam_time_offset`에서 이전 값과 새 값을 비교한다.
   - `--restart`를 빼면 서비스는 다음 시작(다음 부팅)부터 새 파일을 쓴다.
4. 이전 캘리브로 되돌리려면 `config/service/backup/<날짜_시각>/`의 파일을 `config/service/`에 복사한다. 그다음 서비스를 다시 시작한다.

- Pi에는 `python3-numpy`, `python3-scipy`, `python3-yaml`이 필요하다. .60에는 이 패키지가 있다.
- 크기나 crop이 다른 `capture.yaml`을 주면 스크립트는 파일을 바꾸지 않고 끝난다.

## 8. 되돌리기

### 8.1 카메라 드라이버

1. DKMS 모듈을 지운다.

   ```bash
   dkms status | grep ar0234 | cut -d, -f1 | sort -u | xargs -rI{} sudo dkms remove {} --all
   ```

2. overlay를 지운다.

   ```bash
   sudo rm /boot/firmware/overlays/ar0234.dtbo
   ```

3. `config.txt`를 복원한다.

   ```bash
   sudo cp /boot/firmware/config.txt.bak-ar0234 /boot/firmware/config.txt
   ```

4. Pi를 재부팅한다.

### 8.2 UART와 router

1. router를 멈추고 끈다.

   ```bash
   sudo systemctl disable --now mavlink-router
   ```

2. 부트 파일을 복원한다.

   ```bash
   sudo cp /boot/firmware/config.txt.bak-uart0 /boot/firmware/config.txt
   sudo cp /boot/firmware/cmdline.txt.bak-uart0 /boot/firmware/cmdline.txt
   ```

   `cmdline.txt.bak-uart0`은 스크립트가 `cmdline.txt`를 바꾼 경우에만 있다.
3. `serial-getty@ttyAMA0`의 mask를 푼다.

   ```bash
   sudo systemctl unmask serial-getty@ttyAMA0
   ```

4. Pi를 재부팅한다.

> **주의:** 2단계와 3단계를 모두 실행한 뒤 `config.txt.bak-ar0234`로 복원하면 UART 설정도 없어진다. `.bak-ar0234`는 3단계 전의 상태이다.

### 8.3 VIO 서비스

1. 서비스를 멈추고 지운다.

   ```bash
   sudo bash ~/pi5-setup/setup_vio_service.sh --uninstall
   ```

2. 옵션 파일이 필요 없으면 지운다.

   ```bash
   sudo rm /etc/default/granite-vio
   ```

## 9. 확인된 것과 확인되지 않은 것

확인된 것:

- 스크립트의 `config.txt`·`cmdline.txt` 편집은 Ubuntu 24.04 형식의 사본에서 시험했다. 두 번째 실행은 파일을 바꾸지 않았다.
- `fc_mav.py`의 `params`, `apply`, `imu`는 모의 FC(UDP)에서 시험했다. 번들 wheel을 푸는 경로는 x86_64 wheel로 같은 방식을 시험했다.
- `mavlink-routerd-v4-aarch64`는 device-setup `install.sh`가 NAS에서 받는 파일과 같다. .60에서 이 바이너리가 동작했다.
- 스크립트가 하는 각 단계는 .60(uvify-pi5-01)에서 손으로 한 절차와 같다(2026-11-09 - 11-11).
- `granite-vio.service`는 `systemd-analyze verify`를 통과했다(데스크톱, 임시 배포 폴더).
- .60에서 2026-11-12에 시험했다.
  - `setup_vio_service.sh --skip-apt`: 번들 설치, `config/service/` 생성, unit 갱신.
  - `renew-config.sh`: 번들 캘리브를 만든 `capture.yaml`로 실행하면, 만든 config가 번들 config와 같다. 크기나 crop이 다른 `capture.yaml`은 거부한다.
  - 서비스 시작: 672x600(`binning=1`)으로 동작한다.
  - IMU 대기: FC 데이터를 15 s 늦게 보내는 시험(UDP 중계)에서 granite가 기다린 뒤 정상으로 시작했다.
  - 672x600 정지 시험(45 s): 20 fps, `granite_vio` CPU 약 90-120 %, 노출 중앙부터 상태 출력까지 지연 min 21.2 ms, p50 27.4 ms.

확인되지 않은 것:

- 두 셸 스크립트를 실제 Pi에서 처음부터 끝까지 실행하지 않았다. 작성 시점에 .60에 SSH로 접속할 수 없었다.
- `fc_mav.py`를 실제 V6X에 실행하지 않았다. aarch64 wheel의 import도 Pi에서 시험하지 않았다.
- 커널 6.11 이상에서 드라이버 동작.
- xeye config(`capture_kurokesu_ar0234_pi.yaml`)의 주석은 FC를 PX4로, 연결을 USB로 설명한다. 실제 FC는 ArduPilot이고, 연결은 UART이다. 동작에는 영향이 없다.
- USB를 뺀 상태에서 FC를 별도 전원으로 구동하는 시험.
- UART 전환 뒤 카메라–IMU `time_offset`. USB 녹화로 얻은 값(13.7 ms)은 링크 지연이 바뀌어 다시 재야 한다.
- FC와 Pi에 전원을 같이 넣는 실제 부팅에서 60 s 대기가 충분한지 시험하지 않았다. 2026-11-11의 부팅에서는 5 s 대기가 모자라서 서비스가 실패했다.
