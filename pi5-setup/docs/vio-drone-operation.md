# VIO 드론 운용 매뉴얼

이 문서는 VIO를 장착한 드론을 현장에서 운용하는 방법을 설명합니다. 지상 노트북을 연결하는 방법, VIO를 시작하고 끄는 방법, 상태를 확인하는 방법을 다룹니다.

VIO는 드론의 Raspberry Pi 5(RPi)에서 실행됩니다. VIO는 카메라 영상과 FC의 IMU로 드론의 위치와 자세를 계산하고, 그 결과를 FC에 MAVLink `ODOMETRY`(20 Hz)로 보냅니다.

## 1. 장비와 주소

| 장비 | 주소 | 비고 |
|---|---|---|
| 지상 노트북 | `192.168.168.102` | MACA 데이터링크 지상국에 유선으로 연결합니다 |
| RPi (VIO) | `192.168.168.101` | 계정 `uvify`, 비밀번호 `uvify` |
| 지상 시각화 | 노트북 UDP `5100` 포트 | RPi가 노트북으로 보냅니다 |

- RPi의 주소는 고정입니다. RPi는 DHCP를 쓰지 않습니다.
- RPi의 Wi-Fi와 Bluetooth는 꺼져 있습니다. 데이터링크와의 전파 간섭을 막기 위해서입니다. 그래서 RPi는 인터넷에 연결되지 않습니다.

## 2. 지상 노트북 연결

1. 노트북을 MACA 데이터링크 지상국에 유선(LAN)으로 연결합니다.
2. 노트북의 유선 연결에 IPv4 주소를 수동으로 설정합니다.
   - 주소: `192.168.168.102`
   - 넷마스크: `255.255.255.0` (`/24`)
   - 게이트웨이: 비워 둡니다.

   Ubuntu에서는 "설정 > 네트워크 > 유선 > IPv4 > 수동"에서 설정합니다. 명령으로 설정하려면 아래 명령을 씁니다. `<인터페이스>`는 `ip link`로 확인한 유선 인터페이스 이름입니다.

   ```bash
   nmcli con add type ethernet ifname <인터페이스> con-name maca \
     ipv4.method manual ipv4.addresses 192.168.168.102/24 ipv6.method disabled
   nmcli con up maca
   ```

3. RPi에 연결되는지 확인합니다. 응답이 와야 합니다.

   ```bash
   ping 192.168.168.101
   ```

4. RPi에 SSH로 접속합니다. 비밀번호는 `uvify`입니다.

   ```bash
   ssh uvify@192.168.168.101
   ```

데이터링크는 대역폭이 낮고 연결이 잠시 끊길 수 있습니다. `ping`이 응답하지 않으면 10-20초 기다린 뒤 다시 시도합니다.

## 3. VIO 자동 시작

RPi가 부팅되면 VIO 서비스(`granite-vio`)가 자동으로 시작됩니다. 서비스는 아래 조건을 기다립니다.

| 조건 | 기다리는 시간 | 시간이 지나면 |
|---|---|---|
| 카메라(`/dev/video0`) | 최대 30초 | 서비스가 끝납니다 |
| FC의 IMU 데이터(`HIGHRES_IMU`) | 서비스 시작 뒤 최대 60초 | 서비스가 끝납니다 |

> **주의:** FC가 60초 안에 IMU 데이터를 보내야 합니다. FC 전원이 늦게 들어오거나 FC와 RPi 사이의 UART가 연결되지 않으면 VIO가 시작되지 않습니다. 이때는 FC가 켜진 것을 확인한 뒤 `~/start_vio.sh`로 VIO를 시작합니다(5장).

- 서비스는 한 번 끝나면 자동으로 다시 시작하지 않습니다. 다시 시작하면 VIO의 원점과 기준 방향이 새로 정해지기 때문입니다.
- VIO는 초기화한 뒤에만 FC로 `ODOMETRY`를 보냅니다.

## 4. 운용 순서

1. 드론을 평평한 바닥에 놓습니다.
2. 드론을 전원에 연결합니다. FC와 RPi가 부팅되고 VIO 서비스가 자동으로 시작됩니다.
3. 지상 노트북을 연결하고(2장) VIO 상태를 확인합니다(6장). 주기 상태 줄이 나오고 `tracking=initializing`이어야 합니다.
4. 드론을 2초 이상 움직이지 않습니다.
5. 드론을 들어 올리거나 시험 장소로 옮깁니다. 이 움직임으로 VIO가 초기화합니다. 상태 줄이 `tracking=tracking`, `metric_initialized=1`로 바뀌어야 합니다.
6. 모터를 arming하고 이륙합니다.

> **주의:** 비행 중에는 VIO를 끄거나 다시 시작하지 않습니다. VIO가 멈추면 FC가 위치를 받지 못합니다. 비행 중에는 정지 구간이 없어서 VIO가 다시 초기화할 수 없습니다.

## 5. 수동으로 끄고 시작하기

RPi에 SSH로 접속한 뒤 아래 스크립트를 실행합니다. 부팅 때 자동으로 시작하는 설정은 그대로 남습니다.

| 작업 | 명령 | 동작 |
|---|---|---|
| VIO 끄기 | `~/stop_vio.sh` | VIO를 멈춥니다. VIO는 결과 파일을 쓴 뒤 끝납니다(몇 초 걸립니다). 마지막 줄에 결과 폴더(`Results: ...`)가 나옵니다 |
| VIO 시작 | `~/start_vio.sh` | 꺼져 있는 VIO를 시작합니다. 이미 실행 중이면 아무것도 하지 않고 알립니다 |

- VIO를 다시 시작하려면 `~/stop_vio.sh`를 실행한 다음 `~/start_vio.sh`를 실행합니다. 시작하기 전에 드론을 바닥에 놓습니다(4장의 1번부터).
- 같은 일을 `systemctl`로도 할 수 있습니다: `sudo systemctl stop granite-vio`, `sudo systemctl start granite-vio`.
- 부팅 때 자동으로 시작하지 않게 하려면 `sudo systemctl disable granite-vio`를 실행합니다. 다시 켜려면 `sudo systemctl enable granite-vio`를 실행합니다.

## 6. 상태 확인

| 확인할 것 | 명령 |
|---|---|
| 서비스 상태 | `systemctl status granite-vio` |
| 실시간 로그 | `journalctl -u granite-vio -f` (Ctrl-C로 나옵니다. VIO는 계속 실행됩니다) |

로그의 주요 줄:

| 로그 줄 | 뜻 |
|---|---|
| `IMU: FC streams HIGHRES_IMU at 200 Hz` | FC가 IMU를 200 Hz로 보냅니다. 정상입니다 |
| `Waiting for the IMU data (N s)` | FC의 IMU 데이터를 기다리고 있습니다. 60초가 지나면 서비스가 끝납니다 |
| `granite_vio: The IMU did not deliver data in 60 s` | FC가 IMU를 보내지 않아서 VIO가 끝났습니다(9장 문제 해결) |
| `ODOMETRY: the VIO is unhealthy; stopped sending` | VIO가 비정상으로 판정되어 FC로의 송신을 멈췄습니다. 이 실행에서는 송신이 다시 시작되지 않습니다 |

1초마다 상태 줄이 나옵니다. 예:

```text
t=58.37 input_fps=19.78 state_fps=19.80 dropped=20 imu_hz=200.12 exposure_ms=3.01 gain_db=18.62 brightness=20.1 ... tracking=tracking support=inertial metric_initialized=1 p= 0.12 -0.30 0.05
```

| 항목 | 정상 값 | 뜻 |
|---|---|---|
| `input_fps` | 약 20 | 카메라 영상의 비율 [Hz] |
| `state_fps` | 약 20 | VIO 결과의 비율 [Hz]. FC로 가는 `ODOMETRY`의 비율과 같습니다 |
| `dropped` | 늘지 않음 | VIO가 처리하지 못하고 버린 영상의 수(누적) |
| `imu_hz` | 약 200 | FC IMU 데이터의 비율 [Hz] |
| `brightness` | 약 20 | 영상 밝기 [%]. 자동 노출의 목표는 20 %입니다 |
| `tracking` | `tracking` | `initializing`: 초기화 전, `tracking`: 정상, `lost`: 추적 실패 |
| `metric_initialized` | `1` | 초기화가 끝났는지 |
| `p` | 실제 이동과 비슷함 | VIO 위치 [m]. 초기화한 위치가 원점입니다 |

> **경고:** 드론이 멈춰 있는데 `p`가 계속 커지거나 `tracking=lost`가 계속되면 VIO가 발산한 것입니다. 이때 VIO가 FC로 송신을 멈추지 않을 수 있습니다. 비행 전이면 `~/stop_vio.sh`로 끄고, 드론을 바닥에 놓은 뒤 `~/start_vio.sh`로 다시 시작합니다.

## 7. 지상 시각화 뷰어

RPi는 VIO의 pose, 영상, 특징점을 지상 노트북(`192.168.168.102`)의 UDP `5100` 포트로 보냅니다. 평균 전송률은 1.7 Mbit/s 이하입니다.

1. `pi5-setup/resources/vio/UVIFY_VIO_Viewer-x86_64.AppImage`를 노트북에 복사합니다. 노트북은 Ubuntu 24.04 이상이어야 합니다.
2. 실행 권한을 주고 실행합니다. 뷰어는 모든 인터페이스의 `5100` 포트에서 받습니다.

   ```bash
   chmod +x UVIFY_VIO_Viewer-x86_64.AppImage
   ./UVIFY_VIO_Viewer-x86_64.AppImage
   ```

시각화 송신을 끄거나 켜려면 RPi의 `/etc/default/granite-vio`를 고칩니다(`sudo` 필요). 그다음 VIO를 다시 시작합니다(5장).

```bash
# 지금 설정
GRANITE_VIO_ARGS="--no-record --imu_send_odometry 1"
# 시각화 송신 끄기
GRANITE_VIO_ARGS="--no-record --imu_send_odometry 1 --send_vis_msg 0"
```

| 옵션 | 뜻 |
|---|---|
| `--no-record` | 영상을 녹화하지 않습니다. 이 옵션을 지우면 영상과 IMU를 모두 녹화합니다. 녹화는 저장 공간을 많이 씁니다 |
| `--imu_send_odometry 1` | VIO 결과를 FC로 보냅니다. 지우지 않습니다 |
| `--send_vis_msg 0` | 지상 시각화 송신을 끕니다. `1`이면 켭니다 |
| `--vis_msg_max_kbps N` | 시각화 평균 전송률의 상한 [kbit/s]. 지금 설정은 1700입니다 |

## 8. 실행 결과

VIO를 실행할 때마다 RPi의 `~/deploy/granite/runs/<날짜_시각>/`에 결과가 남습니다.

| 파일 | 내용 |
|---|---|
| `run_traj_tum.txt` | VIO 궤적(TUM 형식) |
| `run_states.csv` | 상태마다의 위치, 속도, bias, health |
| `run.log` | 실행 로그 |

## 9. 문제 해결

| 증상 | 확인할 것 |
|---|---|
| `ping 192.168.168.101`이 응답하지 않음 | 데이터링크 연결 상태, 노트북 주소(`192.168.168.102/24`), RPi 전원을 확인합니다. 10-20초 뒤 다시 시도합니다 |
| 서비스가 `failed`이고 로그에 `The IMU did not deliver data in 60 s`가 있음 | FC 전원과 FC-RPi UART 커넥터를 확인합니다. FC가 켜진 뒤 `~/start_vio.sh`를 실행합니다 |
| `imu_hz`가 0이거나 200보다 많이 낮음 | FC-RPi UART 연결을 확인합니다 |
| `brightness`가 아주 낮거나 높음 | 렌즈 덮개, 렌즈 오염, 조명을 확인합니다 |
| `tracking=lost`가 계속되거나 `p`가 계속 커짐 | 6장의 경고를 따릅니다 |
| xeye capture가 `no data on udp 127.0.0.1:14601`을 출력함 | VIO와 capture는 같은 카메라와 IMU 포트를 씁니다. `~/stop_vio.sh`로 VIO를 끈 뒤 capture를 실행합니다. 그래도 같으면 FC 전원과 UART를 확인합니다 |

## 10. 주의 사항

- 비행 중에는 VIO를 끄거나 다시 시작하지 않습니다.
- VIO가 실행 중이면 xeye capture와 캘리브레이션 캡처를 실행할 수 없습니다. 먼저 `~/stop_vio.sh`를 실행합니다.
- 카메라나 FC의 장착 위치를 바꾸면 카메라-IMU 캘리브레이션을 다시 해야 합니다. FC의 `AHRS_ORIENTATION`을 바꿀 때도 같습니다.
- VIO의 x축은 초기화한 순간의 드론 앞쪽입니다. 북쪽이 아닙니다.
