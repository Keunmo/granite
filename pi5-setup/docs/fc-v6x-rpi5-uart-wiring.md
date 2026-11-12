# UVify V6X와 Raspberry Pi 5의 UART 연결

이 문서는 UVify V6X 비행 컨트롤러(FC)와 Raspberry Pi 5를 UART로 연결하는 방법을 설명한다. 이 연결로 Pi는 FC의 IMU(`HIGHRES_IMU`, 200 Hz)를 받고, VIO pose를 FC로 보낸다.

## 1. 기준 자료와 대상 장비

| 항목 | 내용 |
|---|---|
| FC | UVify V6X. USB 이름 `UVify-Core-V6X-MG10` |
| FC 펌웨어 | ArduPilot 4.7.2 공식 빌드(git `b18b5978`). 2026-11-11에 HEARTBEAT(`autopilot=3`)와 `AUTOPILOT_VERSION`으로 확인했다 |
| 컴패니언 | Raspberry Pi 5, Ubuntu 24.04 |
| 기준 문서 | Confluence [UVify V6X](https://uvify.atlassian.net/wiki/spaces/ATD/pages/5863636994/UVify+V6X), [UVify V5X](https://uvify.atlassian.net/wiki/spaces/ATD/pages/5853052953/UVify+V5X) |

> **주의:** V6X 핀아웃 문서는 없다. 이 문서의 커넥터 핀 순서는 V5X 핀아웃 문서(v1.1)를 기준으로 한다. V6X 문서에 따르면 USART2와 UART4의 MCU 핀은 V5X와 같다. 그러나 같은 커넥터 핀으로 나오는지는 확인되지 않았다. 배선 전에 테스터로 GND 핀과 VCC 핀을 확인한다.

## 2. 필요한 대역폭

계산은 MAVLink2, signing 없음, 8N1(1 byte = 10 bit) 기준이다.

| 방향 | 메시지 | 크기 [byte] | 비율 | 대역폭 |
|---|---|---|---|---|
| FC → Pi | `HIGHRES_IMU` | 62 + 14 = 76 | 200 Hz | 15.2 KB/s |
| FC → Pi | `HEARTBEAT`, `TIMESYNC`, `STATUSTEXT` 등 | - | 낮음 | 1 KB/s 미만 |
| Pi → FC | `ODOMETRY`(covariance 포함) | 약 233 + 14 = 247 | 20 Hz | 4.9 KB/s |
| Pi → FC | `VISION_POSITION_ESTIMATE`(`ODOMETRY` 대신 쓸 때) | 117 + 14 = 131 | 20 Hz | 2.6 KB/s |

- UART는 full-duplex이다. TX 선과 RX 선이 따로 있으므로 두 방향은 대역폭을 나누지 않는다.
- 921600 baud는 92 KB/s이다. FC → Pi 방향은 약 17%, Pi → FC 방향은 약 5%를 쓴다.
- 460800 baud(46 KB/s)도 충분하다. 115200 baud(11.5 KB/s)는 IMU 200 Hz만으로도 부족하다.
- pose 전송 비율은 카메라 frame rate와 같다. granite와 openvio는 frame마다 상태를 하나 낸다.

## 3. 사용할 FC 포트

**UART2 커넥터(ArduPilot `SERIAL3`)를 쓴다.** 이유는 다음과 같다.

- TELEM1과 TELEM2는 통신보드 커넥터(Hirose DF12 36핀)로만 나온다. 통신보드가 없으면 쓸 수 없다.
- UART2는 FC 아랫면의 SH 6핀 커넥터로 바로 나온다. 이 커넥터에는 flow control(CTS/RTS)이 있다.
- 통신보드의 CONN 포트는 VOUT이 12 V일 수 있다. UART2에는 이 위험이 없다.

## 4. 배선

FC UART2(JST SH 1.0 mm 6핀, 아랫면)를 Pi 5의 40핀 헤더(UART0)에 연결한다.

| FC 핀 | FC 신호 | Pi 5 핀 | Pi 5 신호 |
|---|---|---|---|
| 1 | VCC (5 V) | 연결하지 않는다 | - |
| 2 | TX (Out) | 10 | GPIO15 (RXD) |
| 3 | RX (In) | 8 | GPIO14 (TXD) |
| 4 | CTS (In) | 11 | GPIO17 (RTS) |
| 5 | RTS (Out) | 36 | GPIO16 (CTS) |
| 6 | GND | 6 | GND |

> **경고:** FC의 VCC(5 V) 핀을 Pi의 GPIO 핀에 연결하지 않는다. 5 V는 Pi의 GPIO를 손상시킨다. FC와 Pi는 전원을 따로 받고 GND만 공유한다.

- FC와 Pi의 UART는 모두 3.3 V이다. 레벨 변환기는 필요 없다.
- 4번과 5번(flow control)이 없어도 TX, RX, GND 3선으로 동작한다. flow control은 수신 buffer가 넘칠 때 데이터 손실을 막는다.

## 5. ArduPilot 설정

V6X 문서에 따르면 ArduPilot의 시리얼 순서는 다음과 같다.

| ArduPilot | MCU UART | 커넥터 | 문서상 기본값 |
|---|---|---|---|
| `SERIAL1` | UART7 | 통신보드 TELEM1 | MAVLink2, 921600 |
| `SERIAL2` | UART5 | 통신보드 TELEM2 | MAVLink2, 921600 |
| `SERIAL3` | USART2 | UART2 | RC 입력(GHST) |
| `SERIAL4` | UART4 | UART4 | MAVLink2, 57600(OSD) |
| `SERIAL5` | USART1 | GPS module #1 | GPS, 230400 |
| `SERIAL6` | UART8 | GPS module #2 | GPS, 230400 |
| `SERIAL7` | USART3 | Console | - |

UART2를 쓰려면 다음 순서로 설정한다.

1. `SERIAL3_PROTOCOL`을 `2`(MAVLink2)로 바꾼다.
2. `SERIAL3_BAUD`를 `921`로 바꾼다.
3. flow control 선을 연결했으면 `BRD_SER3_RTSCTS`를 `1`로 바꾼다.
4. `SR3_*`에서 필요 없는 stream을 `0`으로 바꾼다.
5. FC를 재부팅한다.

ArduPilot은 포트의 TX buffer가 모자라면 메시지를 보내지 않고 건너뛴다. baud가 낮으면 IMU 수신 비율이 경고 없이 떨어질 수 있다.

## 6. Raspberry Pi 5 설정

1. `/boot/firmware/config.txt`에 `dtparam=uart0=on`을 추가한다. flow control을 쓰면 `dtoverlay=uart0-pi5,ctsrts`를 대신 추가한다.
2. `/boot/firmware/cmdline.txt`에 `console=serial0,...` 항목이 있으면 지운다.
3. Pi를 재부팅한다.
4. `/dev/ttyAMA0`이 있는지 확인한다.

## 7. mavlink-router 설정

mavlink-routerd가 FC 포트를 맡는다. capture의 `imu_mavlink`(UDP 127.0.0.1:14601)와 VIO pose 송신 프로그램은 router의 UDP endpoint를 쓰므로 바꿀 필요가 없다.

`main.conf`의 `[UartEndpoint fc]`만 바꾼다.

```ini
[UartEndpoint fc]
Device = /dev/ttyAMA0
Baud = 921600
# flow control 선을 연결했을 때만:
FlowControl = true
```

변경 뒤 router를 다시 시작한다.

```bash
sudo systemctl restart mavlink-router
```

## 8. 다른 포트

| 포트 | 장점 | 확인할 점 |
|---|---|---|
| TELEM1 (`SERIAL1`, 통신보드 CONN1 또는 CONN2) | 기본값이 MAVLink2 921600이다. 파라미터를 바꿀 필요가 없다 | 통신보드가 있어야 한다. 어느 CONN이 TELEM1인지 확인되지 않았다. HM-TRP 무선 모뎀이 어느 TELEM을 쓰는지 확인되지 않았다. CONN의 VOUT은 5 V 또는 12 V이다 |
| UART4 (`SERIAL4`, SH 4핀: VCC, TX, RX, GND) | 4핀이라 배선이 단순하다 | flow control이 없다. `SERIAL4_BAUD`를 `921`로 바꿔야 한다 |
| TELEM2 (`SERIAL2`, 통신보드) | 기본값이 MAVLink2 921600이다 | MG10-AI 변형은 PC8을 모터 출력 11로 쓴다. PC8은 TELEM2의 RTS이므로 flow control을 못 쓸 수 있다 |

통신보드 CONN 포트의 핀 순서는 VOUT, RXD, TXD, RTS, CTS, GND이다(Pixhawk TELEM 순서).

> **경고:** 통신보드 CONN 포트의 VOUT 핀을 Pi에 연결하지 않는다. VOUT은 조립 사양에 따라 12 V일 수 있다.

## 9. 연결 확인

1. HEARTBEAT를 받는지 확인한다. HEARTBEAT의 `autopilot` 값은 `3`(ArduPilot)이어야 한다.
2. capture를 실행한다.
3. `imu_fc.csv`의 IMU 수신 비율을 확인한다. 목표는 200 Hz이다.

## 10. 실기 확인 결과 (2026-11-11, uvify-pi5-01)

4장의 6선 배선(flow control 포함)으로 확인했다.

변경 전 FC 값은 다음과 같았다. 문서상 기본값과 다른 값이 있다.

| 파라미터 | 변경 전 | 변경 후 |
|---|---|---|
| `SERIAL3_PROTOCOL` | 23 (RCIN) | 2 (MAVLink2) |
| `SERIAL3_BAUD` | 115200 | 921 |
| `SERIAL3_OPTIONS` | 4 (half-duplex) | 0 |
| `BRD_SER3_RTSCTS` | 2 (auto) | 1 |
| `SERIAL1_BAUD`, `SERIAL2_BAUD` | 115 (문서상 기본값 921과 다름) | 바꾸지 않음 |

- `SERIAL3_OPTIONS` 4(half-duplex)가 남아 있으면 TX와 RX가 한 선을 쓴다. 이 값은 반드시 0으로 바꾼다.
- ArduPilot 4.7.2는 `SR3_*` 파라미터에 응답하지 않았다. stream 비율 파라미터의 이름은 확인하지 않았다.
- Pi `config.txt`에 `dtoverlay=uart0-pi5,ctsrts`를 추가했다. 재부팅 뒤 `/dev/ttyAMA0`이 생겼다. 백업은 `config.txt.bak-uart0`이다.
- mavlink-router 서비스는 `DynamicUser=yes`, `SupplementaryGroups=dialout`으로 실행된다. 그래서 `/dev/ttyAMA0`(dialout 그룹)을 열 수 있다. `main.conf` 백업은 `/etc/mavlink-router/main.conf.bak-usb`이다.

| 시험 | 결과 |
|---|---|
| FC → Pi (`/dev/ttyAMA0`, 921600, RTS/CTS) | HEARTBEAT `autopilot=3`, `TIMESYNC` 수신 |
| Pi → FC | `AUTOPILOT_VERSION` 요청에 `COMMAND_ACK` 0과 버전 4.7.2 응답 |
| capture 10초 (`config/capture.yaml`, router 경유) | `imu_fc.csv` 1654 샘플, 평균 200.0 Hz, 최대 간격 8.0 ms. `cam0` 20.0 Hz. FC가 `HIGHRES_IMU` 200 Hz 요청을 받아들였다 |

## 11. 확인되지 않은 항목

- V6X 커넥터의 실제 핀 순서. 이 문서는 V5X 문서를 기준으로 한다. 10장의 시험에서 4장의 핀 순서로 동작했다.
- 대상 FC에 통신보드가 있는지.
- USB를 뺀 상태에서 FC를 별도 전원으로 구동하는 시험.
- 긴 시간 동안의 UART 수신 손실.
