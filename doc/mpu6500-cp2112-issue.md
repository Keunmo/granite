# Pi 4 리그의 CP2112 IMU 전송 시간 초과

이 문서는 Pi 4 live 리그에서 IMU 읽기가 가끔 실패하는 문제를 기록합니다. 증상, 원인 조사, Granite의 대응, 남은 선택지를 다룹니다.

- 리그: Pi 4(`uvify@10.20.10.50`), Ubuntu 커널 `6.8.0-1064-raspi`.
- IMU: MPU-9250(`WHO_AM_I` 0x71). 가속도계와 자이로는 MPU-6500 코어입니다. 주소는 0x68입니다.
- 브리지: Silicon Labs CP2112 HID USB-I2C 브리지(`10c4:ea90`), 커널 드라이버 `hid-cp2112`. 2026-11-06에는 `/dev/i2c-23`입니다.
- 관련 문서: `doc/xeye_to_granite_guide.md` 7절.

상태 표기는 `xeye_to_granite_guide.md`와 같습니다. **[검증]**은 실행해서 확인한 내용입니다. **[코드 확인]**은 코드를 읽고 확인한 내용입니다. **[미검증]**은 확인하지 않은 내용입니다.

## 1. 결론

- 약 150회 전송에 1회, I2C 전송이 `errno 110`(`ETIMEDOUT`)으로 실패합니다. 실패 전송은 약 54 ms 걸립니다 [검증].
- CP2112가 전송 상태 질문에 응답을 보내지 않습니다. 응답은 늦게 오는 것이 아니라 오지 않습니다. 드라이버는 50 ms를 기다린 다음 로그 없이 `-ETIMEDOUT`을 돌려줍니다 [검증, usbmon과 dynamic_debug].
- 응답이 어디에서 사라지는지(CP2112 자체, Pi 4의 USB 허브 경로, 케이블)는 확인하지 않았습니다 [미검증].
- 앱 코드 문제가 아닙니다. xeye `ImuMPU6050`의 FIFO 루프를 그대로 옮긴 시험 프로그램에서도 같은 비율로 납니다. 카메라 없이 IMU만 읽어도 납니다. 읽기 주기를 20~100 ms로 바꿔도 줄지 않습니다 [검증].
- 데이터 오염은 없습니다. 녹화 데이터에 중복 샘플과 깨진 샘플이 없습니다 [검증].
- Granite는 `1af850d`부터 오류 뒤에 FIFO를 유지하고 샘플 경계만 다시 맞춥니다. 그래서 손실은 약 0.3%이고 IMU 빈칸은 최대 약 28 ms입니다(3절).

## 2. 원인 조사

### 2.1 드라이버 동작 [코드 확인, `drivers/hid/hid-cp2112.c` v6.8]

한 I2C 쓰기-읽기 전송은 다음 USB HID 보고서로 진행합니다.

| 방향 | 보고서 | 예 | 뜻 |
|---|---|---|---|
| OUT | `0x11` Data Write Read Request | `11 d0 0030 01 74` | 주소 0x68(0xd0)에 레지스터 0x74를 쓰고 48 바이트를 읽음 |
| OUT | `0x15` Transfer Status Request | `15 01` | 전송 상태를 질문함 |
| IN | `0x16` Transfer Status Response | `16 01 02 ...` | 진행 중(읽는 중) |
| IN | `0x16` Transfer Status Response | `16 02 05 ...` | 완료, 성공 |
| OUT | `0x12` Data Read Force Send | `12 0002` | 읽은 데이터를 보내라고 요청함 |
| IN | `0x13` Data Read Response | `13 00 02 ...` | 읽은 데이터 |

- `cp2112_i2c_xfer`는 상태 질문을 최대 `XFER_STATUS_RETRIES`(10)회 보냅니다. 응답이 "진행 중"이면 다시 질문합니다.
- 각 질문 뒤에 `cp2112_wait`가 `RESPONSE_TIMEOUT`(50 ms)까지 기다립니다. 응답이 없으면 `-ETIMEDOUT`을 돌려줍니다.
- 상태 질문의 시간 초과는 경고 로그를 남기지 않습니다. 전송 취소(`CP2112_CANCEL_TRANSFER`)도 보내지 않습니다. 취소와 경고는 "진행 중" 응답이 10회 이어질 때만 생깁니다.
- 드라이버는 장치 설정에서 `retry_time`을 1로 정합니다. 이 값은 I2C 쪽 재시도이고, USB 응답 손실과는 관계가 없습니다.
- 그래서 커널 로그(`journalctl -k`, `/var/log/kern.log`)에는 오류가 남지 않습니다 [검증].

### 2.2 usbmon과 dynamic_debug 측정 [검증, 30 s]

조건: `imu_fifo_test`(4절), 192 바이트 청크, 20 ms 주기, 모드 1(경계 재정렬). 같은 실행에서 usbmon과 `hid_cp2112` dynamic_debug를 같이 기록했습니다.

| 값 | 결과 |
|---|---|
| I2C 전송 | 2808회(개수 읽기 1402, 데이터 읽기 1406) |
| 실패 | 18회(개수 읽기 10, 데이터 읽기 8), 모두 `errno 110` |
| 응답이 필요한 OUT 요청 | 8603 |
| 받은 IN 응답 | 8585 |
| 차이 | 18 (실패 수와 같음) |
| 샘플 비율 | 198.8 Hz (설정 200 Hz) |

- 실패마다 상태 질문 `15 01`의 OUT은 정상으로 끝납니다(상태 0). IN URB는 대기 중입니다. 그러나 IN 응답이 오지 않습니다.
- 늦은 응답이 다음 요청에 섞이는 경우는 없습니다. 받은 응답 수가 정확히 실패 수만큼 적습니다.
- 실패는 항상 두 번 연속입니다. 두 번째 실패는 항상 다음 개수 읽기(레지스터 0x72)입니다. 세 번째 전송은 정상입니다. 이 현상이 장치 상태와 관계가 있는지는 확인하지 않았습니다 [미검증].

usbmon의 실패 한 쌍입니다(장치 `1:004`, 시각 단위 µs).

```text
1723893390 S Io:1:004:1 -115:1 6 = 11d00030 0174        # 데이터 읽기 48 B 요청
1723895381 S Io:1:004:1 -115:1 2 = 1501                 # 상태 질문 1
1723897225 C Ii:1:004:1 0:1 64 = 16010200 ...           # 응답: 진행 중
1723897262 S Io:1:004:1 -115:1 2 = 1501                 # 상태 질문 2
1723900231 C Ii:1:004:1 0:1 64 = 16010200 ...           # 응답: 진행 중
1723900294 S Io:1:004:1 -115:1 2 = 1501                 # 상태 질문 3
1723902354 C Io:1:004:1 0:1 2 >                         # OUT 완료. IN 응답은 오지 않음
1723973784 S Io:1:004:1 -115:1 6 = 11d00002 0172        # 71 ms 뒤 다음 개수 읽기
1723975451 S Io:1:004:1 -115:1 2 = 1501                 # 상태 질문
1723976370 C Io:1:004:1 0:1 2 >                         # OUT 완료. IN 응답은 오지 않음
1724047585 S Io:1:004:1 -115:1 6 = 11d00002 0172        # 다음 개수 읽기
1724049518 S Io:1:004:1 -115:1 2 = 1501
1724051261 C Ii:1:004:1 0:1 64 = 16020500 ...           # 응답: 완료. 정상으로 돌아옴
```

같은 실패의 커널 로그(dynamic_debug)입니다.

```text
[87740.315846] I2C write-read 0x68 wlen 1 rlen 48
[87740.319730] xfer status: 01 02 0000 0000
[87740.322739] xfer status: 01 02 0000 0000
[87740.375573] I2C transfer finished: -110
[87740.396252] I2C write-read 0x68 wlen 1 rlen 2
[87740.449600] I2C transfer finished: -110
[87740.470043] I2C write-read 0x68 wlen 1 rlen 2
[87740.473781] xfer status: 02 05 0000 0002
```

### 2.3 USB 경로

`lsusb -t`의 결과입니다 [검증].

```text
Bus 001.Port 001: Dev 001, Class=root_hub, Driver=xhci_hcd/1p, 480M
    |__ Port 001: Dev 002, If 0, Class=Hub, Driver=hub/4p, 480M
        |__ Port 004: Dev 004, If 0, Class=Human Interface Device, Driver=usbhid, 12M   # CP2112
```

- CP2112는 Full Speed(12 Mbit/s) 장치입니다. Pi 4 보드의 USB 2.0 허브(480 Mbit/s) 아래에 있습니다.
- 그래서 CP2112의 전송은 허브의 Transaction Translator를 지납니다. 응답이 이 경로에서 사라질 가능성이 있습니다. 시험하지 않았습니다 [미검증].
- 같은 허브에 다른 HID 장치(`Dev 005`, 12M)가 있습니다. 이 장치의 영향은 시험하지 않았습니다 [미검증].

## 3. Granite의 대응 (`1af850d`)

코드: `src/device/pi_camera_imu.cpp`의 `imuLoop`, `include/granite/device/pi_sensor_utils.h`의 `FifoSampleClock`.

1. 전송이 실패하면 FIFO를 리셋하지 않습니다. 다음 주기에서 FIFO 개수를 다시 읽습니다.
2. 개수가 12 바이트(샘플 1개)의 배수가 아니면, 나머지 바이트를 읽어서 버립니다. 이것으로 샘플 경계를 다시 맞춥니다.
3. 데이터 읽기가 실패했을 때만 샘플 시계를 다시 시작합니다. 개수 읽기 실패는 샘플을 잃지 않습니다.
4. `FifoSampleClock::reset()`은 마지막 시각을 유지합니다. 그래서 IMU 시각은 항상 증가합니다.

결과 [검증, Pi 4, 60 s 녹화]:

| 방식 | IMU 비율 | 최대 빈칸 | 빈칸 합 |
|---|---|---|---|
| 이전(xeye와 같음): 오류 뒤 FIFO 리셋 | 192.5 Hz | 201 ms | 2.35 s |
| `1af850d`: FIFO 유지와 경계 재정렬 | 199.1 Hz | 28 ms | 0.37 s |

- live 앱의 `LIVE` 줄에서 `imu_i2c_errors`가 이 실패 수입니다(30 s에 약 12회, 60 s에 약 28회).
- 같은 대응을 xeye `ImuMPU6050`과 openvio에도 쓸 수 있습니다. 아직 적용하지 않았습니다.
- 정지 중 녹화에서 본 가속도 이상치는 실제 충격 한 번입니다. 이 문제와 관계가 없습니다 [검증].

## 4. 재현 절차

### 4.1 시험 프로그램

`eval/src/imu_fifo_test.cc`는 xeye `imu_mpu6050.cc`의 `SetupFifoMode`와 `FetchFifoData`를 옮긴 프로그램입니다. 전송마다 시간을 재고, 실패를 단조 시각과 함께 출력합니다.

1. 데스크톱에서 빌드합니다.

   ```bash
   cd ~/workspace/granite_reliability
   aarch64-linux-gnu-g++-13 -O2 -std=c++17 eval/src/imu_fifo_test.cc -o imu_fifo_test
   scp imu_fifo_test 10.20.10.50:granite_eval/imutest/
   ```

2. Pi 4에서 granite live 앱과 openvio를 끕니다. IMU는 한 프로세스만 읽어야 합니다.
3. CP2112의 I2C 번호를 확인합니다.

   ```bash
   for d in /sys/bus/i2c/devices/i2c-*; do echo $d $(cat $d/name); done | grep CP2112
   ```

4. 시험을 실행합니다. 인자는 장치, 초, 청크 바이트, 주기 ms, 모드, 오류 뒤 대기 ms입니다. 모드 0은 오류 뒤 FIFO 리셋(xeye)이고, 모드 1은 경계 재정렬(Granite)입니다.

   ```bash
   ~/granite_eval/imutest/imu_fifo_test /dev/i2c-23 30 192 20 1 0 | tee test.log
   ```

5. 끝의 요약에서 `errors`, `rate`, `errno 110` 수를 확인합니다.

### 4.2 usbmon과 dynamic_debug (root 필요)

Pi 4의 `/etc/sudoers.d/usb-debug`는 다음 명령만 암호 없이 허용합니다. 조사가 끝나면 사용자가 `sudo rm /etc/sudoers.d/usb-debug`로 지웁니다.

1. usbmon을 올리고 debugfs를 마운트합니다.

   ```bash
   sudo /usr/sbin/modprobe usbmon
   sudo /usr/bin/mount -t debugfs none /sys/kernel/debug
   ```

2. `hid_cp2112`의 디버그 출력을 켭니다.

   ```bash
   echo 'module hid_cp2112 +p' | sudo /usr/bin/tee /sys/kernel/debug/dynamic_debug/control
   ```

3. 버스 1의 USB 기록을 시작합니다. `lsusb`에서 CP2112의 버스 번호를 확인합니다.

   ```bash
   sudo /usr/bin/cat /sys/kernel/debug/usb/usbmon/1u > usbmon.txt &
   ```

4. 4.1절의 시험을 실행합니다.
5. 기록을 멈추고 커널 로그를 저장합니다.

   ```bash
   sudo kill %1
   sudo /usr/bin/dmesg > dmesg_all.txt
   ```

6. 디버그 출력을 끕니다.

   ```bash
   echo 'module hid_cp2112 -p' | sudo /usr/bin/tee /sys/kernel/debug/dynamic_debug/control
   ```

- 2026-11-06의 기록은 Pi 4 `~/granite_eval/imutest/`에 있습니다(`usbmon.txt`, `dmesg_all.txt`, `test.log`).
- usbmon에서 `S Io ... = 1501` 뒤에 `C Ii ... = 16...`이 오지 않으면 응답 손실입니다.

## 5. 남은 선택지

| 선택지 | 내용 | 장점 | 단점 |
|---|---|---|---|
| A. 현재 상태 유지 | `1af850d`의 재정렬 | 작업 없음. 손실 약 0.3%, 최대 빈칸 약 28 ms | 실패 한 번에 약 54 ms × 2를 기다립니다. 원인은 그대로입니다. |
| B. hidraw 사용자 공간 드라이버 | `/dev/hidraw*`로 CP2112 보고서를 직접 보냄. 응답이 없으면 짧은 시간(예: 5 ms) 뒤에 상태를 다시 질문 | 빈칸이 짧아질 수 있습니다 [미검증] | `/dev/hidraw*` 권한용 udev 규칙이 필요합니다(현재 root 전용). 코드가 늘어납니다. 응답 손실 자체는 없어지지 않습니다. |
| C. MPU-9250을 Pi 4 I2C에 직접 연결 | Pi 4 GPIO의 `/dev/i2c-1`(3.3 V, SDA GPIO2, SCL GPIO3) | USB 경로가 없어집니다. 전송 시간도 짧아집니다 [미검증] | 배선이 필요합니다. Pi 5 배선은 `doc/Pi5_MPU9250_Wiring.md`를 참고합니다. |

- 추천은 C입니다. 배선이 어려우면 B를 시험합니다.
- 원인 위치를 구분하는 쉬운 시험이 있습니다. CP2112를 Pi 5나 데스크톱에 꽂고 4.1절의 시험을 실행합니다. 다른 호스트에서도 실패하면 CP2112 쪽 문제입니다. 실패하지 않으면 Pi 4의 USB 경로 문제입니다. 아직 하지 않았습니다.
