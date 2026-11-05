# MPU-9250를 Raspberry Pi 5 GPIO에 직접 연결하기

이 문서는 MPU-9250을 CP2112 USB 브리지 대신 Raspberry Pi 5의 40핀 헤더에 직접 연결하는 방법을 정합니다. 핀 번호는 [Raspberry Pi 공식 GPIO 문서](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio)의 물리 핀 번호(1–40)입니다. `GPIO n`은 BCM GPIO 번호입니다.

## 결론

**SPI0 + INT 핀(GPIO 25)으로 연결합니다.** 커널 `inv-mpu6050-spi` IIO 드라이버가 INT 신호로 샘플마다 시각을 기록합니다.

I2C 연결도 동작합니다. 배선이 2선 적고 overlay가 이미 있습니다. 하지만 SPI가 더 빠르고, 지연이 짧고, 배선 잡음에 강합니다. VIO에서 두 방식의 시각 정확도는 같은 INT 방식이므로 같습니다. 차이는 버스 여유와 신호 안정성입니다.

## 비교

| 항목 | CP2112 (현재) | GPIO I2C1 | GPIO SPI0 (권장) |
|---|---|---|---|
| 14 B 읽기 시간 | 7.0 ms (실측) | 약 0.4 ms (400 kHz 계산값) | 약 0.12 ms (1 MHz 계산값) |
| 실용 샘플률 | 200 Hz (상한 약 400 Hz, 미검증) | 1 kHz 가능 (버스 점유 약 40%) | 1 kHz 이상 |
| 샘플 시각 | FIFO 번호로 추정, 오차 약 1–2 ms (시뮬레이션) | INT 기반 커널 시각 | INT 기반 커널 시각 |
| 신호 방식 | USB | open-drain, pull-up 필요 | push-pull |
| 배선 수 (INT 포함) | USB 1개 | 6개 | 7개 |
| 커널 설정 | `hid-cp2112` | 기존 overlay `i2c-sensor` | 사용자 overlay (이 레포에 포함) |
| 자력계(AK8963) | 접근 가능 | bypass로 접근 가능 | MPU 내부 I2C master로만 접근 |

- MPU-9250 사양: I2C는 최대 400 kHz입니다. SPI는 모든 레지스터에 1 MHz, 센서·interrupt 레지스터 읽기에 20 MHz까지 지원합니다.
- 커널 드라이버는 SPI 클록 하나만 씁니다. 따라서 overlay는 쓰기 한계인 1 MHz로 설정합니다. 1 MHz에서도 1 kHz 샘플률에 충분합니다.
- VIO에는 자력계가 필요 없습니다. 따라서 SPI의 자력계 제약은 문제가 아닙니다.
- 이 Pi(Ubuntu 24.04, kernel `6.8.0-1064-raspi`)에는 `inv-mpu6050`, `inv-mpu6050-i2c`, `inv-mpu6050-spi` 모듈이 있습니다.

## 권장 배선: SPI0

먼저 Pi의 전원을 끕니다. MPU 보드를 CP2112에서 분리합니다. 한 MPU에는 호스트 하나만 연결합니다.

| MPU-9250 보드 핀 | 칩 신호 (SPI 모드) | Pi 5 물리 핀 | Pi 신호 |
|---|---|---|---|
| VCC | 전원 | **17** | 3V3 |
| GND | 접지 | **25** | GND |
| SCL | SCLK | **23** | GPIO 11 (SPI0 SCLK) |
| SDA | SDI (MOSI) | **19** | GPIO 10 (SPI0 MOSI) |
| AD0 | SDO (MISO) | **21** | GPIO 9 (SPI0 MISO) |
| NCS | nCS | **24** | GPIO 8 (SPI0 CE0) |
| INT | INT | **22** | GPIO 25 |
| FSYNC | FSYNC | **20** | GND |
| EDA, ECL | 보조 I2C | 연결 안 함 | — |

모든 신호가 헤더의 17–25번 핀 영역에 모입니다. 선은 짧게(약 20 cm 이하) 만듭니다.

### 설정

1. 레포의 overlay를 컴파일합니다.

   ```sh
   dtc -@ -I dts -O dtb -o mpu9250-spi0.dtbo scripts/pi5/mpu9250-spi0-overlay.dts
   ```

2. dtbo를 설치합니다.

   ```sh
   sudo install -m 644 mpu9250-spi0.dtbo /boot/firmware/overlays/
   ```

3. `/boot/firmware/config.txt`의 `[all]` 아래에 두 줄을 추가합니다.

   ```ini
   dtparam=spi=on
   dtoverlay=mpu9250-spi0
   ```

4. 재부팅합니다.

5. 드라이버 인식을 확인합니다.

   ```sh
   dmesg | grep -i mpu
   cat /sys/bus/iio/devices/iio:device*/name
   ```

   `mpu9250`이 보이면 정상입니다.

overlay는 spidev0.0(CE0)을 끄고 CE1(`/dev/spidev0.1`)은 남깁니다. INT 핀은 `dtoverlay=mpu9250-spi0,int_pin=<GPIO>`로 바꿀 수 있습니다. 이 overlay를 Pi 5 base DTB에 `fdtoverlay`로 적용하는 검사는 통과합니다. 실제 배선 동작은 미검증입니다.

## 대안 배선: I2C1

배선을 줄이려면 I2C1을 씁니다.

| MPU-9250 보드 핀 | Pi 5 물리 핀 | Pi 신호 |
|---|---|---|
| VCC | **1** | 3V3 |
| GND | **6** | GND |
| SDA | **3** | GPIO 2 (I2C1 SDA) |
| SCL | **5** | GPIO 3 (I2C1 SCL) |
| INT | **7** | GPIO 4 |
| AD0 | **9** | GND (주소 0x68) |
| NCS | **17** | 3V3 (I2C 모드 고정) |
| FSYNC | **14** | GND |
| EDA, ECL | 연결 안 함 | — |

`/boot/firmware/config.txt` 설정입니다.

```ini
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000
dtoverlay=i2c-sensor,mpu9250,int_pin=4
```

현재 I2C1 클록은 기본값 100 kHz입니다. 따라서 `i2c_arm_baudrate=400000`이 필요합니다. GPIO 2와 GPIO 3에는 Pi 보드의 고정 pull-up이 있습니다.

## 주의

- **경고: Pi GPIO는 3.3 V 신호만 받습니다. MPU 보드 VCC에 5 V를 연결하면 안 됩니다.** 일부 보드는 SDA·SCL pull-up을 VCC에 연결합니다. VCC가 5 V이면 Pi 핀에 5 V가 걸릴 수 있습니다.
- 연결 전에 보드 전원을 3.3 V로 넣고, 신호 핀의 대기 전압이 3.3 V 이하인지 측정합니다.
- 데이터시트는 쓰지 않는 FSYNC를 GND에 연결하도록 권장합니다. FSYNC는 카메라 strobe 동기화에 쓸 수도 있습니다.
- 보드 핀 이름(VCC, SCL, SDA, AD0, NCS, INT, FSYNC, EDA, ECL)은 일반적인 GY-9250/GY-6500 계열 기준입니다. 보드의 실제 인쇄를 확인합니다.

## 미검증 항목과 남은 작업

- 이 문서의 배선은 실제 하드웨어에서 시험하지 않았습니다.
- IIO 드라이버의 샘플 시각 정밀도는 측정하지 않았습니다.
- `granite_pi_cam_imu_vio` 예제는 현재 CP2112 경로(i2c-dev + FIFO)만 지원합니다. 직접 연결을 쓰려면 IIO 버퍼(`/dev/iio:deviceN`) 입력 경로를 추가해야 합니다.
