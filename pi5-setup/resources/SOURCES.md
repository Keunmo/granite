# resources 출처

이 폴더의 파일은 다른 레포나 서버에서 복사했다. 2026-11-11에 복사했고, granite와 뷰어는 2026-11-12에 다시 복사했다.

| 경로 | 출처 | 버전 | 변경 |
|---|---|---|---|
| `ar0234-rpi-driver/` | ar0234-rpi-driver 레포(origin: `https://github.com/Kurokesu/ar0234-rpi-driver.git`), uvifyR1PC2 `~/workspace/ar0234-rpi-driver` | 브랜치 `rpi5-ubuntu2404`, 커밋 `f2d4538` (깨끗한 작업 트리를 `rsync`로 복사) | `.git/`, `.github/`를 뺐다. 이 브랜치는 원격에 없다 |
| `mavlink-router/install.sh`, `build.sh`, `mavlink-router.service` | device-setup 레포(`git@github.com:uvify/device-setup.git`) `orin-nx/scripts/mavlink-router/` | 브랜치 `master`, 커밋 `3447c02` | 없음 |
| `mavlink-router/main.conf` | 이 폴더에서 새로 작성 | - | device-setup의 Jetson용 `main.conf`(`/dev/ttyTHS1`)를 Pi 5 UART(`/dev/ttyAMA0`, RTS/CTS)용으로 바꿨다 |
| `mavlink-router/mavlink-routerd-v4-aarch64`, `.sha256` | NAS `upload/devel/device-setup/orin-nx/mavlink-router/` | mavlink-router `v4` | 없음. device-setup `install.sh`가 받는 파일과 같다 |
| `vio/granite-vio.service`, `vio/granite-vio.default` | 이 폴더에서 새로 작성 (2026-11-11, 2026-11-12) | - | granite `cross/pi5/pi5_live_kurokesu_vio.sh`(브랜치 `mavlink`)를 부팅 때 실행한다. 서비스는 `config/service/`의 672x600 캘리브와 config를 쓴다 |
| `vio/capture_to_granite.py` | granite 레포 `eval/scripts/capture_to_granite.py` | 브랜치 `mavlink`, 커밋 `341a11e` | 없음. `renew-config.sh`가 Pi에서 실행한다 |
| `vio/granite/bin/`, `vio/granite/lib/` | granite 레포(`git@github.com:uvify/granite.git`) `cross/pi5/build.sh`의 결과 `~/cross/pi5-build/granite/` | 브랜치 `mavlink`, 커밋 `341a11e` (`vio/granite/DEPLOYED`). IMU 첫 데이터 대기 60 s를 포함한다 | 없음. 지상 시각화 메시지 version 2(기본 h264, 조각 전송, 전송률 상한, SCHED_IDLE)를 포함한다. `libgranite.so`는 sysroot의 OpenH264(Ubuntu 24.04 arm64 `libopenh264-dev`, `libopenh264-7` 2.4.1+dfsg-1)와 링크한다. .60의 `~/deploy/granite`는 이 번들과 같다(2026-11-12) |
| `vio/granite/config/` | 같은 커밋의 `cross/pi5/live/*.json` | 같다 | 없음. 448x400과 672x600 두 벌이다. 캘리브는 .60 `~/xeye-deploy/config/capture.yaml`(1920x1200, crop 1344x1200)에서 만들었다 |
| `vio/granite/scripts/` | 같은 커밋의 `cross/pi5/pi5_live_kurokesu_vio.sh`, `cross/pi5/granite-live-kurokesu.desktop`, `scripts/pi4_live_mono_vio.svg`(이름을 `pi5_live_kurokesu_vio.svg`로 바꿈) | 같다 | 없음 |
| `vio/UVIFY_VIO_Viewer-x86_64.AppImage` | granite 레포 `tools/vio_viewer/build_appimage.sh`의 결과 `tools/vio_viewer/dist/` | 브랜치 `mavlink`, 커밋 `3db6aa4` (plot은 최근 30 s) | 없음. 지상 PC(x86-64, Ubuntu 24.04 이상)용 뷰어다. OpenH264(Ubuntu 24.04 `libopenh264-7` 2.4.1)를 포함한다. Pi에는 설치하지 않는다 |
| `python/*.whl` | PyPI | pymavlink 2.4.50, fastcrc 0.5.0, lxml 6.1.3 (CPython 3.12, aarch64) | 없음 |

SHA256:

```text
8af69f4d0a56d28242de8647e05bd9b325e509610f27c787800118887874e67a  mavlink-router/mavlink-routerd-v4-aarch64
60d1342ba91795650635acf1e44c0d04bbb14319b5787f413a3303e15778fecc  python/fastcrc-0.5.0-cp312-cp312-manylinux_2_17_aarch64.manylinux2014_aarch64.whl
f683dc6300317700025e41d89a43e0276692ded16113a3c43eab704d605c58e5  python/lxml-6.1.3-cp312-cp312-manylinux2014_aarch64.manylinux_2_17_aarch64.whl
1ba0b7e86b15b981ed2ddacb30840892b7974e2a60a769fa3b796752b5677e00  python/pymavlink-2.4.50-cp312-cp312-manylinux2014_aarch64.manylinux_2_17_aarch64.manylinux_2_28_aarch64.whl
```

## 다시 만드는 방법

PC에서 실행한다. `P`는 이 `pi5-setup` 폴더이다.

```bash
P=~/workspace/ddp-drone/pi5-setup
# git archive는 쓰지 않는다. .gitattributes의 export-ignore가 docs/를 뺀다.
git -C ~/workspace/ar0234-rpi-driver switch rpi5-ubuntu2404
git -C ~/workspace/ar0234-rpi-driver status --porcelain   # 출력이 없어야 한다
rsync -a --delete --exclude .git --exclude .github ~/workspace/ar0234-rpi-driver/ $P/resources/ar0234-rpi-driver/
for f in install.sh build.sh mavlink-router.service; do
  git -C ~/workspace/device-setup show master:orin-nx/scripts/mavlink-router/$f > $P/resources/mavlink-router/$f
done
B=https://uvifya16f.synology.me:49152/upload/devel/device-setup/orin-nx/mavlink-router
curl -fsSL -o $P/resources/mavlink-router/mavlink-routerd-v4-aarch64 $B/mavlink-routerd-v4-aarch64
curl -fsSL -o $P/resources/mavlink-router/mavlink-routerd-v4-aarch64.sha256 $B/mavlink-routerd-v4-aarch64.sha256
pip download pymavlink==2.4.50 --platform manylinux2014_aarch64 --platform manylinux_2_17_aarch64 \
  --platform manylinux_2_28_aarch64 --python-version 3.12 --implementation cp --abi cp312 \
  --only-binary=:all: -d $P/resources/python
```

granite (`vio/granite/`)는 아래 명령으로 다시 만든다. granite 레포가 깨끗한 작업 트리이어야 한다. 그래야 `DEPLOYED`의 커밋이 파일과 맞는다.

```bash
P=~/workspace/ddp-drone/pi5-setup; G=~/workspace/granite_reliability; B=~/cross/pi5-build/granite; V=$P/resources/vio/granite
git -C $G status --porcelain -uno              # 출력이 없어야 한다
(cd $G && cross/pi5/build.sh)
rm -rf $V && mkdir -p $V/bin $V/lib $V/config $V/scripts
cp $B/granite_vio $B/granite_pi_cam_imu_vio $B/granite_mono_vio_dataset $V/bin/
cp $B/libgranite.so $V/lib/
cp $G/cross/pi5/live/*.json $V/config/
cp $G/cross/pi5/pi5_live_kurokesu_vio.sh $G/cross/pi5/granite-live-kurokesu.desktop $V/scripts/
cp $G/scripts/pi4_live_mono_vio.svg $V/scripts/pi5_live_kurokesu_vio.svg
chmod +x $V/bin/* $V/scripts/pi5_live_kurokesu_vio.sh
echo "$(git -C $G rev-parse --short HEAD) $(date -Is)" > $V/DEPLOYED
cp $G/eval/scripts/capture_to_granite.py $P/resources/vio/
(cd $V && find bin lib config scripts -type f | sort | xargs sha256sum > SHA256SUMS)
```
