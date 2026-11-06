# Raspberry Pi 4 크로스 빌드

데스크톱(Ubuntu, x86_64)에서 Pi 4(Ubuntu 24.04, aarch64)용 Granite를 빌드하고 배포합니다. Pi 4에서는 빌드하지 않습니다. Pi 4 직접 빌드는 9–21분이 걸리고, 크로스 빌드는 약 1분이 걸립니다.

| 파일 | 내용 |
|---|---|
| `pi4-ubuntu2404.toolchain.cmake` | 컴파일러(`aarch64-linux-gnu-g++-13`), sysroot, CPU 옵션(`-mcpu=cortex-a72+crc`) |
| `CMakeLists.txt` | `libgranite.so`, `granite_vio`, `granite_pi_cam_imu_vio`, `granite_mono_vio_dataset` |
| `build.sh` | Pangolin 정적 빌드(처음 한 번)와 Granite 빌드 |
| `deploy.sh` | Pi 4의 `~/granite_eval/bin/granite/`에 배포 |

## 사용법

1. 데스크톱에서 빌드합니다.

   ```bash
   cd ~/workspace/granite_reliability
   cross/pi4/build.sh
   ```

2. Pi 4에 배포합니다. live 스크립트와 바탕화면 아이콘도 바꾸었으면 `--scripts`를 붙입니다.

   ```bash
   cross/pi4/deploy.sh            # 바이너리만
   cross/pi4/deploy.sh --scripts  # 바이너리, live 스크립트, 바탕화면 아이콘
   ```

- 빌드 폴더는 `~/cross/pi4-build/`입니다(`BUILD_ROOT`로 바꿈). Pangolin은 `pangolin/`, Granite는 `granite/`에 있습니다.
- Pangolin을 다시 빌드하려면 `~/cross/pi4-build/pangolin/`을 지우고 `build.sh`를 다시 실행합니다.
- 배포는 항상 같은 폴더를 덮어씁니다. 그 폴더에는 가장 최근 빌드만 남습니다. 배포한 커밋은 `bin/granite/DEPLOYED`에 있습니다. 커밋하지 않은 변경이 있으면 커밋 뒤에 `-dirty`가 붙습니다.
- 배포는 파일을 임시 이름으로 복사한 다음 이름을 바꿉니다. 그래서 Pi 4에서 실행 중인 `granite_vio`는 이전 파일을 계속 씁니다.
- 실행 파일은 같은 폴더의 `libgranite.so`를 씁니다(RUNPATH `$ORIGIN`).

## 준비

- 컴파일러: `sudo apt install g++-13-aarch64-linux-gnu gcc-13-aarch64-linux-gnu`
- GLEW 헤더: `sudo apt install libglew-dev`. Pi 4 sysroot에는 GLEW 헤더가 없습니다. 그래서 데스크톱 헤더(2.2.0, Pi 4의 `libGLEW.so.2.2`와 같은 버전)를 씁니다.
- sysroot: `~/sysroots/pi4-ubuntu2404`(`PI_SYSROOT`로 바꿈). Pi 4의 라이브러리와 헤더의 사본입니다. 처음 한 번 만듭니다.

  ```bash
  mkdir -p ~/sysroots/pi4-ubuntu2404 && cd ~/sysroots/pi4-ubuntu2404
  rsync -aR 10.20.10.50:/usr/include :/usr/lib/aarch64-linux-gnu :/usr/lib/gcc/aarch64-linux-gnu \
      :/usr/share/eigen3 :/usr/lib/cmake :/usr/share/pkgconfig ./
  ```

  Pi 4의 절대 경로 심볼릭 링크(예: `/lib/aarch64-linux-gnu/...`)는 sysroot 안을 가리키도록 상대 링크로 바꿔야 합니다. 기존 sysroot는 이 작업을 했습니다. 그때 쓴 명령은 기록이 없습니다. `symlinks -rc .`(패키지 `symlinks`)로 같은 결과를 얻을 수 있습니다(미검증).

- Pi 4에 새 패키지(예: `libopencv-dev`의 새 버전)를 설치하면 sysroot를 다시 rsync합니다.

## 빌드 옵션

- Pangolin: video, python, 도구는 끕니다. Pi 4 libstdc++ 헤더는 `<cstdint>`를 간접으로 포함하지 않으므로 `-include cstdint`를 씁니다. sysroot에 `GL/glu.h`가 없으므로 `-DGLEW_NO_GLU`를 씁니다.
- `libgranite.so`는 rosbag 없이 빌드합니다(`GRANITE_NO_ROSBAG`, sysroot에 boost가 없음). 그래서 Pi 4의 `granite_vio`는 `--dataset-type bag`을 읽지 못합니다. EuRoC 폴더를 씁니다.
- 실행 파일은 `--allow-shlib-undefined`로 연결합니다. sysroot의 일부 라이브러리(libgdal)는 Pi가 실행 시에 고르는 BLAS/LAPACK의 심볼을 씁니다.
