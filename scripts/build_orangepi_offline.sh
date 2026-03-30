#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"

echo "[librs250] root=${ROOT_DIR}"
echo "[librs250] python=${PYTHON_BIN}"

if [[ "$(dpkg --print-architecture)" != "arm64" ]]; then
  echo "[librs250] warning: this helper was prepared for Orange Pi arm64" >&2
fi

sudo python3 -m pip uninstall -y pyrealsense2 || true
sudo dpkg -r librealsense2 || true

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

cd "${BUILD_DIR}"

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_SHARED_LIBS=ON \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_WITH_TM2=ON \
  -DIMPORT_DEPTH_CAM_FW=ON \
  -DREALSENSE_FIRMWARE_CACHE_DIR="${ROOT_DIR}/third-party/fw-cache" \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE="${PYTHON_BIN}" \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_TOOLS=OFF \
  -DBUILD_UNIT_TESTS=OFF \
  -DBUILD_LEGACY_LIVE_TEST=OFF \
  -DBUILD_WITH_OPENMP=OFF \
  -DCHECK_FOR_UPDATES=OFF

make -j"$(nproc)"
sudo make install
sudo ldconfig

"${PYTHON_BIN}" - <<'PY'
import pyrealsense2 as rs
print("pyrealsense2_import=ok")
print("module_file=", rs.__file__)
print("device_count=", len(rs.context().query_devices()))
PY
