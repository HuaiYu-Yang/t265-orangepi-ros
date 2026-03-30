# T265 On Orange Pi ROS Noetic
If you need to use this project, you need to download the release into the project folder and rename the folder to 'bundle'. This folder contains four parts, which the automation script will handle automatically. You only need to unzip the zip files; there is no need to extract the tar parts. This file already includes all the offline dependencies required for compilation.

Offline-friendly Intel T265 setup for Orange Pi arm64 on Ubuntu 20.04 / ROS Noetic / Python 3.8.

This repository is organized for GitHub upload constraints:

- the large offline bundle is split into `<25MB` parts under `bundle/`
- Windows deployment script automatically reconstructs the tarball locally before upload
- Orange Pi still receives the same offline build bundle and uses the same one-step build script

## What is included

- split offline bundle parts
- Orange Pi offline build helper
- ROS Python node that publishes:
  - `/t265/odom/raw`
  - `/t265/odom/filtered`
- two patch files documenting source modifications

Evaluation and CSV plotting tools are intentionally excluded from this repo.

## Repository layout

```text
.
├─ README.md
├─ .gitignore
├─ bundle
│  ├─ librealsense-2.50.0-offline-arm64.tar.gz.part01
│  ├─ librealsense-2.50.0-offline-arm64.tar.gz.part02
│  ├─ librealsense-2.50.0-offline-arm64.tar.gz.part03
│  └─ librealsense-2.50.0-offline-arm64.tar.gz.part04
├─ patches
│  ├─ 0001-python-offline-pybind11-and-init.patch
│  └─ 0002-firmware-cache.patch
├─ ros
│  └─ t265_odom_node.py
└─ scripts
   ├─ build_orangepi_offline.sh
   └─ deploy_orangepi_from_windows.py
```

## Why this repo exists

T265 on Linux arm64 is awkward in practice:

- newer `librealsense` versions are problematic for T265
- Orange Pi often has poor direct network access during build
- stock `librealsense` configure tries to fetch pybind11 and firmware online
- high-level `pipeline.start()` was unreliable in this environment

This repo preserves the working path used here:

- `librealsense 2.50.0`
- offline pybind11 source
- offline firmware cache
- low-level `Tracking Module` API

## Quick start from Windows

From the repository root:

```powershell
python .\scripts\deploy_orangepi_from_windows.py `
  --host 192.168.xxx.xxx `
  --user orangepi `
  --password orangepi
```

This will:

- reconstruct `librealsense-2.50.0-offline-arm64.tar.gz` from `bundle/*.part*`
- upload it to Orange Pi
- extract it under `/home/<user>/librealsense-2.50.0`
- run `build_orangepi_offline.sh`
- upload `ros/t265_odom_node.py` to `/home/<user>/t265_odom_node.py`

Then on Orange Pi:

```bash
source /opt/ros/noetic/setup.bash
python3 ~/t265_odom_node.py
```

Published topics:

- `/t265/odom/raw`
- `/t265/odom/filtered`

## Manual bundle reconstruction

If you need to reconstruct the tarball manually on Windows:

```powershell
Get-Content .\bundle\librealsense-2.50.0-offline-arm64.tar.gz.part* -AsByteStream |
  Set-Content .\librealsense-2.50.0-offline-arm64.tar.gz -AsByteStream
```

On Linux:

```bash
cat bundle/librealsense-2.50.0-offline-arm64.tar.gz.part* > librealsense-2.50.0-offline-arm64.tar.gz
```

## Manual Orange Pi build

If you prefer manual steps:

```bash
cd /home/orangepi
tar -xzf librealsense-2.50.0-offline-arm64.tar.gz
cd librealsense-2.50.0
bash build_orangepi_offline.sh
cp /path/to/t265_odom_node.py ~/t265_odom_node.py
```

Then:

```bash
source /opt/ros/noetic/setup.bash
python3 ~/t265_odom_node.py
```

## Source patches

Two source-level patches are preserved in this repo:

1. `patches/0001-python-offline-pybind11-and-init.patch`
   - use bundled pybind11 source
   - install Python package `__init__.py`

2. `patches/0002-firmware-cache.patch`
   - use cached firmware binaries during configure

## Notes

- This workflow targets `arm64 Ubuntu 20.04 / ROS Noetic / Python 3.8`.
- T265 may temporarily enumerate as `03e7:2150 Movidius`; the working state is `8087:0b37 Intel RealSense T265`.
- The node uses the low-level sensor API, not the high-level pipeline API.
- Current filter design is tuned for 2D vehicle use. `z` is intentionally conservative.

## License and attribution

If you publish this repo with the bundled source archive parts, keep Intel `librealsense` license and attribution rules in mind for redistributed source and patches.
