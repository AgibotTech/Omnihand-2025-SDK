English | [中文](README.zh_CN.md)

# OmniHand 2025 SDK

## Overview

OmniHand 2025 is a compact, high-DOF interactive dexterous hand featuring `10 active + 6 passive degrees of freedom`. Weighing only 500g, it utilizes CANFD communication interfaces and is equipped with `400+ tactile points and 0.1N array resolution, with maximum fingertip force of 5N`. It's suitable for various humanoid robots and robotic arms. Its compact, lightweight design and rich tactile interaction capabilities make it valuable for interactive services, research, education, and light-duty operations.

To facilitate rapid development and application, we provide the OmniHand Agile 2025 SDK development package, supporting both Python and C++ API interfaces for quick implementation of dexterous hand control and data acquisition functions.

![](document/pic/hand.jpg)

## Getting Started

### System Requirements

#### Hardware Requirements

Supports both CANFD communication interfaces

- CANFD: Currently supports ZLG USBCANFD series (Recommended: USBCANFD-100U-mini/USBCANFD-100U/USBCANFD-200U)

#### Software Requirements

- Operating System: Ubuntu 22.04 (x86_64)
- Compiler: gcc 11.4 or higher
- Build Tool: CMake 3.24 or higher
- Python: 3.10 or higher

### Installation

Choose between source compilation or pre-compiled package installation.

#### Source Compilation Installation

Execute the following commands in the project root directory:

```bash
./build.sh -DCMAKE_BUILD_TYPE=Release \
           -DCMAKE_INSTALL_PREFIX=./build/install \
           -DBUILD_PYTHON_BINDING=ON \
           -DBUILD_CPP_EXAMPLES=OFF \
```

The DBUILD_PYTHON_BINDING option is for building Python binding modules, and DBUILD_CPP_EXAMPLES is for building C++ example code.

#### Pre-compiled Package Installation

##### Python wheel Package Installation

```bash
# Download the corresponding version of python wheel package from GitHub
pip install ./omnihand_2025_py-0.8.0-cp310-cp310-linux_x86_64.whl
```

## Dexterous Hand Motor Index

OmniHand 2025 has 10 degrees of freedom, indexed from 1 to 10. The corresponding control motors are shown in the following image:

![](document/pic/hand_motors.jpg)

## Running Examples

```bash
cd python/example
python3 ./demo_set_motor.py
```

## Directory Structure

```bash
.
├── assets # Model files
├── build.sh # Build script
├── cmake # CMake modules directory
├── CMakeLists.txt # Main CMake configuration file
├── document # Documentation directory
├── examples # C++ example code
├── python # Python binding module (Python interface generated from C++ source)
├── src
│ ├── c_agibot_hand_base.cc
│ ├── c_agibot_hand_base.h
│ ├── can_bus_device
│ │ ├── socket_can
│ │ └── zlg_usb_canfd
│ ├── CMakeLists.txt
│ ├── export_symbols.h
│ ├── implementation
│ │ ├── c_agibot_hand_can
│ │ └── c_agibot_hand_rs
│ ├── kinematics_solver
│ ├── proto.h
│ └── rs_485_device
└── thirdParty # Third-party dependency libraries

```

## API Documentation

For detailed API usage instructions, please refer to the following links:

- [OmniHand 2025 SDK C++ API Documentation](document/en/API_CPP.md)
- [OmniHand 2025 SDK Python API Documentation](document/en/API_PYTHON.md)

## FAQ

### Q1: Using can driver, unable to communicate with the hand when starting the program?

**A:** First, ensure the driver is correctly installed. See [ZLG Driver Installation Guide](https://manual.zlg.cn/web/#/42/1710) for details. Make sure the hand's power is connected and the USB is connected to the computer, then execute:

```shell
lsusb
sudo chmod 666 /dev/bus/usb/xxx/yyy
```

### Q2: Using serial port driver, unable to communicate with the hand when starting the program?

**A:** You need to grant read and write permissions to the corresponding serial port. Execute the following commands:
Apply
Note: The first command lists all USB serial ports, and the second command grants read and write permissions to the specified port (e.g. /dev/ttyUSB0). Make sure to use the correct port number if it's different on your system.

```shell
ll /dev/ttyUSB*

sudo chmod a+rw /dev/ttyUSB0
```

### Q3: Python packaging errors during source compilation?

**A:** Check if the following dependencies are installed:

```shell
sudo apt install python3.10-dev
pip3 install build setuptools wheel
```

## Copyright Notice

Copyright (c) 2025 Agibot Co., Ltd. OmniHand 2025 SDK is licensed under Mulan PSL v2.

_Document Version: v0.8.0_  
_Last Updated: September 2025_
