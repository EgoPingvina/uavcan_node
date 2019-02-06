# uavcan_node

Controller based on STM32F103 for:
1. coaxial rotor(duplicate signal for both motors)
2. actuators

## Getting Started(for Windows)

* libuavcan preparing:

1. install px4 toolchain for windows (on cygwin)

documentation: https://dev.px4.io/en/setup/dev_env_windows_cygwin.html

download from: https://github.com/PX4/windows-toolchain/releases

2. build uavcan protocol sources by dsdl-compiler

run /PX4/run-console.bat

go to repo parent folder

invoke 
```
libuavcan/libuavcan/dsdl_compiler/libuavcan_dsdlc /cygdrive/*path to parent folder*/libuavcan/dsdl/uavcan -Odsdlc_g -vvv
```
* Environment preparing
1. install Visual Studio
2. install VisualGDB 5.3r8

## Usage
1. open Header files/Startup/Config.hpp in Solution Explorer or uavcan_node\uavcan_node\Inc\Confog.hpp in explorer
2. set value to CONTROLLER from Controller info region
3. set value to deviceId from Servos/ESC info region
4. build project
5. click Debug->Program and start without debugging
