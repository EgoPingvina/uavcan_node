# twin-engine-controller

Controller based on STM32F103 for coaxial rotor(duplicate signal for both motors)

## Getting Started

* libuavcan preparing(windows):

1. install cygwin

documentation: https://dev.px4.io/en/setup/dev_env_windows_cygwin.html

download from: https://github.com/PX4/windows-toolchain/releases

2. build

run /PX4/run-console.bat

go to libuavcan parent folder

invoke 
```
libuavcan/libuavcan/dsdl_compiler/libuavcan_dsdlc /cygdrive/*path to parent folder*/libuavcan/dsdl/uavcan -Odsdlc_g -vvv
```
