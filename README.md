# ble_led_cube
BLE cube for controlling BLE LED dimmers. Rotating the cube, a compatible BLE LED dimmer changes LED colour accordingly: each face corresponds to a configured RGBW PWM values group that it is advertised once the cube is moved and put on a new face.
Refer to doc.txt for more details.


**Install**

Download Segger JLink tool from https://www.segger.com/jlink-software.html. Unpack it and move it to /opt directory.
Download the Nordic SDK from http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v11.x.x/. Unpack it and move it to /opt directory.

Clone this repo in your projects directory:

    $ git clone https://github.com/marcorussi/ble_led_cube.git
    $ cd ble_led_cube
    $ gedit Makefile

Verify and modify following names and paths as required according to your ARM GCC toolchain:

```
PROJECT_NAME := ble_cube
NRFJPROG_PATH := ./tools
SDK_PATH := /opt/nRF5_SDK_11.0.0_89a8197
LINKER_SCRIPT := ble_cube_gcc_nrf51.ld
GNU_INSTALL_ROOT := /home/marco/ARMToolchain/gcc-arm-none-eabi-4_9-2015q2
GNU_VERSION := 4.9.3
GNU_PREFIX := arm-none-eabi
```

**Flash**

Connect your nrf51 Dev. Kit, make and flash it:
 
    $ make
    $ make flash_softdevice (for the first time only)
    $ make flash

You can erase the whole flash memory by running:

    $ make erase


**DFU Upgrade**

For creating a .zip packet for DFU upgrade run the following command:

nrfutil dfu genpkg dimmer.zip --application ble_cube_s130.hex --application-version 0xffffffff --dev-revision 0xffff --dev-type 0xffff --sd-req 0xfffe




