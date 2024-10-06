# Bluetooth: Smart LED Demo

Application demonstrating the BLE Peripheral role that exposes GATT services.

Controller app: https://github.com/simicity/SmartLedControllerApp


## Dependencies

- [Zephyr](https://zephyrproject.org/)
- nRF5340 DK by Nordic Semiconductor
- nRF Connect SDK v2.6.1 by Nordic Semiconductor
- [Nanopb](https://github.com/nanopb/nanopb) (protobuf)

## How to build and flash

To build:
```
west build -b nrf5340dk_nrf5340_cpuapp
```

To clean-build:
```
west build -b nrf5340dk_nrf5340_cpuapp --pristine
```

To flash
```
west flash
```