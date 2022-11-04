# AVIC Bridge driver

The Advanced Vehicle InterConnect Bridge (AVIC) driver supports dedicated comunication over USB with an AVIC peripheral. The AVIC protocol offers CAN FD network for Linux SocketCAN over USB, direct vehicle commands and various monitoring systems.

## Build

Make sure kernel headers are installed. On Debian based systems:

`sudo apt install linux-headers-$(uname -r)`

If all the dependencies are installed then `make` will build the module.

## DKMS Support

_This is the recommended method for installing the AVIC module._

To keep the module up-to-date with the latest kernel the DKMS can be used with the AVIC module.
Make sure DKMS is installed on the system. Run from the source directory of the module:

```sudo make install_dkms```

The module will be loaded when hardware is plugged into the system.

## Usage

If the module is not installed then load the module from path:

```sh
sudo modprobe can-dev
sudo insmod ./avic_can.ko
```

If the module was installed after build, modprobe should be able to locate the module:

```sh
sudo modprobe can-dev
sudo modprobe avic_can.ko
```

If the module is loaded and the hardware was detected the Linux kernel will show a CAN network interface. To enable the CAN interface device you must set the bitrate and bring the interface up:

```sh
sudo ip link set canX up type can bitrate 500000
```

For further information on CAN network interfaces see `ip link`.

Use the `can-utils` for testing and diagnosis.
