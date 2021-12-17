# AVIC Bridge driver

The AVIC Bridge is used to communicate with the AVIC peripheral firmware. It supports an AVIC firmware controller and a CAN FD implementation for Linux SocketCAN.

## Build

Clone the source code and build the kernel module. If all the dependencies are installed then `make` will build the module.

## Usage

If the module is not installed then load the module from disk:

```sh
sudo modprobe can-dev
sudo insmod ./avic_can.ko
```

Otherwise:

```sh
sudo modprobe can-dev
sudo modprobe avic_can.ko
```

The Linux kernel will now show a CAN network interface. See `ip link`. To enable the CAN interface device you must set the bitrate and bring the interface up:

```sh
sudo ip link set canX up type can bitrate 500000
```

See the `can-utils` for testing and diagnosis.
