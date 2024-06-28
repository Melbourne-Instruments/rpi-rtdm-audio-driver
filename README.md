# Audio RTDM driver

Xenomai real-time audio driver for the Melbourne Instruments NINA RPi Hat.

## Building

Have the kernel sources and an ARMv7 cross-compilation toolchain on your host machine then do:

```
$ export KERNEL_PATH=<path to kernel source tree>
$ export CROSS_COMPILE=<arm compiler prefix>
$ make
```

It's possible to just use the official [Elk Audio OS cross-compiling SDK](https://github.com/elk-audio/elkpi-sdk), in which case you don't have to set the `CROSS_COMPILE` environment variable since the SDK will do it automatically.

As an alternative, you can build using the devshell option of Bitbake if you have all the Yocto layers ready on the host machine:

```
$ bitbake -c devshell virtual/kernel
$ export KERNEL_PATH=<path to kernel source in your bitbake tmp build files>
$ make
```

## Usage Example
To load the driver as an out-of-tree module, run as sudo:

```
$ insmod bcm2835-spi-nina.ko
$ insmod audio_rtdm.ko audio_buffer_size=128
```

If the modules are installed already as part of the Kernel you can just do instead:

```
 $ modprobe audio_rtdm audio_buffer_size=128
```

Copyright 2020-2022 Melbourne Instruments, Australia
