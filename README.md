# Audio RTDM driver

Xenomai real-time audio driver for the Melbourne Instruments NINA/DELIA RPi Hat.

## Building

Note: The environment variable MELBINST_HAT must be set to either 0 (NINA) or 1 (DELIA) or the build will fail.
Have the kernel sources and an ARMv7 cross-compilation toolchain on your host machine then do:

```
$ export MELBINST_HAT=<0 for NINA, 1 for DELIA>
$ export KERNEL_PATH=<path to kernel source tree>
$ export CROSS_COMPILE=<arm compiler prefix>
$ make
```

It's possible to just use the Melbourne Instruments cross-compiling SDK, in which case you don't have to set the `CROSS_COMPILE` environment variable since the SDK will do it automatically.

As an alternative, you can build using the devshell option of Bitbake if you have all the Yocto layers ready on the host machine:

```
$ bitbake -c devshell virtual/kernel
$ export MELBINST_HAT=<0 for NINA, 1 for DELIA>
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

Copyright 2020-2024 Melbourne Instruments, Australia
