ccflags-y += -DSIMULATE_NINA_HAT=$(SIMULATE_NINA_HAT)
obj-m += bcm2835-spi-nina.o
obj-m += audio-rtdm.o

all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=${CROSS_COMPILE} -C $(KERNEL_PATH)  M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNEL_PATH) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_PATH) M=$(PWD) clean
	@rm -f *.o
	@rm -f *.o.*
