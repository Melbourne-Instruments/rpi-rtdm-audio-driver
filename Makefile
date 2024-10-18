ccflags-y += -DMELBINST_HAT=$(MELBINST_HAT)
ccflags-y += -DSIMULATE_MELBINST_HAT=$(SIMULATE_MELBINST_HAT)
ifeq ($(MELBINST_HAT),0)
obj-m += bcm2835-spi-nina.o
else
obj-m += bcm2835-spi-delia.o
endif
obj-m += audio-rtdm.o
ifeq ($(MELBINST_HAT),0)
bcm2835-spi-nina-objs := bcm2835-spi-melbinst.o
else
bcm2835-spi-delia-objs := bcm2835-spi-melbinst.o
endif

all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=${CROSS_COMPILE} -C $(KERNEL_PATH)  M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNEL_PATH) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_PATH) M=$(PWD) clean
	@rm -f *.o
	@rm -f *.o.*
