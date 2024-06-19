# Enable these flags when compiling against a GCOV-enabled kernel
# loadable objects are not compatible with different configured kernels (i.e. gcov) 
# GCOV_PROFILE := y
# CFLAGS +=-ftest-coverage -fprofile-arcs
# export CFLAGS

LINUX_SRC=/lib/modules/$(shell uname -r)/build

ifneq ($(KERNELRELEASE),)
#kbuild part of Makefile
include Kbuild
else
# Normal Makefile - The device overlay is compiled outside of the makefile -
# Usually by a bash script running the makefile
all:
	$(MAKE) -C $(LINUX_SRC) M=$$PWD modules
	dtc -@ -I dts -O dtb -o ./arch/arm/boot/dts/tmf8806-overlay.dtbo ./arch/arm/boot/dts/tmf8806-overlay.dts
	dtc -@ -I dts -O dtb -o ./arch/arm/boot/dts/tmf8806-overlay-polled.dtbo ./arch/arm/boot/dts/tmf8806-overlay-polled.dts
	dtc -@ -I dts -O dtb -o ./arch/arm/boot/dts/tmf8806-overlay-fpc.dtbo ./arch/arm/boot/dts/tmf8806-overlay-fpc.dts
	dtc -@ -I dts -O dtb -o ./arch/arm/boot/dts/tmf8806-overlay-polled-fpc.dtbo ./arch/arm/boot/dts/tmf8806-overlay-polled-fpc.dts


modules:
	$(MAKE) -C $(LINUX_SRC) M=$$PWD $@

clean:
	$(MAKE) -C $(LINUX_SRC) M=$$PWD clean
	$(RM) ./arch/arm/boot/dts/*.dtbo
	$(RM) .tmp*.gcno *.gcov *.gz coverage.info
	$(RM) -rf html sys 
endif
