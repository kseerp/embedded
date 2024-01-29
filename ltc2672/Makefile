obj-m += ltc2672.o

all: module dt
	echo Builded Device Tree Overlay and kernel module

module:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	
dt: ltc2672_overlay.dts
	dtc -I dts -O dtb -o ltc2672_overlay.dtbo ltc2672_overlay.dts

clean:
	$(shell sudo rmmod ltc2672)
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -rf
