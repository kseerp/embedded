obj-m += ltc2664.o

all: module dt
	echo Builded Device Tree Overlay and kernel module

module:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	
dt: ltc2664_overlay.dts
	dtc -I dts -O dtb -o ltc2664_overlay.dtbo ltc2664_overlay.dts

clean:
	$(shell sudo rmmod ltc2664)
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -rf
