obj-m += adg1414.o
obj-m += gpio-regmap.o

all: module dt
	echo Builded Device Tree Overlay and kernel module

module:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	
dt: adg1414_overlay.dts
	dtc -I dts -O dtb -o adg1414_overlay.dtbo adg1414_overlay.dts

clean:
	$(shell sudo rmmod adg1414)
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -rf
