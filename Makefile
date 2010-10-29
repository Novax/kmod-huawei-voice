#
# Makefile for the USB serial device drivers.
#

# Object file lists.

obj-m			+= option.o
obj-m			+= usb_wwan.o
obj-m			+= huawei_voice.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
