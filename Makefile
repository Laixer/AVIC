KDIR ?= /lib/modules/`uname -r`/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	 $(MAKE) -C $(KDIR) M=$(PWD) modules_install

dkms:
	mkdir /usr/src/avic-can-1.0.0/
	cp -r . /usr/src/avic-can-1.0.0/
	dkms add -m avic-can/1.0.0
	dkms build -m avic-can/1.0.0
	dkms install -m avic-can/1.0.0
