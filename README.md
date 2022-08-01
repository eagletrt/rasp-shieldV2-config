# Instructions

	git clone <THIS_INCREDIBLE_REPO>
	git submodule update --init

## CAN Interfaces

	cd rasp-setup

Install dependencies:

	sudo apt install device-tree-compiler
	sudo apt-get install can-utils

Compile mcp2515 overlays:

    cd mcp2515-overlays
	sudo dtc -@ -Hepapr -I dts -O dtb -o /boot/overlays/mcp2515-can0.dtbo mcp2515-can0-overlay.dts
	sudo dtc -@ -Hepapr -I dts -O dtb -o /boot/overlays/mcp2515-can1.dtbo mcp2515-can1-overlay.dts

Overwrite boot config:

	cd ..
	sudo cp config.txt /boot/config.txt

Copy and start CAN services:

	sudo cp startup_services/* /etc/systemd/system/
	sudo systemctl enable can0_startup.service
	sudo systemctl enable can1_startup.service
	sudo reboot

Check if CAN interfaces are working:

	ls /sys/bus/devices/spi0.0/net
	ls /sys/bus/devices/spi0.1/net

you should see "can0" and "can1" respecively.</br>
Anyway, `ifconfig -a` should show "can0" and "can1" interfaces.

NOTE:
- you can use `cansend` and `candump` commands to respectively send and dump data respectively to and from CAN bus.

## RTC Module
### Compiling MCP-795x module

Install requirements:

	sudo apt update && sudo apt upgrade
	sudo apt install build-essential linux-source libncurses5-dev
	uname -a #check kernel version
	sudo apt install raspberrypi-kernel-headers #MUST BE THE SAME VERSION AS THE KERNEL
	sudo rpi-update
	sudo reboot

Clone Linux kernel in order to compile modules:

	cd /usr/src/
	git clone https://github.com/raspberrypi/linux.git #MUST BE SAME VERSION OF LINUX HEADERS

Check if you have the version under /lib/modules/:

	ls /lib/modules/<your_kernel_version>

Copy last `.config` compilation file in new kernel headers:

	cp /usr/src/<previous_working_version>/.config /lib/modules/<your_kernel_version>/

Uncomment line `RTC_DRV_MCP795` with a text editor. </br>
Return to the repo folder and compile:

	cd mcp795x-module/rtc-tollsimy-mod/
	sudo make -C /lib/modules/<your_kernel_version>/build M=<path_to_the_repo>/mcp795x-module/rtc-tollsimy-mod/ modules

Copy compiled module to /lib/modules/<your_kernel_version> and load it:

	sudo cp rtc-mcp795.ko /lib/modules/<your_kernel_version>/kernel/drivers/rtc/
	sudo depmod
	sudo modprobe rtc-mcp795

Under /etc/modules-load.d/modules.conf add `rtc_mcp795` to get the module loaded at boot time.

### Compile device overlay

    cd ../../mcp795x-overlays
    sudo dtc -@ -Hepapr -I dts -O dtb -o /boot/overlays/rtc-mcp795-overlay.dtbo rtc-mcp795-overlay.dts
    sudo reboot

You should now see a device named "rtc0" under `/dev/` and `/dev/spidev1.0` should be gone.

### Disable NTP and other bad stuff

Disable fake-hwclock:

	sudo apt remove --purge  fake-hwclock
	sudo update-rc.d -f fake-hwclock remove
	sudo systemctl disable fake-hwclock

Open `/lib/udev/hwclock-set` with a text editor and comment these lines so at startup RTC timezone will be set correctly:

	if [ -e /run/systemd/system ] ; then
	exit 0
	fi
	
	/sbin/hwclock --rtc=$dev --systz	
	
In order to sync system time with RTC time at startup modify this line from this:
	
	/sbin/hwclock --rtc=$dev --hctosys

to this:

	busybox hwclock --rtc=$dev --hctosys

Disable NTC sync:
	
	sudo systemctl stop ntp.service
	sudo systemctl disable ntp.service
	sudo systemctl stop systemd-timesyncd.service
	sudo systmectl disable systemd-timesyncd.service

### Startup services (check if needed)

In order to use a service that synchronize system time with HW clock time at startup do:

Move the content of the folder startup_services in /etc/systemd/system/ :

	cd ..
	sudo cp startup_services/* /etc/systemd/system/
	cd /etc/systemd/system/
    sudo systemctl enable rtc-date-startup.service
    sudo reboot

now, with this service enabled, system clock time will be updated with RTC time at every startup.

Then, add a crontab entry that sync time every 5 minutes with an internet pool if it is reachable:
	
	sudo chrontab -e

then add the following line:
	
	*/5 * * * * sudo ntpdate it.pool.ntp.org && sudo busybox hwclock -f /dev/rtc0 -w && sudo /bin/bash -c 'echo "Date set from internet" > /dev/kmsg' 

NOTES: 
- If backup battery dies, you just have to replace it and restart the Rasp with internet connection on.

- To retreive the current data always use System Time (e.g. with `date` command), because it's the only one always synchronized (with internet or with HW clock).

###  How to manually synchronize time
In order to synchronize Hardware clock time (RTC time) with system time do:
	
	sudo busybox hwclock --systohc -f /dev/rtc0

If instead you want to synchronize System clock Time with RTC Hardware Clock time do:

	sudo busybox hwclock --hctosys -f /dev/rtc0

Print date and time to see if system time is updated:

	date

or better:

	timedatectl

NOTE: It's mandatory to use the busybox hwclock, otherwise hwclock command will fail. (probably because of interrupt pin not routed -> need also modification of overlay and boot config file).

## NRF

Clone and build the NRF24 library:

	cd <path_to_the_repo>/rf24libs/RF24/
	sudo apt install pigpio
	./configure --driver=SPIDEV
	make
	sudo make install
	
Build and start the main:

	cd ../../nrf-main/
	g++ nrf-main.cpp -l rf24 -o nrf-main
	sudo ./nrf-main ARG1 ARG2 (see source code)

NOTE: the program MUST run with superuser permissions in order to take control of the SPI interface.
