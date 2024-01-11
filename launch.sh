#!/bin/sh
#

function env_init
{
export CROSS_COMPILE=/home/red/.local/x-tools/armv6-rpi-linux-gnueabihf/bin/armv6-rpi-linux-gnueabihf-
export CC="${CROSS_COMPILE}gcc"
}

if [ $# -gt 0 ];then
	echo $1
	case $1 in
		e) env_init;;
		t) make ARCH=arm CROSS_COMPILE=${CROSS_COMPILE} dtbs -j10;;
		k) make ARCH=arm CROSS_COMPILE=${CROSS_COMPILE} modules -j10;;
		cscope) make ARCH=arm CROSS_COMPILE=${CROSS_COMPILE} cscope -j10;;
		ik) make ARCH=arm CROSS_COMPILE=${CROSS_COMPILE} INSTALL_MOD_PATH=${DESTDIR} INSTALL_MOD_STRIP=${INSTALL_MOD_STRIP} modules_install -j10;;
		m) echo "menuconfig";make ARCH=arm CROSS_COMPILE=${CROSS_COMPILE} menuconfig;;
		*) echo "oh no";;
	esac
else
	echo "just build"
	make ARCH=arm CROSS_COMPILE=${CROSS_COMPILE} -j10
fi
