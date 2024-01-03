#!/bin/sh
#

function env_init
{
export TI_SDK_PATH=/home/red/Projects/bbb_debuger/tiiiiiiiiiiii
export LINUX_DEVKIT_PATH=${TI_SDK_PATH}/linux-devkit
export SDK_PATH_TARGET=${LINUX_DEVKIT_PATH}/sysroots/armv7at2hf-neon-oe-linux-gnueabi/
export CROSS_COMPILE=${TI_SDK_PATH}/external-toolchain-dir/arm-gnu-toolchain-11.3.rel1-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-
export CC="${CROSS_COMPILE}gcc --sysroot=${SDK_PATH_TARGET}"
export INSTALL_MOD_STRIP=1
export DESTDIR=install_ko
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
