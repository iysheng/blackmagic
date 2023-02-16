#!/bin/sh


function do_command()
{
	echo $1
	case $1
		in
		h)  echo "host"; # 编译 PC 程序
            make PROBE_HOST=hosted ENABLE_DEBUG=1;;
		d) echo "download firmware"; # 下载固件
            sudo dfu-util -d 1d50:6018,:6017 -s 0x08002000:leave -D src/blackmagic.bin;; 
	esac
}


if [ $# -eq 1 ];then
do_command $1
else
	echo "do build native version"
    make ENABLE_DEBUG=1
fi


