/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2022 1BitSquared <info@1bitsquared.com>
 * Written by Rachel Mant <git@dragonmux.network>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/cm3/nvic.h>

#include "general.h"
#include "usb.h"
#include "usb_descriptors.h"
#include "usb_serial.h"
#include "usb_dfu_stub.h"
#include "serialno.h"

/* 这个是传进去的 usb 设备结构体指针 */
usbd_device *usbdev = NULL;
uint16_t usb_config;

/* We need a special large control buffer for this device: */
/* 大的控制缓冲区 */
static uint8_t usbd_control_buffer[256];

/*
 * usb 接口初始化,这里很关键
 * */
void blackmagic_usb_init(void)
{
	/* 更新序列号 */
	read_serial_number();

	/* 初始化这个 usbd 设备 */
	usbdev = usbd_init(&USB_DRIVER, &dev_desc, &config, usb_strings, sizeof(usb_strings) / sizeof(char *),
		usbd_control_buffer, sizeof(usbd_control_buffer));

	/*
	 * 给这个 config 注册两个回调函数
	 * */
	usbd_register_set_config_callback(usbdev, usb_serial_set_config);
	usbd_register_set_config_callback(usbdev, dfu_set_config);

	/*
	 * 设置 usb 中断的优先级
	 * */
	nvic_set_priority(USB_IRQ, IRQ_PRI_USB);
	nvic_enable_irq(USB_IRQ);
}

uint16_t usb_get_config(void)
{
	return usb_config;
}

/* USB 中断处理函数, 这个中断处理函数是 USB 通信的入口
 * 针对 native 平台宏展开后是函数 usb_lp_can_rx0_isr
 * 这个中断函数在 libopencm3 中定义
 * */
void USB_ISR(void)
{
	usbd_poll(usbdev);
}
