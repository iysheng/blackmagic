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

#ifndef PLATFORMS_COMMON_USB_H
#define PLATFORMS_COMMON_USB_H

#include <stdint.h>
#include <libopencm3/usb/usbd.h>

extern usbd_device *usbdev;
extern uint16_t usb_config;

#if defined(USB_HS)
#define CDCACM_PACKET_SIZE  512U
#define TRACE_ENDPOINT_SIZE 512U
#else
/* 针对 native 版本没有定义 USB_HS,只会走到这里 */
#define CDCACM_PACKET_SIZE  64U
#define TRACE_ENDPOINT_SIZE 64U
#endif

#if !defined(USB_MAX_INTERVAL)
#define USB_MAX_INTERVAL 255U
#endif

/* 这些是 ENDPOINT 编号不是随意指定的和下面的
 * CDCACM_GDB_ENDPOINT 对应的是 gdb_data_endp 的地址, bEndpointAddress
 * UART_IF_NO + 1 对应的是 uart_data_endp 的地址, bEndpointAddress
 * TRACE_IF_NO 有关系
 * */
#define CDCACM_GDB_ENDPOINT  1U
#define CDCACM_UART_ENDPOINT 3U
#define TRACE_ENDPOINT       5U

#define GDB_IF_NO  0U
#define UART_IF_NO 2U
#define DFU_IF_NO  4U
/* native 平台开启了这个宏,所以总的 interface 数量是 6 */
#ifdef PLATFORM_HAS_TRACESWO
#define TRACE_IF_NO      5U
#define TOTAL_INTERFACES 6U
#else
#define TOTAL_INTERFACES 5U
#endif

void blackmagic_usb_init(void);

/* Returns current usb configuration, or 0 if not configured. */
uint16_t usb_get_config(void);

#endif /* PLATFORMS_COMMON_USB_H */
