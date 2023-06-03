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

#ifndef PLATFORMS_COMMON_USB_DESCRIPTORS_H
#define PLATFORMS_COMMON_USB_DESCRIPTORS_H

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/usb/dfu.h>

#include "usb.h"
#include "serialno.h"
#include "version.h"
#include "usb_types.h"

#define BOARD_IDENT "Black Magic Probe " PLATFORM_IDENT FIRMWARE_VERSION

/* Top-level device descriptor */
/* 顶层的设备描述符 */
static const usb_device_descriptor_s dev_desc = {
	/* 设备描述符的长度固定为 18 */
	.bLength = USB_DT_DEVICE_SIZE,
	/* 设备描述符的类型固定为 1 */
	.bDescriptorType = USB_DT_DEVICE,
	/* USB 规范版本号 */
	.bcdUSB = 0x0200,
	/* 类代码,杂项设备 */
	.bDeviceClass = 0xef, /* Miscellaneous Device */
	/* 子类代码 */
	.bDeviceSubClass = 2, /* Common Class */
	/* 协议代码 */
	.bDeviceProtocol = 1, /* Interface Association */
#if defined(LM4F) || defined(USB_HS)
	/* The USB specification requires that the control endpoint size for high
	 * speed devices (e.g., stlinkv3) is 64 bytes.
	 * On the LM4F is is required to be 64 bytes for the ICDI driver. */
	.bMaxPacketSize0 = 64,
#else
	/* 端点 0 支持最大数据包长度 */
	.bMaxPacketSize0 = 32,
#endif
	/* 厂商产品设备ID */
	.idVendor = 0x1d50,
	.idProduct = 0x6018,
	.bcdDevice = 0x0100,
	/* 供应商字符串描述符索引 */
	.iManufacturer = 1,
	/* 产品字符串描述符索引 */
	.iProduct = 2,
	/* 设备序列号字符串索引 */
	.iSerialNumber = 3,
	/* 可能的配置描述符的数量 */
	.bNumConfigurations = 1,
};

/* GDB interface descriptors */

/* This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux cdc_acm
 * driver. */
static const usb_endpoint_descriptor_s gdb_comm_endp = {
	/* endpoint 描述符长度为 7 */
	.bLength = USB_DT_ENDPOINT_SIZE,
	/* endpoint 描述符类型 5 */
	.bDescriptorType = USB_DT_ENDPOINT,
	/* IN 类型的描述符 */
	.bEndpointAddress = (CDCACM_GDB_ENDPOINT + 1U) | USB_REQ_TYPE_IN,
	/* 该端点的属性:这里表示 中断传输 */
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	/* 该端点支持的最大包长 */
	.wMaxPacketSize = 16,
	/* 端点的查询时间 */
	.bInterval = USB_MAX_INTERVAL,
};

/* gdb 的数据 endpoint 实体,分别对应 IN 和 OUT */
static const usb_endpoint_descriptor_s gdb_data_endp[] = {
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = CDCACM_GDB_ENDPOINT,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = CDCACM_PACKET_SIZE,
		.bInterval = 1,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = CDCACM_GDB_ENDPOINT | USB_REQ_TYPE_IN,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = CDCACM_PACKET_SIZE,
		.bInterval = 1,
	},
};

static const struct {
	usb_cdc_header_descriptor_s header;
	usb_cdc_call_management_descriptor_s call_mgmt;
	usb_cdc_acm_descriptor_s acm;
	usb_cdc_union_descriptor_s cdc_union;
} __attribute__((packed)) gdb_cdcacm_functional_descriptors = {
	.header =
		{
			.bFunctionLength = sizeof(usb_cdc_header_descriptor_s),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
			.bcdCDC = 0x0110,
		},
	.call_mgmt =
		{
			.bFunctionLength = sizeof(usb_cdc_call_management_descriptor_s),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
			.bmCapabilities = 0,
			/* 数据接口的地址对应的是 gdb_data_iface 的地址 */
			.bDataInterface = GDB_IF_NO + 1U,
		},
	.acm =
		{
			.bFunctionLength = sizeof(usb_cdc_acm_descriptor_s),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_ACM,
			.bmCapabilities = 2, /* SET_LINE_CODING supported */
		},
	.cdc_union =
		{
			.bFunctionLength = sizeof(usb_cdc_union_descriptor_s),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_UNION,
			/* 控制接口的 interface */
			.bControlInterface = GDB_IF_NO,
			.bSubordinateInterface0 = GDB_IF_NO + 1U,
		},
};

/*
 * usb 接口描述符 gdb_command_interface 实体
 * */
static const usb_interface_descriptor_s gdb_comm_iface = {
	/* 描述符的长度也是 9 字节 */
	.bLength = USB_DT_INTERFACE_SIZE,
	/* 描述符的类型也是 4 */
	.bDescriptorType = USB_DT_INTERFACE,
	/* 接口号,这个值和接口关联描述符(0X0B)中的 bFirstInterface 对应 */
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	/* 端点 0 以外的端点数 */
	.bNumEndpoints = 1,
	/* 类代码 */
	.bInterfaceClass = USB_CLASS_CDC,
	/* 子类代码 */
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	/* 协议代码 */
	.bInterfaceProtocol = USB_CDC_PROTOCOL_NONE,
	/* 字符串描述符的索引值 */
	.iInterface = 4,

	/* 关联的 endpoint */
	.endpoint = &gdb_comm_endp,

	/* 额外数据和额外数据长度 */
	.extra = &gdb_cdcacm_functional_descriptors,
	.extralen = sizeof(gdb_cdcacm_functional_descriptors),
};

static const usb_interface_descriptor_s gdb_data_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = GDB_IF_NO + 1U,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = gdb_data_endp,
};

/*
 * USB 接口关联描述符
 * */
static const usb_iface_assoc_descriptor_s gdb_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	/* 关联接口的起始接口号,这个号在接口描述符的 bInterfaceNumber 中定义 */
	.bFirstInterface = GDB_IF_NO,
	/* 与该功能关联的连续接口的数量 */
	.bInterfaceCount = 2,
	.bFunctionClass = USB_CLASS_CDC,
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol = USB_CDC_PROTOCOL_NONE,
	/* 字符串描述符索引 */
	.iFunction = 4,
};

/* Physical/debug UART interface */
/* 调试串口 command 对应的 endpoint 描述符
 * 这个 ep 只有 IN 方向
 * */
static const usb_endpoint_descriptor_s uart_comm_endp = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = (CDCACM_UART_ENDPOINT + 1U) | USB_REQ_TYPE_IN,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = USB_MAX_INTERVAL,
};

static const usb_endpoint_descriptor_s uart_data_endp[] = {
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = CDCACM_UART_ENDPOINT,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
#if defined(USB_HS)
		.wMaxPacketSize = CDCACM_PACKET_SIZE,
#else
		.wMaxPacketSize = CDCACM_PACKET_SIZE / 2U,
#endif
		.bInterval = 1,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = CDCACM_UART_ENDPOINT | USB_REQ_TYPE_IN,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = CDCACM_PACKET_SIZE,
		.bInterval = 1,
	},
};

/* 这个是和 cdc 有关的 interface 描述符中的额外信息 */
static const struct {
	usb_cdc_header_descriptor_s header;
	usb_cdc_call_management_descriptor_s call_mgmt;
	usb_cdc_acm_descriptor_s acm;
	usb_cdc_union_descriptor_s cdc_union;
} __attribute__((packed)) uart_cdcacm_functional_descriptors = {
	.header =
		{
			.bFunctionLength = sizeof(usb_cdc_header_descriptor_s),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
			.bcdCDC = 0x0110,
		},
	.call_mgmt =
		{
			.bFunctionLength = sizeof(usb_cdc_call_management_descriptor_s),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
			.bmCapabilities = 0,
			.bDataInterface = UART_IF_NO + 1U,
		},
	.acm =
		{
			.bFunctionLength = sizeof(usb_cdc_acm_descriptor_s),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_ACM,
			.bmCapabilities = 2, /* SET_LINE_CODING supported*/
		},
	.cdc_union =
		{
			.bFunctionLength = sizeof(usb_cdc_union_descriptor_s),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_UNION,
			.bControlInterface = UART_IF_NO,
			.bSubordinateInterface0 = UART_IF_NO + 1U,
		},
};

static const usb_interface_descriptor_s uart_comm_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = UART_IF_NO,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_NONE,
	.iInterface = 5,

	/* 针对这些控制端口,就有一些额外的属性信息 */
	.endpoint = &uart_comm_endp,

	/* 针对这些控制端口,就有一些额外的属性信息 */
	.extra = &uart_cdcacm_functional_descriptors,
	.extralen = sizeof(uart_cdcacm_functional_descriptors),
};

static const usb_interface_descriptor_s uart_data_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = UART_IF_NO + 1U,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = uart_data_endp,
};

static const usb_iface_assoc_descriptor_s uart_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = UART_IF_NO,
	.bInterfaceCount = 2,
	.bFunctionClass = USB_CLASS_CDC,
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol = USB_CDC_PROTOCOL_NONE,
	.iFunction = 5,
};

/* DFU interface */

const usb_dfu_descriptor_s dfu_function = {
	.bLength = sizeof(usb_dfu_descriptor_s),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = 1024,
	.bcdDFUVersion = 0x011a,
};

const usb_interface_descriptor_s dfu_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = DFU_IF_NO,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xfe,
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 1,
	.iInterface = 6,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

static const usb_iface_assoc_descriptor_s dfu_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = DFU_IF_NO,
	.bInterfaceCount = 1,
	.bFunctionClass = 0xfe,
	.bFunctionSubClass = 1,
	.bFunctionProtocol = 1,
	.iFunction = 6,
};

/* Trace/SWO interface */

#ifdef PLATFORM_HAS_TRACESWO
static const usb_endpoint_descriptor_s trace_endp = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = TRACE_ENDPOINT | USB_REQ_TYPE_IN,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = TRACE_ENDPOINT_SIZE,
	.bInterval = 0,
};

const usb_interface_descriptor_s trace_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = TRACE_IF_NO,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = 0xff,
	.bInterfaceSubClass = 0xff,
	.bInterfaceProtocol = 0xff,
	.iInterface = 7,

	.endpoint = &trace_endp,
};

static const usb_iface_assoc_descriptor_s trace_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = TRACE_IF_NO,
	.bInterfaceCount = 1,
	.bFunctionClass = 0xff,
	.bFunctionSubClass = 0xff,
	.bFunctionProtocol = 0xff,
	.iFunction = 7,
};
#endif

/* Interface and configuration descriptors */
/* 和配置描述符关联的接口描述符,根据是否开启 PLATFORM_HAS_TRACESWO,这个长度是 5 或者 6 */
static const usb_interface_s ifaces[] = {
	{
		.num_altsetting = 1,
		/* 关联 gdb comand,通过这个关联描述符将 gdb command 和 gdb data 关联起来 */
		.iface_assoc = &gdb_assoc,
		/* gdb command 接口描述符 */
		.altsetting = &gdb_comm_iface,
	},
	{
		.num_altsetting = 1,
		/* 关联 gdb data */
		.altsetting = &gdb_data_iface,
	},
	{
		.num_altsetting = 1,
		/* 关联串口 */
		.iface_assoc = &uart_assoc,
		.altsetting = &uart_comm_iface,
	},
	{
		.num_altsetting = 1,
		.altsetting = &uart_data_iface,
	},
	{
		.num_altsetting = 1,
		/* 和 dfu 升级有关的 关联描述符 */
		.iface_assoc = &dfu_assoc,
		.altsetting = &dfu_iface,
	},
#if defined(PLATFORM_HAS_TRACESWO)
	{
		.num_altsetting = 1,
		/* 根据需要关联 traceswo */
		.iface_assoc = &trace_assoc,
		.altsetting = &trace_iface,
	},
#endif
};

static const usb_config_descriptor_s config = {
	/* 配置描述符的长度固定是 9 */
	.bLength = USB_DT_CONFIGURATION_SIZE,
	/* 配置描述符的类型固定是 2 */
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	/*
	 * 如果开启 TRACE 那么 interfaces 值是 6, 否则是 5
	 * */
	.bNumInterfaces = TOTAL_INTERFACES,
	.bConfigurationValue = 1,
	/* 字符串描述符的索引值 */
	.iConfiguration = 0,
	/* 配置特性
	 * 第 7 bit 必须为 1
	 * 第 6 bit 表示 self-powered
	 * 第 5 bit 表示支持远程唤醒
	 * 第 4 bit 表示电池供电
	 * */
	.bmAttributes = 0x80,
	/* 所需的最大总线电流是 250ma */
	.bMaxPower = 250,

	.interface = ifaces,
};

/* 字符串描述符集合 */
static const char *const usb_strings[] = {
	"Black Magic Debug", /* 不知道为什么,感觉这里对应的描述符中的 1,依次类推 */
	BOARD_IDENT,
	serial_no,
	"Black Magic GDB Server",
	"Black Magic UART Port",
	"Black Magic DFU",
#if defined(PLATFORM_HAS_TRACESWO)
	"Black Magic Trace Capture",
#endif
};

#endif /* PLATFORMS_COMMON_USB_DESCRIPTORS_H */
