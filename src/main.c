/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
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

/* Provides main entry point. Initialise subsystems and enter GDB protocol loop. */

#include "general.h"
#include "platform.h"
#include "gdb_if.h"
#include "gdb_main.h"
#include "target.h"
#include "exception.h"
#include "gdb_packet.h"
#include "morse.h"
#include "command.h"
#ifdef ENABLE_RTT
#include "rtt.h"
#endif

/* This has to be aligned so the remote protocol can re-use it without causing Problems */
static char pbuf[GDB_PACKET_BUFFER_SIZE + 1U] __attribute__((aligned(8)));

char *gdb_packet_buffer()
{
	return pbuf;
}

static void bmp_poll_loop(void)
{
	SET_IDLE_STATE(false);
	/* 如果 target 正在 running */
	while (gdb_target_running && cur_target) {
		gdb_poll_target();

		// Check again, as `gdb_poll_target()` may
		// alter these variables.
		if (!gdb_target_running || !cur_target)
			break;
		/* 单次接收一个字节 */
		char c = gdb_if_getchar_to(0);
		/* 执行 target 的相关操作??? */
		if (c == '\x03' || c == '\x04')
			/* halt target */
			target_halt_request(cur_target);
		platform_pace_poll();
#ifdef ENABLE_RTT
		if (rtt_enabled)
			poll_rtt(cur_target);
#endif
	}

	/* 开始没有绑定 target 时候要走到这里 */
	SET_IDLE_STATE(true);
	/* 读取 gdb 报文 */
	size_t size = gdb_getpacket(pbuf, GDB_PACKET_BUFFER_SIZE);
	// If port closed and target detached, stay idle
	if (pbuf[0] != '\x04' || cur_target)
		SET_IDLE_STATE(false);
	/* 处理接收到的 gdb 报文 */
	gdb_main(pbuf, GDB_PACKET_BUFFER_SIZE, size);
}

int main(int argc, char **argv)
{
#if PC_HOSTED == 1
	/* 平台初始化 */
	platform_init(argc, argv);
#else
	(void)argc;
	(void)argv;
	platform_init();
#endif

	while (true) {
		volatile exception_s e;
		TRY_CATCH (e, EXCEPTION_ALL) {
			/* 接收数据报文并处理 */
			bmp_poll_loop();
		}
		if (e.type) {
			gdb_putpacketz("EFF");
			target_list_free();
			gdb_outf("Uncaught exception: %s\n", e.msg);
		}
#if PC_HOSTED == 1 && MYIR_LINUX == 0
		if (shutdown_bmda)
			break;
#endif
	}

	target_list_free();
	return 0;
}
