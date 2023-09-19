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

/*
 * This file implements a transparent channel over which the GDB Remote
 * Serial Debugging protocol is implemented. This implementation for STM32
 * uses the USB CDC-ACM device bulk endpoints to implement the channel.
 */

#include "general.h"
#include "platform.h"
#include "gdb_if.h"

#define CDCACM_PACKET_SIZE 512

static __attribute__ ((unused)) uint32_t count_out;
static __attribute__ ((unused)) uint32_t count_in;
static __attribute__ ((unused)) uint32_t out_ptr;
static __attribute__ ((unused)) char buffer_out[CDCACM_PACKET_SIZE];
static __attribute__ ((unused)) char buffer_in[CDCACM_PACKET_SIZE];

void gdb_if_putchar(const char c, const int flush)
{
	(void)c;
	(void)flush;
}

char gdb_if_getchar(void)
{
	return buffer_out[out_ptr++];
}

char gdb_if_getchar_to(const uint32_t timeout)
{
	(void)timeout;
	return -1;
}
