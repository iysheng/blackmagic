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

/* This file implements the platform specific functions for the native implementation. */

#include "general.h"
#include "platform.h"
#include "morse.h"

bool running_status = false;

/* This is defined by the linker script */
extern char vector_table;

/*
 * Starting with hardware version 4 we are storing the hardware version in the
 * flash option user Data1 byte.
 * The hardware version 4 was the transition version that had it's hardware
 * pins strapped to 3 but contains version 4 in the Data1 byte.
 * The hardware 4 is backward compatible with V3 but provides the new jumper
 * connecting STRACE target pin to the UART1 pin.
 * Hardware version 5 does not have the physically strapped version encoding
 * any more and the hardware version has to be read out of the option bytes.
 * This means that older firmware versions that don't do the detection won't
 * work on the newer hardware.
 */
#define BMP_HWVERSION_BYTE FLASH_OPTION_BYTE_2

/*
 * Pins PB[7:5] are used to detect hardware revision.
 * User option byte Data1 is used starting with hardware revision 4.
 * Pin -  OByte - Rev - Description
 * 000 - 0xffff -   0 - Original production build.
 * 001 - 0xffff -   1 - Mini production build.
 * 010 - 0xffff -   2 - Mini V2.0e and later.
 * 011 - 0xffff -   3 - Mini V2.1a and later.
 * 011 - 0xfb04 -   4 - Mini V2.1d and later.
 * xxx - 0xfb05 -   5 - Mini V2.2a and later.
 * xxx - 0xfb06 -   6 - Mini V2.3a and later.
 *
 * This function will return -2 if the version number does not make sense.
 * This can happen when the Data1 byte contains "garbage". For example a
 * hardware revision that is <4 or the high byte is not the binary inverse of
 * the lower byte.
 * Note: The high byte of the Data1 option byte should always be the binary
 * inverse of the lower byte unless the byte is not set, then all bits in both
 * high and low byte are 0xff.
 */
int platform_hwversion(void)
{
	static int hwversion = -1;

	return hwversion;
}

void platform_init(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
}

void platform_nrst_set_val(bool assert)
{
	(void)assert;
}

bool platform_nrst_get_val(void)
{
	return false;
}

bool platform_target_get_power(void)
{
	return false;
}

bool platform_target_set_power(const bool power)
{
	(void)power;
	return true;
}

uint32_t platform_target_voltage_sense(void)
{
	return 123;
}

const char *platform_target_voltage(void)
{
	static char ret[] = "0.0V";
	uint32_t val = platform_target_voltage_sense();
	ret[0] = '0' + val / 10U;
	ret[2] = '0' + val % 10U;

	return ret;
}

void platform_request_boot(void)
{
}

void platform_target_clk_output_enable(bool enable)
{
	(void)enable;
}

void platform_pace_poll(void)
{

}

uint32_t platform_time_ms(void)
{
	return 1;
}

void platform_delay(uint32_t ms)
{
	(void)ms;
}

void morse(const char *msg, bool repeat)
{
    (void)msg;
	(void)repeat;
}

void platform_max_frequency_set(uint32_t freq)
{
	(void)freq;
}

uint32_t platform_max_frequency_get(void)
{
	return 200000;
}
