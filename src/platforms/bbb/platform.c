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

#include <sys/stat.h>
#include <sys/select.h>
#include <dirent.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <unistd.h>
#include <time.h>

#include "general.h"
#include "platform.h"
#include "morse.h"

uint64_t time_ms;
typedef struct timeval timeval_s;
bool running_status = false;
extern unsigned cortexm_wait_timeout;
int fd;
static unsigned int gs_voltage_sense = 0x00;

/* This is defined by the linker script */
extern char vector_table;

#define usleep(x) _delay_here()

void _delay_here(void)
{
	__asm__("nop");
	/* __asm__("nop"); */
	/* __asm__("nop"); */
	/* __asm__("nop"); */
}

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

#define SWDIO_PINS_DELAY_US    15
static bool set_interface_attribs(void)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0) {
		DEBUG_ERROR("error %d from tcgetattr", errno);
		return false;
	}

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK; // disable break processing
	tty.c_lflag = 0;        // no signaling chars, no echo,
	// no canonical processing
	tty.c_oflag = 0;     // no remapping, no delays
	tty.c_cc[VMIN] = 0;  // read doesn't block
	tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
	// enable reading
	tty.c_cflag &= ~CSTOPB;
#if defined(CRTSCTS)
	tty.c_cflag &= ~CRTSCTS;
#endif
	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		DEBUG_ERROR("error %d from tcsetattr", errno);
		return false;
	}
	return true;
}

int platform_buffer_write(const void *const data, const size_t length)
{
	DEBUG_WIRE("%s\n", (const char *)data);
	const ssize_t written = write(fd, data, length);
	if (written < 0) {
		const int error = errno;
		DEBUG_ERROR("Failed to write (%d): %s\n", errno, strerror(error));
		exit(-2);
	}
	else
	{
		tcdrain(fd);
	}
	return (size_t)written == length;
}

/* XXX: We should either return size_t or bool */
/* XXX: This needs documenting that it can abort the program with exit(), or the error handling fixed */
int platform_buffer_read(void *const data, size_t length)
{
	char response = 0;
	timeval_s timeout = {
		.tv_sec = cortexm_wait_timeout / 1000U,
		.tv_usec = 1000U * (cortexm_wait_timeout % 1000U),
	};

	/* Now collect the response */
	for (size_t offset = 0; offset < length;) {
		fd_set select_set;
		FD_ZERO(&select_set);
		FD_SET(fd, &select_set);
		const int result = select(FD_SETSIZE, &select_set, NULL, NULL, &timeout);
		if (result < 0) {
			DEBUG_ERROR("Failed on select\n");
			exit(-4);
		}
		if (result == 0) {
			/* DEBUG_ERROR("Timeout on read\n"); */
			return -5;
		}
		if (read(fd, data + offset, 1) != 1) {
			const int error = errno;
			DEBUG_ERROR("Failed to read response (%d): %s\n", error, strerror(error));
			return -6;
		}
		++offset;
	}

	return length;
}

int platform_buffer_read_extend(void *const data, size_t length, uint32_t ms)
{
	char response = 0;
	timeval_s timeout;
	timeout.tv_sec = ms / 1000U;
	timeout.tv_usec = 1000U * (ms % 1000U);

	/* Now collect the response */
	for (size_t offset = 0; offset < length;) {
		fd_set select_set;
		FD_ZERO(&select_set);
		FD_SET(fd, &select_set);
		const int result = select(FD_SETSIZE, &select_set, NULL, NULL, &timeout);
		if (result < 0) {
			DEBUG_ERROR("Failed on select\n");
			exit(-4);
		}
		if (result == 0) {
			DEBUG_ERROR("Timeout on read\n");
			return -5;
		}
		if (read(fd, data + offset, 1) != 1) {
			const int error = errno;
			DEBUG_ERROR("Failed to read response (%d): %s\n", error, strerror(error));
			return -6;
		}
		++offset;
	}

	return length;
}

static void sigterm_handler(int sig)
{
	(void)sig;
	exit(0);
}

/* pins0 -> 33 -> swdio */
/* pins1 -> 332 -> swclk */
static struct gpiohandle_request gs_bbb_jtag_pins[BBB_MAX_PIN_HANDLER];
/* struct gpiohandle_config swdio_config; */

uint16_t bbb_gpio_get(const uint32_t gpioport, const uint16_t gpios)
{
	int ret = -1;
	static struct gpiohandle_data data = {
		.values[0] = 1,
	};

	if (SWCLK_PORT == gpioport && gpios == SWCLK_PIN)
	{
    	ret = ioctl(gs_bbb_jtag_pins[BBB_SWCLK_PIN_HANDLER].fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
	}
	else if (SWDIO_PORT == gpioport && gpios == SWDIO_PIN)
	{
    	ret = ioctl(gs_bbb_jtag_pins[BBB_SWDIO_PIN_HANDLER].fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
	}

	/* printf("ret=%d ans=%u@(%u, %u)\n", ret, data.values[0], gpioport, gpios); */

	if (!ret)
		return data.values[0];
	else
		return 1;
}

void bbb_gpio_set(const uint32_t gpioport, const uint16_t gpios)
{
	int ret;
	static struct gpiohandle_data data = {
		.values[0] = 1,
	};

	if (SWCLK_PORT == gpioport && gpios == SWCLK_PIN)
	{
    	ret = ioctl(gs_bbb_jtag_pins[BBB_SWCLK_PIN_HANDLER].fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
	}
	else if (SWDIO_PORT == gpioport && gpios == SWDIO_PIN)
	{
    	ret = ioctl(gs_bbb_jtag_pins[BBB_SWDIO_PIN_HANDLER].fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
	}
	usleep(SWDIO_PINS_DELAY_US);
}

void bbb_gpio_clear(const uint32_t gpioport, const uint16_t gpios)
{
	int ret;
	static struct gpiohandle_data data = {
		.values[0] = 0,
	};

	if (SWCLK_PORT == gpioport && gpios == SWCLK_PIN)
	{
    	ret = ioctl(gs_bbb_jtag_pins[BBB_SWCLK_PIN_HANDLER].fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
	}
	else if (SWDIO_PORT == gpioport && gpios == SWDIO_PIN)
	{
    	ret = ioctl(gs_bbb_jtag_pins[BBB_SWDIO_PIN_HANDLER].fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
	}
	usleep(SWDIO_PINS_DELAY_US);
}

void bbb_gpio_set_val(const uint32_t gpioport, const uint16_t gpios, const bool val)
{
	int ret;
	static struct gpiohandle_data data = {
		.values[0] = 0,
	};

    if (val == true)
		data.values[0] = 1;
	else
		data.values[0] = 0;

	if (SWCLK_PORT == gpioport && gpios == SWCLK_PIN)
	{
    	ret = ioctl(gs_bbb_jtag_pins[BBB_SWCLK_PIN_HANDLER].fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
	}
	else if (SWDIO_PORT == gpioport && gpios == SWDIO_PIN)
	{
    	ret = ioctl(gs_bbb_jtag_pins[BBB_SWDIO_PIN_HANDLER].fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
	}
	usleep(SWDIO_PINS_DELAY_US);
}

void bbb_gpio_set_mode(int port, int mode, int pullpush, int pin)
{
	(void)port;
	(void)mode;
	(void)pullpush;
	(void)pin;
}

int swdio_mode_float(void)
{
	int ret;
    static struct gpiohandle_config swdio_config = {
		.flags = GPIOHANDLE_REQUEST_INPUT | GPIOHANDLE_REQUEST_BIAS_DISABLE,
	};
	static struct gpiohandle_data tms_dir_data = {
		.values[0] = 0,
	};

    ret = ioctl(gs_bbb_jtag_pins[BBB_TMSDIR_PIN_HANDLER].fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &tms_dir_data);
	if (ret)
	{
		fprintf(stderr, "Failed to set tms_dir line value:%d\n", ret);
	}
    ret = ioctl(gs_bbb_jtag_pins[BBB_SWDIO_PIN_HANDLER].fd, GPIOHANDLE_SET_CONFIG_IOCTL, &swdio_config);
	if (ret)
	{
		fprintf(stderr, "Failed to set swdio config:%d\n", ret);
	}
	usleep(SWDIO_PINS_DELAY_US);
    return ret;
}

int swdio_mode_drive(void)
{
	int ret;
    static struct gpiohandle_config swdio_config = {
		.flags = GPIOHANDLE_REQUEST_OUTPUT,
	};
	static struct gpiohandle_data tms_dir_data = {
		.values[0] = 1,
	};

    ret = ioctl(gs_bbb_jtag_pins[BBB_TMSDIR_PIN_HANDLER].fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &tms_dir_data);
	if (ret)
	{
		fprintf(stderr, "Failed to set tms_dir line value:%d\n", ret);
	}
    ret = ioctl(gs_bbb_jtag_pins[BBB_SWDIO_PIN_HANDLER].fd, GPIOHANDLE_SET_CONFIG_IOCTL, &swdio_config);
	if (ret)
	{
		fprintf(stderr, "Failed to set swdio config:%d\n", ret);
	}
	usleep(SWDIO_PINS_DELAY_US);

	return ret;
}

int gpio_handler_init(void)
{
	struct gpiohandle_data data;
	char chrdev_name[4][20];
	int fd[4], ret;
	int i;

	for (i = 0; i < 4; i++)
	{
    	snprintf(chrdev_name[i], 20, "/dev/gpiochip%d", i);
		fd[i] = open(chrdev_name[i], 0);
    	if (fd[i] == -1) {
    		ret = -errno;
    		fprintf(stderr, "Failed to open %s\n", chrdev_name[i]);
    		return ret;
    	}
	}


	gs_bbb_jtag_pins[BBB_SWDIO_PIN_HANDLER].lineoffsets[0] = TMS_PIN;
	gs_bbb_jtag_pins[BBB_SWDIO_PIN_HANDLER].flags = GPIOHANDLE_REQUEST_OUTPUT;
	gs_bbb_jtag_pins[BBB_SWDIO_PIN_HANDLER].lines = 1;
	memcpy(gs_bbb_jtag_pins[BBB_SWDIO_PIN_HANDLER].default_values, &data, sizeof(gs_bbb_jtag_pins[BBB_SWDIO_PIN_HANDLER].default_values));
	strcpy(gs_bbb_jtag_pins[BBB_SWDIO_PIN_HANDLER].consumer_label, "swdio");

	ret = ioctl(fd[2], GPIO_GET_LINEHANDLE_IOCTL, &gs_bbb_jtag_pins[BBB_SWDIO_PIN_HANDLER]);
	if (ret == -1) {
		ret = -errno;
		fprintf(stderr, "Failed to issue GET LINEHANDLE IOCTL (%d)\n",
			ret);
	}

	gs_bbb_jtag_pins[BBB_SWCLK_PIN_HANDLER].lineoffsets[0] = TCK_PIN;
	gs_bbb_jtag_pins[BBB_SWCLK_PIN_HANDLER].flags = GPIOHANDLE_REQUEST_OUTPUT;
	gs_bbb_jtag_pins[BBB_SWCLK_PIN_HANDLER].lines = 1;
	memcpy(gs_bbb_jtag_pins[BBB_SWCLK_PIN_HANDLER].default_values, &data, sizeof(gs_bbb_jtag_pins[BBB_SWCLK_PIN_HANDLER].default_values));
	strcpy(gs_bbb_jtag_pins[BBB_SWCLK_PIN_HANDLER].consumer_label, "swclk");
	ret = ioctl(fd[2], GPIO_GET_LINEHANDLE_IOCTL, &gs_bbb_jtag_pins[BBB_SWCLK_PIN_HANDLER]);
	if (ret == -1) {
		ret = -errno;
		fprintf(stderr, "Failed to issue GET LINEHANDLE IOCTL (%d)\n",
			ret);
	}

	gs_bbb_jtag_pins[BBB_TMSDIR_PIN_HANDLER].lineoffsets[0] = TMS_DIR_PIN;
	gs_bbb_jtag_pins[BBB_TMSDIR_PIN_HANDLER].flags = GPIOHANDLE_REQUEST_OUTPUT;
	gs_bbb_jtag_pins[BBB_TMSDIR_PIN_HANDLER].lines = 1;
	memcpy(gs_bbb_jtag_pins[BBB_TMSDIR_PIN_HANDLER].default_values, &data, sizeof(gs_bbb_jtag_pins[BBB_TMSDIR_PIN_HANDLER].default_values));
	strcpy(gs_bbb_jtag_pins[BBB_TMSDIR_PIN_HANDLER].consumer_label, "tmsdir");
	ret = ioctl(fd[2], GPIO_GET_LINEHANDLE_IOCTL, &gs_bbb_jtag_pins[BBB_TMSDIR_PIN_HANDLER]);
	if (ret == -1) {
		ret = -errno;
		fprintf(stderr, "Failed to issue GET LINEHANDLE IOCTL (%d)\n",
			ret);
	}

	gs_bbb_jtag_pins[BBB_PWR_BR_PIN_HANDLER].lineoffsets[0] = PWR_BR_PIN;
	gs_bbb_jtag_pins[BBB_PWR_BR_PIN_HANDLER].flags = GPIOHANDLE_REQUEST_OUTPUT;
	gs_bbb_jtag_pins[BBB_PWR_BR_PIN_HANDLER].lines = 1;
	memcpy(gs_bbb_jtag_pins[BBB_PWR_BR_PIN_HANDLER].default_values, &data, sizeof(gs_bbb_jtag_pins[BBB_PWR_BR_PIN_HANDLER].default_values));
	strcpy(gs_bbb_jtag_pins[BBB_PWR_BR_PIN_HANDLER].consumer_label, "pwrbr");
	ret = ioctl(fd[2], GPIO_GET_LINEHANDLE_IOCTL, &gs_bbb_jtag_pins[BBB_PWR_BR_PIN_HANDLER]);
	if (ret == -1) {
		ret = -errno;
		fprintf(stderr, "Failed to issue GET LINEHANDLE IOCTL (%d)\n",
			ret);
	}
	for (i = 0; i < 4; i++)
    	if (close(fd[i]) == -1)
    		perror("Failed to close GPIO character device file");

	return ret;
}

void handle(union sigval v){
	(void)v;
    time_ms++;
    return;
}

int timer_init(void)
{
    struct sigevent evp;
    struct itimerspec ts;
    timer_t timer;
    int ret;
    memset   (&evp, 0, sizeof(evp));

    evp.sigev_value.sival_ptr = &timer;
    evp.sigev_notify = SIGEV_THREAD;
    evp.sigev_notify_function = handle;
    evp.sigev_value.sival_int = 3;   //作为handle()的参数
    ret = timer_create(CLOCK_REALTIME, &evp, &timer);
    if( ret){
        perror("timer_create");
    }

    ts.it_value.tv_sec = 0;
    ts.it_value.tv_nsec = 1000000;
    ts.it_interval.tv_sec = ts.it_value.tv_sec;
    ts.it_interval.tv_nsec = ts.it_value.tv_nsec;
    ret = timer_settime(timer, TIMER_ABSTIME, &ts, NULL);
    if(ret)
    {
        perror("timer_settime");
    }

	return ret;
}

void platform_init(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	printf("plaform init here\n");
	signal(SIGTERM, sigterm_handler);
	signal(SIGINT, sigterm_handler);

	if (argc > 1)
	{
    	fd = open(argv[1], O_RDWR | O_SYNC | O_NOCTTY);
	}
	else
	{
    	fd = open("/dev/ttyGS0", O_RDWR | O_SYNC | O_NOCTTY);
	}
	if (fd < 0) {
		DEBUG_ERROR("Couldn't open serial port %s\n", argc > 1 ? argv[1] : "/dev/ttyBMP0");
		return;
	}
	/* BMP only offers an USB-Serial connection with no real serial
	 * line in between. No need for baudrate or parity.!
	 */
	if (false == set_interface_attribs())
	{
		DEBUG_ERROR("Oh no failed init attribs\n");
		printf("red Oh no failed init attribs\n");
	}

    if (gpio_handler_init() < 0)
	{
		DEBUG_ERROR("Failed do swdp pins init\n");
	}
	timer_init();
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
	int ret;
	struct gpiohandle_data data = {
		.values[0] = 1,
	};
	if (power == true)
	{
		data.values[0] = 0;
        gs_voltage_sense = 32;
	}
	else
	{
        gs_voltage_sense = 0;
	}

    ret = ioctl(gs_bbb_jtag_pins[BBB_PWR_BR_PIN_HANDLER].fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
	return true;
}

uint32_t platform_target_voltage_sense(void)
{
	return gs_voltage_sense;
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
	return time_ms;
}

void platform_delay(uint32_t ms)
{
	(void)ms;
}

/* 设置最大速率 */
void platform_max_frequency_set(uint32_t freq)
{
	(void)freq;
}

uint32_t platform_max_frequency_get(void)
{
	/* 限制最大速率为 200K */
	return 200000;
}
