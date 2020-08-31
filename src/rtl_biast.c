/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * rtl_biast, tool to set bias tee gpio output
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include "getopt/getopt.h"
#endif

#include <rtl-sdr.h>
#include <rtl_app_ver.h>
#include "convenience/convenience.h"
#include "convenience/rtl_convenience.h"

static rtlsdr_dev_t *dev = NULL;

void usage(void)
{
	fprintf(stderr,
		"rtl_biast, a tool for switching the RTL-SDR.com\n"
		"bias tee or any GPIO ON and OFF. Example to activate\n"
		"the bias tee: rtl_biast -d 0 -b 1\n"
		"Any GPIO:     rtl_biast -d 0 -g 1 -b 1\n"
		"rtl_biast version %d.%d %s (%s)\n"
		"rtl-sdr   library %d.%d %s\n\n",
		APP_VER_MAJOR, APP_VER_MINOR, APP_VER_ID, __DATE__,
		rtlsdr_get_version() >>16, rtlsdr_get_version() & 0xFFFF,
		rtlsdr_get_ver_id() );
	fprintf(stderr,
		"Usage:\trtl_biast [-options]\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-g GPIO select (default: 0)]\n"
		"\t[-b set write bias_on (default: 0, in output mode)]\n"
		"\t[-r read pin (in input mode)]\n"
		"\t[-w write pin (in output mode)]\n"
		"\t[-s read all GPIO pins status (0 = write, 1 = read ?? )]\n"
		"\t[-R read all GPIO pins ?? ]\n");
	exit(1);
}

int main(int argc, char **argv)
{
	int i, r, opt, val;
	int dev_index = 0;
	int dev_given = 0;
	int write_pin_given = 0;
	int read_pin_given = 0;
	int read_all_given = 0;
	int req_status = 0;
	uint32_t bias_on = 0;
	int gpio_pin = 0;
	int device_count;

	while ((opt = getopt(argc, argv, "d:b:w:g:srRh?")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'b':
		case 'w':
			bias_on = atoi(optarg);
			write_pin_given = 1;
			break;
		case 'g':
			gpio_pin = atoi(optarg);
			break;
		case 'r':
			read_pin_given = 1;
			break;
		case 'R':
			read_all_given = 1;
			break;
		case 's':
			req_status = 1;
			break;
		default:
			usage();
			break;
		}
	}

	if (!dev_given) {
		dev_index = verbose_device_search("0");
	}

	if (dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dev, dev_index);
	if (r < 0)
	{
		fprintf(stderr, "error opening device with index %d\n", dev_index);
		return -r;
	}

	if (write_pin_given)
	{
		r = rtlsdr_set_bias_tee_gpio(dev, gpio_pin, bias_on);
		if (r < 0)
			fprintf(stderr, "error setting value %d on pin %d\n", bias_on, gpio_pin);
	}

	else if (read_pin_given)
	{
		r = rtlsdr_set_gpio_input(dev, gpio_pin);
		if (r < 0)
			fprintf(stderr, "error configuring pin %d to input\n", gpio_pin);

		r = rtlsdr_get_gpio_bit(dev, gpio_pin, &val);
		if (r < 0)
			fprintf(stderr, "error reading value for pin %d\n", gpio_pin);
		else
			printf("value %d at pin %d\n", val, gpio_pin);
	}

	else if (read_all_given)
	{
		r = rtlsdr_get_gpio_byte(dev, &val);
		if (r < 0)
			fprintf(stderr, "error reading value for all pins\n");
		else
		{
			printf("GPIO 0x%02x = bin ", val);
			for (gpio_pin = 7; gpio_pin >= 4; --gpio_pin)
				printf("%d", ((val >> gpio_pin) & 1));
			printf(" ");
			for (gpio_pin = 3; gpio_pin >= 0; --gpio_pin)
				printf("%d", ((val >> gpio_pin) & 1));
			printf("\n");
		}
	}

	else if (req_status)
	{
		r = rtlsdr_set_gpio_status(dev, &val);
		if (r < 0)
			fprintf(stderr, "error reading status for all pins\n");
		else
		{
			printf("STATUS 0x%02x = bin ", val);
			for (gpio_pin = 7; gpio_pin >= 4; --gpio_pin)
				printf("%d", ((val >> gpio_pin) & 1));
			printf(" ");
			for (gpio_pin = 3; gpio_pin >= 0; --gpio_pin)
				printf("%d", ((val >> gpio_pin) & 1));
			printf("\n");
		}
	}

	else
	{
		usage();
		r = -1;
	}

exit:
	/*
	 * Note - rtlsdr_close() in this tree does not clear the bias tee
	 * GPIO line, so it leaves the bias tee enabled if a client program
	 * doesn't explictly disable it.
	 *
	 * If that behaviour changes then another rtlsdr_close() will be
	 * needed that takes some extension flags, and one of them should
	 * be to either explicitly close the biast or leave it alone.
	 */
	rtlsdr_close(dev);

	return r >= 0 ? r : -r;
}
