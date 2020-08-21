/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2009 Antti Palosaari <crope@iki.fi>
 * Copyright (C) 2011 Antti Palosaari <crope@iki.fi>
 * Copyright (C) 2012 Thomas Mair <thomas.mair86@googlemail.com>
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 * Copyright (C) 2016 by Robert X. Seger <rseger@gmx.co.uk>
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


#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include "getopt/getopt.h"
#define usleep(x) Sleep(x/1000)
#if defined(_MSC_VER) && (_MSC_VER < 1800)
#define round(x) (x > 0.0 ? floor(x + 0.5): ceil(x - 0.5))
#endif
#define _USE_MATH_DEFINES
#endif

#include <math.h>
#include <libusb.h>

#include <rtl-sdr.h>
#include <rtl_app_ver.h>
#include "convenience/convenience.h"
#include "convenience/rtl_convenience.h"

static volatile int do_exit = 0;

struct dongle_state
{
	int      exit_flag;
	rtlsdr_dev_t *dev;
	int      dev_index;
};

void dongle_init(struct dongle_state *s)
{
    memset(s, 0, sizeof(struct dongle_state));
}

struct dongle_state dongle;

void usage(void)
{
	fprintf(stderr,
		"rtl_ir, display received IR signals\n"
		"rtl_ir  version %d.%d %s (%s)\n"
		"rtl-sdr library %d.%d %s\n\n",
		APP_VER_MAJOR, APP_VER_MINOR, APP_VER_ID, __DATE__,
		rtlsdr_get_version() >>16, rtlsdr_get_version() & 0xFFFF,
		rtlsdr_get_ver_id() );
	fprintf(stderr,
		"Usage:\trtl_ir [-options]\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-w wait_usec]\tDelay to wait before each iteration (10000)\n"
		"\t[-c max_count]\tMaximum number of loop iterations (0)\n"
		"\t[-b]\tDisplay output in binary (default), pulse=1, space=0; each 20 usec\n"
		"\t[-t]\tDisplay output in text format\n"
		"\t[-x]\tDisplay output in raw packed bytes, MSB=pulse/space, 7LSB=duration*20 usec\n"
		"\t[-h]\tHelp\n"
		);
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dongle.dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	rtlsdr_cancel_async(dongle.dev);
}
#endif


int main(int argc, char **argv) {
#ifndef _WIN32
	struct sigaction sigact;
#endif
	int r, opt;
	int i, j;
	int dev_given = 0;
	unsigned int wait_usec = 100000;
	int max_count = 0, iteration_count = 0;
	int output_binary = 0, output_text = 0, output_packed = 0;
	uint8_t buf[128] = { 0 };

	dongle_init(&dongle);

	while ((opt = getopt(argc, argv, "d:c:w:btxh")) != -1) {
		switch (opt) {
		case 'd':
			dongle.dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'w':
			wait_usec = atoi(optarg);
			break;
		case 'c':
			max_count = atoi(optarg);
			break;
		case 'b':
			output_binary = 1;
			break;
		case 't':
			output_text = 1;
			break;
		case 'x':
			output_packed = 1;
			break;
		case 'h':
		default:
			usage();
			break;
		}
	}

	if (dongle.dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dongle.dev, (uint32_t)dongle.dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dongle.dev_index);
		exit(1);
	}
#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	verbose_reset_buffer(dongle.dev);

	if (!output_binary && !output_text && !output_packed)
		output_binary = 1;

	while (!do_exit) {
		usleep(wait_usec);

		r = rtlsdr_ir_query(dongle.dev, buf, sizeof(buf));
		if (r < 0) {
			fprintf(stderr, "rtlsdr_ir_query failed: %d\n", r);
		}

		for (i = 0; i < r; i++) {
			int pulse = buf[i] >> 7;
			int duration = buf[i] & 0x7f;

			if (output_text) {
				printf("pulse %d, duration %d usec\n", pulse, duration * 20);
			}

			if (output_binary) {
				for (j = 0; j < duration; ++j) {
					printf("%d", pulse);
				}
			}

			if (output_packed) {
				putchar(buf[i]);
			}
		}
		if (r != 0) printf("\n");
		fflush(stdout);

		if (max_count != 0 && ++iteration_count >= max_count) do_exit = 1;
	}

	if (do_exit) {
		fprintf(stderr, "\nUser cancel, exiting...\n");}
	else {
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);}

	rtlsdr_cancel_async(dongle.dev);

	rtlsdr_close(dongle.dev);
	return r >= 0 ? r : -r;

	return 0;
}
