/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
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

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include "getopt/getopt.h"
#endif

#include <rtl-sdr.h>
#include <rtl_app_ver.h>
#include "convenience/convenience.h"
#include "convenience/rtl_convenience.h"
#include "convenience/wavewrite.h"

#define DEFAULT_SAMPLE_RATE		2048000
#define DEFAULT_BANDWIDTH		0	/* automatic bandwidth */
#define DEFAULT_BUF_LENGTH		(16 * 16384)
#define MINIMAL_BUF_LENGTH		512
#define MAXIMAL_BUF_LENGTH		(256 * 16384)


static int do_exit = 0;
static uint32_t iq_frames_to_read = 0;
static rtlsdr_dev_t *dev = NULL;

void usage(void)
{
	fprintf(stderr,
		"rtl_sdr, an I/Q recorder for RTL2832 based SDR-receivers\n"
		"rtl_sdr version %d.%d %s (%s)\n"
		"rtl-sdr library %d.%d %s\n\n",
		APP_VER_MAJOR, APP_VER_MINOR, APP_VER_ID, __DATE__,
		rtlsdr_get_version() >>16, rtlsdr_get_version() & 0xFFFF,
		rtlsdr_get_ver_id() );
	fprintf(stderr,
		"Usage:\trtl_sdr -f frequency_to_tune_to [Hz]\n"
		"\t[-s samplerate (default: 2048000 Hz)]\n"
		"\t[-w tuner_bandwidth (default: automatic)]\n"
		"\t[-d device_index or serial (default: 0)]\n"
		"\t[-g gain (default: 0 for auto)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"%s"
		"\t[-b output_block_size (default: 16 * 16384)]\n"
		"\t[-n number of samples to read (default: 0, infinite)]\n"
		"\t[-S force sync output (default: async)]\n"
		"\t[-N no dithering (default: use dithering)]\n"
		"\t[-H write wave Header to file (default: off)]\n"
		"\tfilename (a '-' dumps samples to stdout)\n\n"
		, rtlsdr_get_opt_help(1) );
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	rtlsdr_cancel_async(dev);
}
#endif

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	if (ctx) {
		if (do_exit)
			return;

		if ((iq_frames_to_read) && (iq_frames_to_read < len/2)) {
			len = 2U * iq_frames_to_read;
			iq_frames_to_read = 0;
			do_exit = 1;
			rtlsdr_cancel_async(dev);
		}

		if (!waveHdrStarted) {
			size_t wr = fwrite(buf, 1, len, (FILE*)ctx);
			if ( wr != len) {
				fprintf(stderr, "Short write (wrote %ld of %ld bytes), samples lost, exiting!\n"
						, (long)wr, (long)len );
				rtlsdr_cancel_async(dev);
			}
		} else {
			if ( waveWriteFrames((FILE*)ctx, buf, len/2, 0) ) {
				fprintf(stderr, "Short write, samples lost, exiting!\n");
				rtlsdr_cancel_async(dev);
			}
		}

		if (iq_frames_to_read) {
			if (iq_frames_to_read > len/2)
				iq_frames_to_read -= len/2;
			else {
				do_exit = 1;
				rtlsdr_cancel_async(dev);
			}
		}
	}
}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	char *filename = NULL;
	char *tempfilename = NULL;
	int n_read;
	int r, opt;
	int gain = 0;
	int ppm_error = 0;
	int sync_mode = 0;
	int dithering = 1;
	FILE *file;
	uint8_t *buffer;
	const char * rtlOpts = NULL;
	int dev_index = 0;
	int dev_given = 0;
	int writeWav = 0;
	uint64_t frequency = 100000000;
	uint32_t bandwidth = DEFAULT_BANDWIDTH;
	uint32_t samp_rate = DEFAULT_SAMPLE_RATE;
	uint32_t out_block_size = DEFAULT_BUF_LENGTH;
	int verbosity = 0;

	while ((opt = getopt(argc, argv, "d:f:g:s:w:b:n:p:O:SNHv")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			frequency = (uint64_t)atofs(optarg);
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10); /* tenths of a dB */
			break;
		case 's':
			samp_rate = (uint32_t)atofs(optarg);
			break;
		case 'w':
			bandwidth = (uint32_t)atofs(optarg);
			break;
		case 'p':
			ppm_error = atoi(optarg);
			break;
		case 'O':
			rtlOpts = optarg;
			break;
		case 'b':
			out_block_size = (uint32_t)atof(optarg);
			break;
		case 'n':
			iq_frames_to_read = (uint32_t)atof(optarg);
			break;
		case 'S':
			sync_mode = 1;
			break;
		case 'N':
			dithering = 0;
			break;
		case 'H':
			writeWav = 1;
			break;
		case 'v':
			++verbosity;
			break;
		default:
			usage();
			break;
		}
	}

	if (argc <= optind) {
		usage();
	} else {
		filename = argv[optind];
	}

	if(out_block_size < MINIMAL_BUF_LENGTH ||
	   out_block_size > MAXIMAL_BUF_LENGTH ){
		fprintf(stderr,
			"Output block size wrong value, falling back to default\n");
		fprintf(stderr,
			"Minimal length: %u\n", MINIMAL_BUF_LENGTH);
		fprintf(stderr,
			"Maximal length: %u\n", MAXIMAL_BUF_LENGTH);
		out_block_size = DEFAULT_BUF_LENGTH;
	}

	buffer = malloc(out_block_size * sizeof(uint8_t));

	if (!dev_given) {
		dev_index = verbose_device_search("0");
	}

	if (dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dev, (uint32_t)dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
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

	if (!dithering) {
		fprintf(stderr, "Disabling dithering...  ");
		r = rtlsdr_set_dithering(dev, dithering);
		if (r) {
			fprintf(stderr, "failure\n");
		} else {
			fprintf(stderr, "success\n");
		}
	}

	/* Set the sample rate */
	verbose_set_sample_rate(dev, samp_rate);

	/* Set the tuner bandwidth */
	verbose_set_bandwidth(dev, bandwidth);

	/* Set the frequency */
	verbose_set_frequency(dev, frequency);

	if (0 == gain) {
		 /* Enable automatic gain */
		verbose_auto_gain(dev);
	} else {
		/* Enable manual gain */
		gain = nearest_gain(dev, gain);
		verbose_gain_set(dev, gain);
	}

	if (rtlOpts) {
		rtlsdr_set_opt_string(dev, rtlOpts, verbosity);
	}

	verbose_ppm_set(dev, ppm_error);

	if(strcmp(filename, "-") == 0) { /* Write samples to stdout */
		file = stdout;
#ifdef _WIN32
		_setmode(_fileno(stdin), _O_BINARY);
#endif
	} else {
		const char * filename_to_open = filename;
		if (writeWav) {
			tempfilename = malloc( strlen(filename)+8 );
			strcpy(tempfilename, filename);
			strcat(tempfilename, ".tmp");
			filename_to_open = tempfilename;
		}
		file = fopen(filename_to_open, "wb");
		if (!file) {
			fprintf(stderr, "Failed to open %s\n", filename_to_open);
			goto out;
		}
		if (writeWav) {
			waveWriteHeader(samp_rate, frequency, 8, 2, file);
		}
	}

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dev);

	if (sync_mode) {
		fprintf(stderr, "Reading samples in sync mode...\n");
		while (!do_exit) {
			r = rtlsdr_read_sync(dev, buffer, out_block_size, &n_read);
			if (r < 0) {
				fprintf(stderr, "WARNING: sync read failed.\n");
				break;
			}

			if ((iq_frames_to_read) && (iq_frames_to_read < ((uint32_t)n_read /2))) {
				n_read = 2U * iq_frames_to_read;
				do_exit = 1;
			}

			if (!waveHdrStarted) {
				size_t wr = fwrite(buffer, 1, n_read, file);
				if (wr != (size_t)n_read) {
					fprintf(stderr, "Short write (wrote %ld of %ld bytes), samples lost, exiting!\n"
							, (long)wr, (long)n_read );
					break;
				}
			} else {
				if ( waveWriteSamples(file, buffer, n_read/2, 0) ) {
					fprintf(stderr, "Short write, samples lost, exiting!\n");
					break;
				}
			}

			if ((uint32_t)n_read < out_block_size) {
				fprintf(stderr, "Short read, samples lost, exiting!\n");
				break;
			}

			if (iq_frames_to_read) {
				if (iq_frames_to_read > ((uint32_t)n_read /2))
					iq_frames_to_read -= n_read/2;
				else
					do_exit = 1;
			}
		}
	} else {
		fprintf(stderr, "Reading samples in async mode...\n");
		r = rtlsdr_read_async(dev, rtlsdr_callback, (void *)file,
				      0, out_block_size);
	}

	if (file != stdout) {
		if (writeWav) {
			waveFinalizeHeader(file);
			fclose(file);
			remove(filename);	/* delete, in case file already exists */
			r = rename( tempfilename, filename );	/* #include <stdio.h> */
			if ( r )
				fprintf( stderr, "%s: error %d '%s' renaming'%s' to '%s'\n"
					, argv[0], errno, strerror(errno), tempfilename, filename );
		} else {
			fclose(file);
		}

	}

	if (do_exit)
		fprintf(stderr, "\nUser cancel, exiting...\n");
	else
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);

	rtlsdr_close(dev);
	free (buffer);
out:
	return r >= 0 ? r : -r;
}
