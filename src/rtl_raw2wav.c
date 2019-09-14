/*
 * rtl-raw2wav, converts binary/raw data into wave files - including frequency
 * Copyright (C) 2019 by Hayati Ayguen <h_ayguen@web.de>
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
#include <ctype.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include "getopt/getopt.h"
#if defined(_MSC_VER) && (_MSC_VER < 1900)
#define snprintf _snprintf
#endif
#endif

#include "convenience/convenience.h"
#include "convenience/wavewrite.h"

#include "rtl_app_ver.h"


static volatile int do_exit = 0;
static int verbosity = 0;


void usage(void)
{
	fprintf(stderr, "rtl_raw2wav, a raw (binary sampledata) to wave file converter\n");
	fprintf(stderr, "rtl_raw2wav version %u.%u %s (%s)\n",
		APP_VER_MAJOR, APP_VER_MINOR,
		APP_VER_ID, __DATE__ );
	fprintf(stderr,
		"Use:\trtl_raw2wav -w <output_wave_filename> [-options] [input_raw_filename]\n"
		"\t-w filename     output filename\n"
		"\t-f frequency    frequency, to write into output filename\n"
		"\t-s samplerate   samplerate, of raw input\n"
		"\t-c #channels    number of channels, default: 2 - for I and Q\n"
		"\t-b #bits        number of bits per sample, default: 8 - 8 or 16 allowed\n"
		"\t-v              verbose output\n"
		"\t-r filename     input filename for raw samples, default: - for stdin\n\n" );
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
}
#endif


int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	int r, opt;
	const char * rawfilename = NULL;
	const char * wavfilename = NULL;
	char *tempfilename = NULL;
	FILE * outfile = NULL;
	FILE * inpfile = NULL;
	uint32_t freq = 0;
	uint32_t srate = 0;
	int nChan = 2;
	int nBits = 8;
	char acBuf[ 65536 * sizeof(int16_t) * 2 ];  /* 64 K frames with 2 channels and 16 Bit */
	size_t smpSize;
	size_t nRead;

	while ((opt = getopt(argc, argv, "f:s:c:b:r:w:vh")) != -1) {
		switch (opt) {
		case 'f':	freq = (uint32_t)atofs(optarg);		break;
		case 's':	srate = (uint32_t)atofs(optarg);	break;
		case 'c':	nChan = atoi(optarg);	break;
		case 'b':	nBits = 8 * atoi(optarg);	break;
		case 'v':	++verbosity;	break;
		case 'r':	rawfilename = optarg;	break;
		case 'w':	wavfilename = optarg;	break;
		case 'h':
		case '?':
		default:
			usage();
			exit(1);
			break;
		}
	}

	if (verbosity)
		fprintf(stderr, "verbosity set to %d\n", verbosity);

	if (optind < argc) {
		rawfilename = argv[optind];
	}

	if (!wavfilename) {
		usage();
		fprintf(stderr, "error: missing output wave filename!\n");
		exit(1);
	}

	if (nChan < 1 || nChan > 2) {
		usage();
		fprintf(stderr, "error: number of channels must be 1 or 2!\n");
		exit(1);
	}
	if (nBits != 8 && nBits != 16) {
		usage();
		fprintf(stderr, "error: number of bits per sample must be 8 or 16!\n");
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


	if (!rawfilename || !strcmp(rawfilename, "-")) {
#ifdef _WIN32
		_setmode(_fileno(stdin), _O_BINARY);
#endif
		inpfile = stdin;
	} else {
		inpfile = fopen(rawfilename, "rb");
		if (!inpfile) {
			fprintf(stderr, "error: Failed to open input %s\n", rawfilename);
			exit(1);
		}
	}

	tempfilename = malloc( strlen(wavfilename)+8 );
	strcpy(tempfilename, wavfilename);
	strcat(tempfilename, ".tmp");

	outfile = fopen(tempfilename, "wb");
	if (!outfile) {
		fprintf(stderr, "Error: Failed to open output %s\n", tempfilename);
		exit(1);
	} else {
		if (verbosity)
			fprintf(stderr, "Open %s for write\n", tempfilename);
		waveWriteHeader(srate, freq, nBits, nChan, outfile);
	}

	smpSize = nChan * ( nBits == 16 ? sizeof(int16_t) : sizeof(uint8_t) );
	while ( !feof(inpfile) ) {
		if (do_exit) {
			fprintf(stderr, "\nUser cancel, exiting...\n");
			break;
		}
		nRead = fread(acBuf, smpSize, 65536, inpfile);
		if (nRead)
			fwrite(acBuf, smpSize, nRead, outfile);
	}

	waveFinalizeHeader(outfile);
	fclose(outfile);
	remove(wavfilename);	/* delete, in case file already exists */
	r = rename( tempfilename, wavfilename );	/* #include <stdio.h> */
	if ( r )
		fprintf( stderr, "%s: error %d '%s' renaming'%s' to '%s'\n"
			, argv[0], errno, strerror(errno), tempfilename, wavfilename );

	return 0;
}

/* vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab */
