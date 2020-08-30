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
#include <math.h>

#ifndef _WIN32
  #include <unistd.h>
  #include <sys/types.h>
  #include <sys/stat.h>

#else
  #include <windows.h>
  #include <fcntl.h>
  #include <io.h>
  #include <sys/types.h>
  #include <sys/stat.h>

  #include "getopt/getopt.h"

  #if defined(_MSC_VER) && (_MSC_VER < 1900)
    #define snprintf _snprintf
  #endif
#endif

#include <rtl_app_ver.h>
#include "convenience/convenience.h"
#include "convenience/wavewrite.h"


static volatile int do_exit = 0;
static int verbosity = 0;


void usage(void)
{
	fprintf(stderr,
		"rtl_raw2wav, a raw (binary sampledata) to wave file converter\n"
		"rtl_raw2wav version %d.%d %s (%s)\n\n",
		APP_VER_MAJOR, APP_VER_MINOR, APP_VER_ID, __DATE__ );
	fprintf(stderr,
		"Usage:\trtl_raw2wav -w <output_wave_filename> [-options] [input_raw_filename]\n"
		"\t-w filename     output filename\n"
		"\t-f frequency    frequency, to write into output filename\n"
		"\t-s samplerate   samplerate, of raw input\n"
		"\t-c #channels    number of channels, default: 2 - for I and Q\n"
		"\t-b #bits        number of bits per sample, default: 8 - 8 or 16 allowed\n"
		"\t-u time         set start time in UTC:       'yyy-mm-ddThh:mm:dd.zzz'\n"
		"\t-t time         set start time in localtime: 'yyy-mm-ddThh:mm:dd.zzz'\n"
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
	time_t tim = 0;
	double fraction = 0.0;
	int haveTime = 0;

	while ((opt = getopt(argc, argv, "f:s:c:b:r:u:t:w:vh")) != -1) {
		switch (opt) {
		case 'f':	freq = (uint32_t)atofs(optarg);		break;
		case 's':	srate = (uint32_t)atofs(optarg);	break;
		case 'c':	nChan = atoi(optarg);	break;
		case 'b':	nBits = atoi(optarg);
				if (nBits <= 4) /* assume intention was: bytes per sample */
					nBits *= 8;
				break;
		case 'v':	++verbosity;	break;
		case 'r':	rawfilename = optarg;	break;
		case 'u':	tim = utctimestr_to_time(optarg, &fraction);
			if (tim)	haveTime = 1;
			break;
		case 't':	tim = localtimestr_to_time(optarg, &fraction);
			if (tim)	haveTime = 1;
			break;
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

	while (!wavfilename) {
		if (rawfilename) {
			int slen = strlen(rawfilename);
			const char * rawEnd = rawfilename + slen;
			if (slen > 4 && (!strcmp(rawEnd - 4, ".bin") || !strcmp(rawEnd - 4, ".raw"))) {
				char * wfn = strdup(rawfilename);
				strcpy(wfn+slen-4, ".wav");
				wavfilename = wfn;
				if (verbosity)
					fprintf(stderr, "Warning: deduced .wav filename '%s' from rawfilename '%s'\n", wavfilename, rawfilename);
				break;  /* the while() */
			}
		}
		usage();
		fprintf(stderr, "Error: missing output wave filename!\n");
		exit(1);
	}

	if (nChan < 1 || nChan > 2) {
		usage();
		fprintf(stderr, "Error: number of channels must be 1 or 2!\n");
		exit(1);
	}
	if (nBits != 8 && nBits != 16) {
		usage();
		fprintf(stderr, "Error: number of bits per sample must be 8 or 16!\n");
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

	smpSize = nChan * ( nBits == 16 ? sizeof(int16_t) : sizeof(uint8_t) );
	if (verbosity >= 2)
		fprintf(stderr, "Frame size = %d channels * %d bits = %d bytes\n", nChan, nBits, (int)smpSize);

	if (!rawfilename || !strcmp(rawfilename, "-")) {
#ifdef _WIN32
		_setmode(_fileno(stdin), _O_BINARY);
#endif
		inpfile = stdin;
		if (verbosity)
			fprintf(stderr, "Using stdin as input\n");
	} else {
		if (verbosity >= 2)
			fprintf(stderr, "Opening input '%s'\n", rawfilename);
		inpfile = fopen(rawfilename, "rb");
		if (!inpfile) {
			fprintf(stderr, "Error: Failed to open input %s\n", rawfilename);
			exit(1);
		}
		if (verbosity)
			fprintf(stderr, "Opened '%s' for input\n", rawfilename);
		if (!haveTime) {
			int gotmodtim = 0;
#if defined(_WIN32)
			struct _stat attr;
			if (!_stat(rawfilename, &attr)) {
				tim = attr.st_mtime;
				fraction = 0.0;
				gotmodtim = 1;
			}
#elif defined(__APPLE__)
			struct stat attr;
			if (!stat(rawfilename, &attr)) {
				tim = attr.st_mtime;
				fraction = attr.st_mtimespec.tv_nsec / 1E9;
				gotmodtim = 1;
			}
#else
			struct stat attr;
			if (!stat(rawfilename, &attr)) {
				tim = attr.st_mtime;
				fraction = attr.st_mtim.tv_nsec / 1E9;
				gotmodtim = 1;
			}
#endif
			if (gotmodtim) {
				long fs = 0;
				if (verbosity >= 2)
					fprintf(stderr, "Got 'last modified' from input file '%s'\n", rawfilename);
				if (!fseek(inpfile, 0, SEEK_END)) {
					fs = ftell(inpfile);
					if (fs > 0) {
						double dur = (double)(fs) / ( (double)srate * smpSize );
						double di = floor(dur);
						double df = dur - di;
						tim -= (time_t)di;
						fraction -= df;
						if (fraction < 0.0) {
							fraction += 1.0;
							tim -= 1;
						}
						if (verbosity)
							fprintf(stderr, "Set output's timestamp from input files 'last modified' minus duration of %f seconds\n", dur);
						haveTime = 1;
					}
				}
				fseek(inpfile, 0, SEEK_SET);
			}
		}
	}

	tempfilename = malloc( strlen(wavfilename)+8 );
	strcpy(tempfilename, wavfilename);
	strcat(tempfilename, ".tmp");

	if (verbosity >= 2)
		fprintf(stderr, "Opening output '%s'\n", tempfilename);
	outfile = fopen(tempfilename, "wb");
	if (!outfile) {
		fprintf(stderr, "Error: Failed to open output '%s'\n", tempfilename);
		exit(1);
	} else {
		if (verbosity)
			fprintf(stderr, "Opened '%s' for output\n", tempfilename);
		waveWriteHeader(srate, freq, nBits, nChan, outfile);
		if (haveTime) {
			if (verbosity >= 2)
				fprintf(stderr, "Setting start time of output file\n");
			waveSetStartTime(tim, fraction);
		}
	}

	while ( !feof(inpfile) ) {
		if (do_exit) {
			fprintf(stderr, "\nUser cancel, exiting...\n");
			break;
		}
		nRead = fread(acBuf, smpSize, 65536, inpfile);
		if (nRead > 0)
			waveWriteFrames(outfile, acBuf, nRead, 0);
		if (verbosity >= 2)
			fprintf(stderr, ".");
	}

	if (verbosity >= 2)
		fprintf(stderr, "\nWriting output header\n");
	waveFinalizeHeader(outfile);
	fclose(outfile);
	if (verbosity)
		fprintf(stderr, "Closed output file.\n");
	if (verbosity >= 2)
		fprintf(stderr, "Deleting '%s' - in case it exists\n", wavfilename);
	remove(wavfilename);	/* delete, in case file already exists */
	if (verbosity >= 2)
		fprintf(stderr, "Renaming '%s' into '%s'\n", tempfilename, wavfilename);
	r = rename( tempfilename, wavfilename );	/* #include <stdio.h> */
	if ( r )
		fprintf( stderr, "%s: Error %d '%s' renaming'%s' to '%s'\n"
			, argv[0], errno, strerror(errno), tempfilename, wavfilename );
	else if (verbosity)
		fprintf( stderr, "Renamed output file into '%s'\n", wavfilename );

	if (inpfile != stdin)
		fclose(inpfile);

	return 0;
}

/* vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab */
