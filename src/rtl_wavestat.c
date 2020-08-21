/*
 * rtl-wavestat, display wave file meta information
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
#include "convenience/waveread.h"


static volatile int do_exit = 0;
static int verbosity = 0;


void usage(void)
{
	fprintf(stderr,
		"rtl_wavestat, display wave file meta information\n"
		"rtl_wavestat version %d.%d %s (%s)\n\n",
		APP_VER_MAJOR, APP_VER_MINOR, APP_VER_ID, __DATE__ );
	fprintf(stderr,
		"Use:\trtl_wavestat [-options] <input_wave_filename>\n"
		"\t-a  print all information\n"
		"\t-r  print without field name\n"
		"\t-f  print center frequency in Hz\n"
		"\t-s  print samplerate in Hz\n"
		"\t-c  print number of channels\n"
		"\t-b  print number of bits per sample\n"
		"\t-F  print sample Format\n"
		"\t-u  print start time in UTC:       'yyy-mm-ddThh:mm:dd.zzz'\n"
		"\t-t  print start time in localtime: 'yyy-mm-ddThh:mm:dd.zzz'\n"
		"\t-z  print start time in seconds since 1970-01-01T00:00:00.000\n"
		"\t-d  print file duration in frames (= num samples per channel)\n"
		"\t-D  print file duration in seconds\n"
		"\t-w  input file\n"
		"\t-v  verbose output\n" );
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
	const char * wavfilename = NULL;
	FILE * inpfile = stdin;
	uint32_t freq = 0;
	uint32_t srate = 0;
	int nChan = 0;
	int nBits = 0;
	int16_t formatTag;
	uint32_t numFrames;
	time_t tim = 0;
	double fraction = 0.0;
	struct tm *ptm = NULL;
	int printAll = 0;
	int printFieldName = 1;
	int printFreq = 0;
	int printSRate = 0;
	int printNchan = 0;
	int printNbits = 0;
	int printSmpFmt = 0;
	int printStartZ = 0;
	int printStartL = 0;
	int printStartUnix = 0;
	int printDurationSmp = 0;
	int printDurationTim = 0;

	while ((opt = getopt(argc, argv, "arfscbFutzdDvhw:")) != -1) {
		switch (opt) {
		case 'a':	printAll = 1;	break;
		case 'r':	printFieldName = 0;	break;
		case 'f':	printFreq = 1;	break;
		case 's':	printSRate = 1;	break;
		case 'c':	printNchan = 1;	break;
		case 'b':	printNbits = 1;	break;
		case 'F':	printSmpFmt = 1;	break;
		case 'u':	printStartZ = 1;	break;
		case 't':	printStartL = 1;	break;
		case 'z':	printStartUnix = 1;	break;
		case 'd':	printDurationSmp = 1;	break;
		case 'D':	printDurationTim = 1;	break;
		case 'w':	wavfilename = optarg;	break;
		case 'v':	++verbosity;	break;
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
		wavfilename = argv[optind];
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

	if (!wavfilename || !strcmp(wavfilename, "-")) {
#ifdef _WIN32
		_setmode(_fileno(stdin), _O_BINARY);
#endif
		inpfile = stdin;
		if (verbosity)
			fprintf(stderr, "Using stdin as input\n");
	} else {
		if (verbosity >= 2)
			fprintf(stderr, "Opening input '%s'\n", wavfilename);
		inpfile = fopen(wavfilename, "rb");
		if (!inpfile) {
			fprintf(stderr, "Error: Failed to open input %s\n", wavfilename);
			exit(1);
		}
		if (verbosity)
			fprintf(stderr, "Opened '%s' for input\n", wavfilename);
	}

	r = waveReadHeader(inpfile, &srate, &freq, &nBits, &nChan, &numFrames, &formatTag, verbosity);
	if ( r >= 10 ) {
		fprintf(stderr, "Error %d reading/evaluating wave file header\n", r);
		exit(1);
	} else if ( verbosity >= 2 ) {
		fprintf(stderr, "Success reading/evaluating wave file header\n");
	}

	if ( !printFreq && !printSRate && !printNchan && !printNbits && !printSmpFmt && !printStartZ && !printStartL
		&& !printDurationSmp && !printDurationTim ) {
		printAll = 1;
	}

	if ( printAll || printFreq ) {
		if ( printFieldName )
			fprintf(stdout, "frequency/Hz:\t");
		fprintf(stdout, "%lu\n", (unsigned long)freq);
	}
	if ( printAll || printSRate ) {
		if ( printFieldName )
			fprintf(stdout, "samplerate/Hz:\t");
		fprintf(stdout, "%lu\n", (unsigned long)srate);
	}
	if ( printAll || printNchan ) {
		if ( printFieldName )
			fprintf(stdout, "num_channels:\t");
		fprintf(stdout, "%d\n", nChan);
	}
	if ( printAll || printNbits ) {
		if ( printFieldName )
			fprintf(stdout, "bits_per_sample:\t");
		fprintf(stdout, "%d\n", nBits);
	}
	if ( printAll || printSmpFmt ) {
		if ( printFieldName )
			fprintf(stdout, "sample_format:\t");
		if (formatTag == 0x0001)
			fprintf(stdout, "0x%04X\tPCM\n", (unsigned)formatTag);
		else if (formatTag == 0x0003)
			fprintf(stdout, "0x%04X\tIEEE_FLOAT\n", (unsigned)formatTag);
		else
			fprintf(stdout, "0x%04X\n", (unsigned)formatTag);
	}
	if ( printAll || printStartZ ) {
		waveGetStartTime(&tim, &fraction);
		ptm = gmtime(&tim);
		if (ptm) {
			if ( printFieldName )
				fprintf(stdout, "time_start/Z:\t");
			fprintf(stdout, "%04d-%02d-%02dT%02d:%02d:%02d.%03d\n"
				, ptm->tm_year+1900, ptm->tm_mon+1, ptm->tm_mday
				, ptm->tm_hour, ptm->tm_min, ptm->tm_sec, (int)(fraction*1000.0) );
		}
	}
	if ( printAll || printStartL ) {
		waveGetStartTime(&tim, &fraction);
		ptm = localtime(&tim);
		if (ptm) {
			if ( printFieldName )
				fprintf(stdout, "time_start/L:\t");
			fprintf(stdout, "%04d-%02d-%02dT%02d:%02d:%02d.%03d\n"
				, ptm->tm_year+1900, ptm->tm_mon+1, ptm->tm_mday
				, ptm->tm_hour, ptm->tm_min, ptm->tm_sec, (int)(fraction*1000.0) );
		}
	}
	if ( printAll || printStartUnix ) {
		tim = 0;
		fraction = 0;
		waveGetStartTime(&tim, &fraction);
		if ( printFieldName )
			fprintf(stdout, "time_start/secs (unix):\t");
		fraction += (double)tim;
		fprintf(stdout, "%f\n", fraction );
	}
	if ( printAll || printDurationSmp ) {
		if ( printFieldName )
			fprintf(stdout, "duration/frames:\t");
		fprintf(stdout, "%lu\n", (unsigned long)numFrames);
	}
	if ( printAll || printDurationTim ) {
		if ( printFieldName )
			fprintf(stdout, "duration/secs:\t");
		fprintf(stdout, "%f\n", (double)numFrames/(double)srate);
	}

	if (inpfile != stdin)
		fclose(inpfile);

	return 0;
}

/* vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab */
