/*
 * rtl-wavestream, stream raw data (in specified sample format)
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

#define BLOCKLEN 65536

/* read up to 64K samples */
static uint8_t inpBuffer[BLOCKLEN * sizeof(int32_t)];
/* output is max 4 times bigger: uint8_t -> int32_t */
static uint8_t outBuffer[BLOCKLEN * sizeof(int32_t) * sizeof(int32_t)];

void usage(void)
{
	fprintf(stderr,
		"rtl_wavestream, stream raw data (in specified format)\n"
		"rtl_wavestream version %d.%d %s (%s)\n\n",
		APP_VER_MAJOR, APP_VER_MINOR, APP_VER_ID, __DATE__ );
	fprintf(stderr,
		"Usage:\trtl_wavestream [-options] <input_wave_filename>\n"
		"\t-f <fmt>  sample format for output. default = input format\n"
		"\t            supported formats: 'PCM16'/'PCM' or 'FLOAT32'/'FLOAT'\n"
		"\t-w  input file\n"
		"\t-v        verbose output\n" );
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
	const char * targetFmtStr = "pcm16";
	FILE * inpfile = NULL;
	uint32_t freq = 0;
	uint32_t srate = 0;
	int nChan = 0;
	int nBits = 0;
	int targetFmt = 0;  /* PCM16 = 0, FLOAT32 = 1 */
	int inputFmt = 0;   /* PCM16 = 0, FLOAT32 = 1 */
	int16_t formatTag;
	uint32_t numFrames;

	while ((opt = getopt(argc, argv, "f:w:vh")) != -1) {
		switch (opt) {
		case 'f':	targetFmtStr = optarg;		break;
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

	if ( !strcmp(targetFmtStr, "pcm") || !strcmp(targetFmtStr, "pcm16")
		|| !strcmp(targetFmtStr, "PCM") || !strcmp(targetFmtStr, "PCM16") )
	{
		targetFmt = 0;  /* PCM16 = 0, FLOAT32 = 1 */
		if (verbosity)
			fprintf(stderr, "target sample format: PCM16\n");
	}
	else if ( !strcmp(targetFmtStr, "flt32") || !strcmp(targetFmtStr, "float32") || !strcmp(targetFmtStr, "float")
		|| !strcmp(targetFmtStr, "FLT32") || !strcmp(targetFmtStr, "FLOAT32") || !strcmp(targetFmtStr, "FLOAT") )
	{
		targetFmt = 1;  /* PCM16 = 0, FLOAT32 = 1 */
		if (verbosity)
			fprintf(stderr, "target sample format: FLOAT32\n");
	}
	else
	{
		fprintf(stderr, "Error: unsupported target format. accepting 'PCM16'/'PCM' or 'FLOAT32'/'FLOAT'\n");
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

#ifdef _WIN32
		_setmode(_fileno(stdout), _O_BINARY);
#endif

	if (!wavfilename) {
		fprintf(stderr, "Error: No input file specified!\n");
		exit(1);
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
	} else if ( verbosity >= 2 ) {
		fprintf(stderr, "Success reading/evaluating wave file header\n");
	}

	if ( verbosity ) {
		fprintf(stderr, "frequency/Hz:\t%lu\n", (unsigned long)freq);
		fprintf(stderr, "samplerate/Hz:\t%lu\n", (unsigned long)srate);
		fprintf(stderr, "num_channels:\t%d\n", nChan);
		fprintf(stderr, "bits_per_sample:\t%d\n", nBits);
		if (formatTag == 0x0001)
			fprintf(stderr, "sample_format:\t0x%04X\tPCM\n", (unsigned)formatTag);
		else if (formatTag == 0x0003)
			fprintf(stderr, "sample_format:\t0x%04X\tIEEE_FLOAT\n", (unsigned)formatTag);
		else
			fprintf(stderr, "sample_format:\t0x%04X\n", (unsigned)formatTag);
		fprintf(stderr, "duration/frames:\t%lu\t0x%lX\n", (unsigned long)numFrames, (unsigned long)numFrames);
		fprintf(stderr, "duration/secs:\t%f\n", (double)numFrames/(double)srate);
	}

	if ( formatTag == 0x0001 && nBits == 16 ) {
		inputFmt = 0;
		if (verbosity)
			fprintf(stderr, "input sample format: PCM16\n");
	}
	else if ( formatTag == 0x0003 && nBits == 32 ) {
		inputFmt = 1;
		if (verbosity)
			fprintf(stderr, "input sample format: FLOAT32\n");
	}
	else
	{
		fprintf(stderr, "Error: unsupported input format. only 'PCM16' and 'FLOAT32' supported.\n");
		exit(1);
	}

	{
		void * pvInp = &inpBuffer[0];
		void * pvOut = &outBuffer[0];
		const size_t numFramesPerRead = BLOCKLEN / nChan;
		const size_t numSmpPerRead = numFramesPerRead * nChan;
		const size_t inpSmpSize = (inputFmt  == 0) ? sizeof(int16_t) : sizeof(float);
		const size_t outSmpSize = (targetFmt == 0) ? sizeof(int16_t) : sizeof(float);
		size_t numSamples = numFrames * nChan;
		size_t readTotal = 0;
		size_t numRead;

		if ( verbosity )
		{
			fprintf(stderr, "input  sample size = %u\n", (unsigned)inpSmpSize);
			fprintf(stderr, "output sample size = %u\n", (unsigned)outSmpSize);
			fprintf(stderr, "samples per read = %u smp\n", (unsigned)numSmpPerRead);
		}

		while ( !do_exit )
		{
			const size_t numToRead = numSmpPerRead;
			const size_t readErr = waveReadSamples(inpfile, pvInp, numToRead, 0, &numRead);
			if ( numRead != numToRead )
				fprintf(stderr, "Error: reading %lu delivered %lu smp after %lu smp - left %lu frames\n"
					, (unsigned long)numToRead, (unsigned long)numRead
					, (unsigned long)readTotal, (unsigned long)(numSamples / nChan) );
			if ( readErr )
			{
				fprintf(stderr, "Error reading samples after %lu smp - left %lu frames\n"
					, (unsigned long)readTotal, (unsigned long)(numSamples / nChan) );
			}
			else if ( verbosity >= 2 )
				fprintf(stderr, "read %lu samples: left frames: %lu\n"
					, (unsigned long)numToRead, (unsigned long)numSamples);
			if ( !numRead )
				break;

			if ( inputFmt == targetFmt ) {
				size_t w = fwrite(pvInp, inpSmpSize, numRead, stdout);
				if ( w != numRead ) {
					fprintf(stderr, "Error writing read samples after %lu smp - left %lu frames\n"
						, (unsigned long)readTotal, (unsigned long)(numSamples / nChan) );
					break;
				}
			}
			else if ( inputFmt == 0 && targetFmt == 1 ) {
				const int16_t *ai = (const int16_t*)pvInp;
				float * ao = (float*)pvOut;
				size_t w, k;
				for ( k = 0; k < numRead; ++k )
					ao[k] = ai[k] * (1.0F / 32768.0F);
				w = fwrite(pvOut, outSmpSize, numRead, stdout);
				if ( w != numRead ) {
					fprintf(stderr, "Error writing converted samples after %lu smp - left %lu frames\n"
						, (unsigned long)readTotal, (unsigned long)(numSamples / nChan) );
					break;
				}
			}
			else if ( inputFmt == 1 && targetFmt == 0 ) {
				const float *ai = (const float*)pvInp;
				int16_t * ao = (int16_t*)pvOut;
				size_t w, k;
				for ( k = 0; k < numRead; ++k )
					ao[k] = (int16_t)( ai[k] * 32768.0F );
				w = fwrite(pvOut, outSmpSize, numRead, stdout);
				if ( w != numRead ) {
					fprintf(stderr, "Error writing converted samples after %lu smp - left %lu frames\n"
						, (unsigned long)readTotal, (unsigned long)(numSamples / nChan) );
					break;
				}
			}
			numSamples -= numRead;
			readTotal += numRead;
		}

		if ( verbosity )
		{
			fprintf(stderr, "Written %lu samples in total - left %lu of %lu frames\n"
				, (unsigned long)readTotal, (unsigned long)(numSamples / nChan), (unsigned long)(numFrames * nChan) );
		}
	}

	fclose(inpfile);

	return 0;
}

/* vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab */
