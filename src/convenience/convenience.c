/*
 * Copyright (C) 2014 by Kyle Keen <keenerd@gmail.com>
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

/* a collection of user friendly tools
 * todo: use strtol for more flexible int parsing
 * */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <time.h>
#include <assert.h>

#ifndef _WIN32
#include <unistd.h>
#include <sys/time.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include <process.h>
#define _USE_MATH_DEFINES
#endif

#include <math.h>

#include "rtl-sdr.h"

double atofs(char *s)
/* standard suffixes */
{
	char last;
	int len;
	double suff = 1.0;
	len = strlen(s);
	/* allow formatting spaces from .csv command file */
	while ( len > 1 && isspace(s[len-1]) )	--len;
	last = s[len-1];
	s[len-1] = '\0';
	switch (last) {
		case 'g':
		case 'G':
			suff *= 1e3;
			/* fall-through */
		case 'm':
		case 'M':
			suff *= 1e3;
			/* fall-through */
		case 'k':
		case 'K':
			suff *= 1e3;
			suff *= atof(s);
			s[len-1] = last;
			return suff;
	}
	s[len-1] = last;
	return atof(s);
}

double atoft(char *s)
/* time suffixes, returns seconds */
{
	char last;
	int len;
	double suff = 1.0;
	len = strlen(s);
	last = s[len-1];
	s[len-1] = '\0';
	switch (last) {
		case 'h':
		case 'H':
			suff *= 60;
			/* fall-through */
		case 'm':
		case 'M':
			suff *= 60;
			/* fall-through */
		case 's':
		case 'S':
			suff *= atof(s);
			s[len-1] = last;
			return suff;
	}
	s[len-1] = last;
	return atof(s);
}

double atofp(char *s)
/* percent suffixes */
{
	char last;
	int len;
	double suff = 1.0;
	len = strlen(s);
	last = s[len-1];
	s[len-1] = '\0';
	switch (last) {
		case '%':
			suff *= 0.01;
			suff *= atof(s);
			s[len-1] = last;
			return suff;
	}
	s[len-1] = last;
	return atof(s);
}

int nearest_gain(rtlsdr_dev_t *dev, int target_gain)
{
	int i, r, err1, err2, count, nearest;
	int* gains;
	r = rtlsdr_set_tuner_gain_mode(dev, 1);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to enable manual gain.\n");
		return r;
	}
	count = rtlsdr_get_tuner_gains(dev, NULL);
	if (count <= 0) {
		return 0;
	}
	gains = malloc(sizeof(int) * count);
	count = rtlsdr_get_tuner_gains(dev, gains);
	nearest = gains[0];
	for (i=0; i<count; i++) {
		err1 = abs(target_gain - nearest);
		err2 = abs(target_gain - gains[i]);
		if (err2 < err1) {
			nearest = gains[i];
		}
	}
	free(gains);
	return nearest;
}

int verbose_set_frequency(rtlsdr_dev_t *dev, uint32_t frequency)
{
	int r;
	r = rtlsdr_set_center_freq(dev, frequency);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set center freq.\n");
	} else {
		fprintf(stderr, "Tuned to %u Hz.\n", frequency);
	}
	return r;
}

int verbose_set_sample_rate(rtlsdr_dev_t *dev, uint32_t samp_rate)
{
	int r;
	r = rtlsdr_set_sample_rate(dev, samp_rate);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set sample rate.\n");
	} else {
		fprintf(stderr, "Sampling at %u S/s.\n", samp_rate);
	}
	return r;
}

int verbose_set_bandwidth(rtlsdr_dev_t *dev, uint32_t bandwidth)
{
	int r;
	uint32_t applied_bw = 0;
	/* r = rtlsdr_set_tuner_bandwidth(dev, bandwidth); */
	r = rtlsdr_set_and_get_tuner_bandwidth(dev, bandwidth, &applied_bw, 1 /* =apply_bw */);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set bandwidth.\n");
	} else if (bandwidth > 0) {
		if (applied_bw)
			fprintf(stderr, "Bandwidth parameter %u Hz resulted in %u Hz.\n", bandwidth, applied_bw);
		else
			fprintf(stderr, "Set bandwidth parameter %u Hz.\n", bandwidth);
	} else {
		fprintf(stderr, "Bandwidth set to automatic resulted in %u Hz.\n", applied_bw);
	}
	return r;
}

int verbose_direct_sampling(rtlsdr_dev_t *dev, int on)
{
	int r;
	r = rtlsdr_set_direct_sampling(dev, on);
	if (r != 0) {
		fprintf(stderr, "WARNING: Failed to set direct sampling mode.\n");
		return r;
	}
	if (on == 0) {
		fprintf(stderr, "Direct sampling mode disabled.\n");}
	if (on == 1) {
		fprintf(stderr, "Enabled direct sampling mode, input 1/I.\n");}
	if (on == 2) {
		fprintf(stderr, "Enabled direct sampling mode, input 2/Q.\n");}
	return r;
}

int verbose_offset_tuning(rtlsdr_dev_t *dev)
{
	int r;
	r = rtlsdr_set_offset_tuning(dev, 1);
	if (r != 0) {
		if ( r == -2 )
			fprintf(stderr, "WARNING: Failed to set offset tuning: tuner doesn't support offset tuning!\n");
		else if ( r == -3 )
			fprintf(stderr, "WARNING: Failed to set offset tuning: direct sampling not combinable with offset tuning!\n");
		else
			fprintf(stderr, "WARNING: Failed to set offset tuning.\n");
	} else {
		fprintf(stderr, "Offset tuning mode enabled.\n");
	}
	return r;
}

int verbose_auto_gain(rtlsdr_dev_t *dev)
{
	int r;
	r = rtlsdr_set_tuner_gain_mode(dev, 0);
	if (r != 0) {
		fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
	} else {
		fprintf(stderr, "Tuner gain set to automatic.\n");
	}
	return r;
}

int verbose_gain_set(rtlsdr_dev_t *dev, int gain)
{
	int r;
	r = rtlsdr_set_tuner_gain_mode(dev, 1);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to enable manual gain.\n");
		return r;
	}
	r = rtlsdr_set_tuner_gain(dev, gain);
	if (r != 0) {
		fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
	} else {
		fprintf(stderr, "Tuner gain set to %0.2f dB.\n", gain/10.0);
	}
	return r;
}

int verbose_ppm_set(rtlsdr_dev_t *dev, int ppm_error)
{
	int r;
	if (ppm_error == 0) {
		return 0;}
	r = rtlsdr_set_freq_correction(dev, ppm_error);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set ppm error.\n");
	} else {
		fprintf(stderr, "Tuner error set to %i ppm.\n", ppm_error);
	}
	return r;
}

int verbose_reset_buffer(rtlsdr_dev_t *dev)
{
	int r;
	r = rtlsdr_reset_buffer(dev);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to reset buffers.\n");}
	return r;
}

int verbose_device_search(char *s)
{
	int i, device_count, device, offset;
	char *s2;
	char vendor[256], product[256], serial[256];
	device_count = rtlsdr_get_device_count();
	if (!device_count) {
		fprintf(stderr, "No supported devices found.\n");
		return -1;
	}
	fprintf(stderr, "Found %d device(s):\n", device_count);
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
	}
	fprintf(stderr, "\n");
	/* does string look like raw id number */
	device = (int)strtol(s, &s2, 0);
	if (s2[0] == '\0' && device >= 0 && device < device_count) {
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string exact match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (strcmp(s, serial) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string prefix match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (strncmp(s, serial, strlen(s)) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string suffix match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		offset = strlen(serial) - strlen(s);
		if (offset < 0) {
			continue;}
		if (strncmp(s, serial+offset, strlen(s)) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	fprintf(stderr, "No matching devices found.\n");
	return -1;
}

#ifndef _WIN32

void executeInBackground( char * file, char * args, char * searchStr[], char * replaceStr[] )
{
	pid_t pid;
	char * argv[256] = { NULL };
	int k, argc = 0;
	argv[argc++] = file;
	if (args) {
		argv[argc] = strtok(args, " ");
		while (argc < 256 && argv[argc]) {
			argv[++argc] = strtok(NULL, " ");
			for (k=0; argv[argc] && searchStr && replaceStr && searchStr[k] && replaceStr[k]; k++) {
				if (!strcmp(argv[argc], searchStr[k])) {
					argv[argc] = replaceStr[k];
					break;
				}
			}
		}
	}

	pid = fork();
	switch (pid)
	{
	case -1:
		/* Fork() has failed */
		fprintf(stderr, "error: fork for '%s' failed!\n", file);
		break;
	case 0:
		execvp(file, argv);
		fprintf(stderr, "error: execv of '%s' from within fork failed!\n", file);
		exit(10);
		break;
	default:
		/* This is processed by the parent */
		break;
	}
}

#else

void executeInBackground( char * file, char * args, char * searchStr[], char * replaceStr[] )
{
	char * argv[256] = { NULL };
	int k, argc = 0;
	argv[argc++] = file;
 	if (args) {
		argv[argc] = strtok(args, " \t");
		while (argc < 256 && argv[argc]) {
			argv[++argc] = strtok(NULL, " \t");
			for (k=0; argv[argc] && searchStr && replaceStr && searchStr[k] && replaceStr[k]; k++) {
				if (!strcmp(argv[argc], searchStr[k])) {
					argv[argc] = replaceStr[k];
					break;
				}
			}
		}
	}

	spawnvp(P_NOWAIT, file, argv);
}

#endif


#pragma pack(push)
#pragma pack(1)

typedef struct {
	uint16_t	wYear;			/* 1601 through 30827 */
	uint16_t	wMonth;			/* 1..12 */
	uint16_t	wDayOfWeek;		/* 0 .. 6: 0 == Sunday, .., 6 == Saturday */
	uint16_t	wDay;			/* 1 .. 31 */
	uint16_t	wHour;			/* 0 .. 23 */
	uint16_t	wMinute;		/* 0 .. 59 */
	uint16_t	wSecond;		/* 0 .. 59 */
	uint16_t	wMilliseconds;	/* 0 .. 999 */
} Wind_SystemTime;


typedef struct
{
	/* RIFF header */
	char		riffID[4];	/* "RIFF" string */
	uint32_t	riffSize;	/* full filesize - 8 bytes (maybe with some byte missing...) */
	char		waveID[4];	/* "WAVE" string */

	/* FMT header */
	char		fmtID[4];	/* = "FMT " */
	uint32_t	fmtSize;
	int16_t		wFormatTag;
	int16_t		nChannels;
	int32_t		nSamplesPerSec;
	int32_t		nAvgBytesPerSec;
	int16_t		nBlockAlign;
	int16_t		nBitsPerSample;

	/* auxi header - used by SpectraVue / rfspace / HDSDR / .. */
	char		auxiID[4];	/* ="auxi" (chunk rfspace) */
	uint32_t	auxiSize;
	Wind_SystemTime StartTime;
	Wind_SystemTime StopTime;
	uint32_t	centerFreq;		/* receiver center frequency */
	uint32_t	ADsamplerate;	/* A/D sample frequency before downsampling */
	uint32_t	IFFrequency;	/* IF freq if an external down converter is used */
	uint32_t	Bandwidth;		/* displayable BW if you want to limit the display to less than Nyquist band */
	int32_t		IQOffset;		/* DC offset of the I and Q channels in 1/1000's of a count */
	int32_t		Unused2;
	int32_t		Unused3;
	int32_t		Unused4;
	int32_t		Unused5;

	/* DATA header */
	char		dataID[4];
	uint32_t	dataSize;
} waveFileHeader;

static waveFileHeader waveHdr;

#pragma pack(pop)


uint32_t	waveDataSize = 0;
static int	waveHdrStarted = 0;

void waveSetTime(Wind_SystemTime *p)
{
	struct timeval tv;
	struct tm t;

	gettimeofday(&tv, NULL);
	p->wMilliseconds = tv.tv_usec / 1000;

#ifdef _WIN32
	t = *gmtime(&tv.tv_sec);
#else
	gmtime_r(&tv.tv_sec, &t);
#endif
	
	p->wYear = t.tm_year + 1900;	/* 1601 through 30827 */
	p->wMonth = t.tm_mon + 1;		/* 1..12 */
	p->wDayOfWeek = t.tm_wday;		/* 0 .. 6: 0 == Sunday, .., 6 == Saturday */
	p->wDay = t.tm_mday;			/* 1 .. 31 */
	p->wHour = t.tm_hour;			/* 0 .. 23 */
	p->wMinute = t.tm_min;			/* 0 .. 59 */
	p->wSecond = t.tm_sec;			/* 0 .. 59 */
}

void wavePrepareHeader(unsigned samplerate, unsigned freq, int bitsPerSample, int numChannels)
{
	int	bytesPerSample = bitsPerSample / 8;
	int bytesPerFrame = bytesPerSample * numChannels;

	strncpy( waveHdr.riffID, "RIFF", 4 );
	waveHdr.riffSize = sizeof(waveFileHeader) - 8;		/* to fix */
	strncpy( waveHdr.waveID, "WAVE", 4 );

	strncpy( waveHdr.fmtID, "fmt ", 4 );
	waveHdr.fmtSize = 16;
	waveHdr.wFormatTag = 1;					/* PCM */
	waveHdr.nChannels = numChannels;		/* I and Q channels */
	waveHdr.nSamplesPerSec = samplerate;
	waveHdr.nAvgBytesPerSec = samplerate * bytesPerFrame;
	waveHdr.nBlockAlign = waveHdr.nChannels;
	waveHdr.nBitsPerSample = bitsPerSample;

	strncpy( waveHdr.auxiID, "auxi", 4 );
	waveHdr.auxiSize = 2 * sizeof(Wind_SystemTime) + 9 * sizeof(int32_t);  /* = 2 * 16 + 9 * 4 = 68 */
	waveSetTime( &waveHdr.StartTime );
	waveHdr.StopTime = waveHdr.StartTime;		/* to fix */
	waveHdr.centerFreq = freq;
	waveHdr.ADsamplerate = samplerate;
	waveHdr.IFFrequency = 0;
	waveHdr.Bandwidth = 0;
	waveHdr.IQOffset = 0;
	waveHdr.Unused2 = 0;
	waveHdr.Unused3 = 0;
	waveHdr.Unused4 = 0;
	waveHdr.Unused5 = 0;

	strncpy( waveHdr.dataID, "data", 4 );
	waveHdr.dataSize = 0;		/* to fix later */
	waveDataSize = 0;
}

void waveWriteHeader(unsigned samplerate, unsigned freq, int bitsPerSample, int numChannels, FILE * f)
{
	if (f != stdout) {
		assert( !waveHdrStarted );
		wavePrepareHeader(samplerate, freq, bitsPerSample, numChannels);
		fwrite(&waveHdr, sizeof(waveFileHeader), 1, f);
		waveHdrStarted = 1;
	}
}

void waveFinalizeHeader(FILE * f)
{
	if (f != stdout) {
		assert( waveHdrStarted );
		waveSetTime( &waveHdr.StopTime );
		waveHdr.dataSize = waveDataSize;
		waveHdr.riffSize += waveDataSize;

		fseek(f, 0, SEEK_SET);
		fwrite(&waveHdr, sizeof(waveFileHeader), 1, f);
		waveHdrStarted = 0;
	}
}



// vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab
