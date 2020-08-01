/*
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

#include "waveread.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
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

#include "wavehdr.h"

#define PRINT_DEBUG_OUTPUTS		0

static waveFileHeader waveHdr;

static uint32_t	waveDataSize = 0;


static void waveGetTimeInt(const Wind_SystemTime *p, time_t *tim, double *fraction)
{
	struct tm t;
#ifdef _WIN32
	struct tm gtm, ltm;
	time_t nt, gt, lt;
#endif
	/* fprintf(stderr, "waveGetTimeInt( Wind_SystemTime = %04d-%02d-%02d T %02d:%02d:%02d %f )\n"
		, (int)(p->wYear), (int)(p->wMonth), (int)(p->wDay)
		, (int)(p->wHour), (int)(p->wMinute), (int)(p->wSecond)
		, (float)(p->wMilliseconds / 1000.0) );
	*/

	t.tm_year = p->wYear - 1900;    /* 1601 through 30827 */
	t.tm_mon = p->wMonth -1; 		/* 1..12 */
	t.tm_wday = p->wDayOfWeek;      /* 0 .. 6: 0 == Sunday, .., 6 == Saturday */
	t.tm_mday = p->wDay;			/* 1 .. 31 */
	t.tm_hour = p->wHour;			/* 0 .. 23 */
	t.tm_min = p->wMinute;			/* 0 .. 59 */
	t.tm_sec = p->wSecond;			/* 0 .. 59 */
	t.tm_wday = 0;
	t.tm_yday = 0;
	t.tm_isdst = 0;
	*fraction = p->wMilliseconds / 1000.0;
#ifndef _WIN32
	*tim = timegm( &t );
	/* fprintf(stderr, "  using timegm()\n"); */
#else
	#ifdef _MSC_VER
		*tim = _mkgmtime(&t);
	/* fprintf(stderr, "  using _mkgmtime()\n"); */
	#else
		/* workaround missing mkgmtime on mingw */
		nt = mktime(&t);
		gtm = *gmtime(&nt);
		ltm = *localtime(&nt);
		gt = mktime(&gtm);
		lt = mktime(&ltm);
		assert( nt == gt );
		nt += ( lt - gt );
		*tim = nt;
		/* fprintf(stderr, "  using mktime(), gmtime() and localtime() difference\n"); */
	#endif
#endif
}



void waveGetStartTime(time_t *tim, double *fraction)
{
	waveGetTimeInt(&waveHdr.a.StartTime, tim, fraction);
}

void waveGetStopTime(time_t *tim, double *fraction)
{
	waveGetTimeInt(&waveHdr.a.StopTime, tim, fraction);
}


int  waveReadHeader(FILE * f, uint32_t *srate, uint32_t *freq, int *bitsPerSample, int *numChannels
	, uint32_t *nFrames, int16_t *formatTag, int verbosity)
{
	uint8_t buf[32768];
	size_t rd, smpSize;
	int readAuxiHdr = 0;
	int readDataHdr = 0;
	int rv = 0;

#if PRINT_DEBUG_OUTPUTS
	fprintf(stderr, "waveReadHeader(.., verbosity = %d)\n", verbosity);
#endif

	rd = fread( &waveHdr.r, sizeof(riff_chunk), 1, f );
	if ( rd != 1 )
		return 10;
	if ( memcmp(waveHdr.r.hdr.ID, "RIFF", 4) )
		return 11;
	if ( memcmp(waveHdr.r.waveID, "WAVE", 4 ) )
		return 12;

	rd = fread( &waveHdr.f, sizeof(fmt_chunk), 1, f );
#if PRINT_DEBUG_OUTPUTS
	fprintf(stderr, "read %d chunk '%c%c%c%c' size %u = 0x%X\n", (int)rd
		, waveHdr.f.hdr.ID[0], waveHdr.f.hdr.ID[1], waveHdr.f.hdr.ID[2], waveHdr.f.hdr.ID[3]
		, (unsigned)waveHdr.f.hdr.size, (unsigned)waveHdr.f.hdr.size );
#endif
	if ( rd != 1 )
		return 20;
	if ( memcmp(waveHdr.f.hdr.ID, "fmt ", 4 ) )
		return 21;
	if ( waveHdr.f.hdr.size != 16 )
		return 22;
	*bitsPerSample = waveHdr.f.nBitsPerSample;
	*numChannels = waveHdr.f.nChannels;
	*srate = waveHdr.f.nSamplesPerSec;
	*formatTag = waveHdr.f.wFormatTag;


	rd = fread( &waveHdr.a.hdr, sizeof(chunk_hdr), 1, f );
	while ( !readDataHdr )
	{
		if ( rd != 1 )
			return 30;
	#if PRINT_DEBUG_OUTPUTS
		fprintf(stderr, "read %d chunk '%c%c%c%c' size %u = 0x%X\n", (int)rd
			, waveHdr.a.hdr.ID[0], waveHdr.a.hdr.ID[1], waveHdr.a.hdr.ID[2], waveHdr.a.hdr.ID[3]
			, (unsigned)waveHdr.a.hdr.size, (unsigned)waveHdr.a.hdr.size );
	#endif
		if ( !memcmp(waveHdr.a.hdr.ID, "auxi", 4 ) )
		{
			if ( verbosity )
				fprintf(stderr, "read chunk 'auxi' .. continue searching for 'data'-chunk ..\n");
			if ( waveHdr.a.hdr.size > sizeof(buf) )
				return 32;
			if ( waveHdr.a.hdr.size < (sizeof(auxi_chunk) - sizeof(chunk_hdr)) )
				return 33;
			rd = fread( buf, waveHdr.a.hdr.size, 1, f );
			if ( rd != 1 )
				return 34;
			memcpy( &waveHdr.a.StartTime, buf, sizeof(auxi_chunk) - sizeof(chunk_hdr));
			*freq = waveHdr.a.centerFreq;
			readAuxiHdr = 1;
		}
		else if ( !memcmp(waveHdr.a.hdr.ID, "data", 4 ) )
		{
			if ( verbosity )
				fprintf(stderr, "read chunk 'data' .. going to process - %s 'auxi'-chunk ..\n", readAuxiHdr ? "with" : "without" );
			waveHdr.d.hdr = waveHdr.a.hdr;
			if (!readAuxiHdr) {
				memset(&waveHdr.a, 0, sizeof(auxi_chunk));
				waveHdr.a.StartTime.wYear = 1970;
				waveHdr.a.StartTime.wMonth = 1;
				waveHdr.a.StartTime.wDay = 1;
				*freq = 100UL * 1000 * 1000;	/* set frequency to 100 MHz */
			}
			readDataHdr = 1;
			break;
		}
		else
		{
			if ( verbosity )
				fprintf(stderr, "read chunk '%c%c%c%c' while expecting 'auxi' or 'data'. going to next chunk ..\n"
					, waveHdr.a.hdr.ID[0], waveHdr.a.hdr.ID[1], waveHdr.a.hdr.ID[2], waveHdr.a.hdr.ID[3] );
			if ( waveHdr.a.hdr.size > sizeof(buf) )
				return 32;
			rd = fread( buf, waveHdr.a.hdr.size, 1, f );
			if ( rd != 1 )
				return 34;
		}

		/* read next chunk's header */
		rd = fread( &waveHdr.a.hdr, sizeof(chunk_hdr), 1, f );
	}

	if ( !readDataHdr )
	{
		rd = fread( &waveHdr.d.hdr, sizeof(chunk_hdr), 1, f );
	#if PRINT_DEBUG_OUTPUTS
		fprintf(stderr, "read %d chunk '%c%c%c%c' size %u = 0x%X\n", (int)rd
			, waveHdr.d.hdr.ID[0], waveHdr.d.hdr.ID[1], waveHdr.d.hdr.ID[2], waveHdr.d.hdr.ID[3]
			, (unsigned)waveHdr.d.hdr.size, (unsigned)waveHdr.d.hdr.size );
	#endif
	}
	else
		rd = 1;
	if ( rd != 1 )
		return 40;
	if ( memcmp(waveHdr.d.hdr.ID, "data", 4 ) )
		return 41;

	smpSize = (*bitsPerSample + 7) / 8;		/* round up to next byte */
	smpSize *= *numChannels;
	*nFrames = waveHdr.d.hdr.size / smpSize;

	if ( verbosity >= 2 )
	{
		fprintf(stderr, "riffSize = %lu\n", (unsigned long)waveHdr.r.hdr.size );
		fprintf(stderr, "dataSize = %lu\n", (unsigned long)waveHdr.d.hdr.size);
		fprintf(stderr, "nBlockAlign = %d\n", (int)waveHdr.f.nBlockAlign);
		fprintf(stderr, "smpSize = %d\n", (int)smpSize);
		fprintf(stderr, "*nFrames = %lu\n", (unsigned long)(*nFrames) );
	}

	return rv;
}


int  waveReadSamples(FILE* f,  void * vpData, size_t numSamples, int needCleanData, size_t *numRead)
{
	size_t nw;
	switch (waveHdr.f.nBitsPerSample)
	{
	case 0:
	default:
		return 1;
	case 8:
		/* no endian conversion needed for single bytes */
		nw = fread(vpData, sizeof(uint8_t), numSamples, f);
		*numRead = nw;
		return (nw == numSamples) ? 0 : 1;
	case 16:
		/* TODO: endian conversion needed */
		nw = fread(vpData, sizeof(int16_t), numSamples, f);
		if ( needCleanData )
		{
			/* TODO: convert back endianness */
		}
		*numRead = nw;
		return (nw == numSamples) ? 0 : 1;
	}
}

int  waveReadFrames(FILE* f,  void * vpData, size_t numFrames, int needCleanData, size_t *numRead)
{
	size_t nw;
	switch (waveHdr.f.nBitsPerSample)
	{
	case 0:
	default:
		return 1;
	case 8:
		/* no endian conversion needed for single bytes */
		nw = fread(vpData, waveHdr.f.nChannels * sizeof(uint8_t), numFrames, f);
		*numRead = nw;
		return (nw == numFrames) ? 0 : 1;
	case 16:
		/* TODO: endian conversion needed */
		nw = fread(vpData, waveHdr.f.nChannels * sizeof(int16_t), numFrames, f);
		if ( needCleanData )
		{
			/* TODO: convert back endianness */
		}
		*numRead = nw;
		return (nw == numFrames) ? 0 : 1;
	}
}

// vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab
