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
#ifndef __WAVEREAD_H
#define __WAVEREAD_H

#include <stdint.h>
#include <stdio.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif


int  waveReadHeader(FILE * f, uint32_t *samplerate, uint32_t *freq, int *bitsPerSample, int *numChannels
	, uint32_t *nFrames, int16_t *formatTag, int verbosity);
int  waveReadFrames(FILE* f,  void * vpData, size_t numFrames, int needCleanData, size_t *numRead);
int  waveReadSamples(FILE* f,  void * vpData, size_t numSamples, int needCleanData, size_t *numRead);  /* returns 0, when no errors occured */
void waveGetStartTime(time_t *tim, double *fraction);
void waveGetStopTime(time_t *tim, double *fraction);

#ifdef __cplusplus
}
#endif

#endif /*__WAVEREAD_H*/
