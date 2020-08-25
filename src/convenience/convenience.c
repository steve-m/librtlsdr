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


static struct tm * str_to_tm( const char * str, struct tm * t, double * fraction ) {
	char b[16];
	int k, v;
	/* 0         1         2   */
	/* 01234567890123456789012 */
	/* 2019-09-15T01:53:20.234 - mostly ISO 8601 */

	*fraction = 0.0;
	t->tm_sec	= 0;
	t->tm_min	= 0;
	t->tm_hour	= 0;
	t->tm_mday	= 1;
	t->tm_mon	= 0;
	t->tm_year	= 0;
	t->tm_wday	= 0;
	t->tm_yday	= 0;
	t->tm_isdst	= -1;

	/* date */
	if ( (str[4] == '-' || str[4] == '/') && str[4] == str[7] ) {
		/* year */
		b[4] = 0;	for ( k = 0; k < 4; ++k )	b[k] = str[k];
		v = atoi(b);
		t->tm_year = v - 1900;
		/* month */
		b[2] = 0;	for ( k = 0; k < 2; ++k )	b[k] = str[5+k];
		v = atoi(b);
		if (v < 1 || v > 12)
			return NULL;
		t->tm_mon = v - 1;
		/* day */
		b[2] = 0;	for ( k = 0; k < 2; ++k )	b[k] = str[8+k];
		v = atoi(b);
		if (v < 1 || v > 31)
			return NULL;
		t->tm_mday = v;
	} else
		return NULL;

	if (str[10] == 0 )
		return t;

	/* time */
	if ( str[10] != 'T' && str[10] != ' ' && str[10] != '_' )
		return NULL;
	if ( (str[13] == ':' || str[13] == '/') && str[13] == str[16] ) {
		/* hour */
		b[2] = 0;	for ( k = 0; k < 2; ++k )	b[k] = str[11+k];
		v = atoi(b);
		if (v < 0 || v > 23)
			return NULL;
		t->tm_hour = v;
		/* minute */
		b[2] = 0;	for ( k = 0; k < 2; ++k )	b[k] = str[14+k];
		v = atoi(b);
		if (v < 0 || v > 59)
			return NULL;
		t->tm_min = v;
		/* second */
		b[2] = 0;	for ( k = 0; k < 2; ++k )	b[k] = str[17+k];
		v = atoi(b);
		if (v < 0 || v > 61)
			return NULL;
		t->tm_sec = v;
	} else
		return NULL;

	if (str[19] == 0 )
		return t;

	/* fraction */
	if ( str[19] == '.' || str[19] == ',' ) {
		for ( k = 0; k < 16; ++k )	b[k] = 0;
		strcpy(b, "0.");
		strncpy(&b[2], &str[20], 12);
		*fraction = atof(b);
		return t;
	}

	/* return t anyway .. without fraction */
	return t;
}


time_t utctimestr_to_time(const char * str, double * fraction) {
	struct tm t;
	struct tm *p;
#ifdef _WIN32
	struct tm gtm;
	struct tm ltm;
	time_t nt;
	time_t gt;
	time_t lt;
#endif
	p = str_to_tm( str, &t, fraction );
	if (!p)
		return 0;
	p->tm_isdst = 0;
#ifndef _WIN32
	return timegm(p);
#else
	#ifdef _MSC_VER
		return _mkgmtime(p);
	#else
		/* workaround missing mkgmtime on mingw */
		nt = mktime(p);
		gtm = *gmtime(&nt);
		ltm = *localtime(&nt);
		gt = mktime(&gtm);
		lt = mktime(&ltm);
		assert( nt == gt );
		nt += ( lt - gt );
		return nt;
	#endif
#endif
}


time_t localtimestr_to_time(const char * str, double * fraction) {
	struct tm t;
	struct tm *p;

	p = str_to_tm( str, &t, fraction );
	
	if (!p)
		return 0;
#ifndef _WIN32
	return timelocal(p);
#else
	return mktime(p);
#endif
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


// vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab
