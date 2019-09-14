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
