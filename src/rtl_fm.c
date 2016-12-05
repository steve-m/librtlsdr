/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 * Copyright (C) 2015 by Hayati Ayguen <h_ayguen@web.de>
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


/*
 * written because people could not do real time
 * FM demod on Atom hardware with GNU radio
 * based on rtl_sdr.c and rtl_tcp.c
 *
 * lots of locks, but that is okay
 * (no many-to-many locks)
 *
 * todo:
 *	   sanity checks
 *	   scale squelch to other input parameters
 *	   test all the demodulations
 *	   pad output on hop
 *	   frequency ranges could be stored better
 *	   scaled AM demod amplification
 *	   auto-hop after time limit
 *	   peak detector to tune onto stronger signals
 *	   fifo for active hop frequency
 *	   clips
 *	   noise squelch
 *	   merge stereo patch
 *	   merge soft agc patch
 *	   merge udp patch
 *	   testmode to detect overruns
 *	   watchdog to reset bad dongle
 *	   fix oversampling
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
#define usleep(x) Sleep(x/1000)
#if defined(_MSC_VER) && (_MSC_VER < 1900)
#define snprintf _snprintf
#endif
#if defined(_MSC_VER) && (_MSC_VER < 1800)
#define round(x) (x > 0.0 ? floor(x + 0.5): ceil(x - 0.5))
#endif
#define _USE_MATH_DEFINES
#endif

#ifdef NEED_PTHREADS_WORKARROUND
#define HAVE_STRUCT_TIMESPEC
#endif

#include <math.h>
#include <pthread.h>
#include <libusb.h>

#include "rtl-sdr.h"
#include "convenience/convenience.h"

#define DEFAULT_SAMPLE_RATE		24000
#define DEFAULT_BUF_LENGTH		(1 * 16384)
#define MAXIMUM_OVERSAMPLE		16
#define MAXIMUM_BUF_LENGTH		(MAXIMUM_OVERSAMPLE * DEFAULT_BUF_LENGTH)
#define AUTO_GAIN				-100
#define DEFAULT_BUFFER_DUMP		4096

#define FREQUENCIES_LIMIT		1024

static int BufferDump = DEFAULT_BUFFER_DUMP;
static int OutputToStdout = 1;
static int MinCaptureRate = 1000000;

static volatile int do_exit = 0;
static int lcm_post[17] = {1,1,1,3,1,5,3,7,1,9,5,11,3,13,7,15,1};
static int ACTUAL_BUF_LENGTH;

static int *atan_lut = NULL;
static int atan_lut_size = 131072; /* 512 KB */
static int atan_lut_coef = 8;

static int verbosity = 0;
static int printLevels = 0;
static int printLevelNo = 1;
static int levelMax = 0;
static int levelMaxMax = 0;
static double levelSum = 0.0;

enum trigExpr { crit_IN =0, crit_OUT, crit_LT, crit_GT };
char * aCritStr[] = { "in", "out", "<", ">" };

struct cmd_state
{
	const char * filename;
	FILE * file;
	int lineNo;
	char acLine[4096];
	int checkADCmax;
	int checkADCrms;
	uint32_t prevFreq;
	int prevGain;
	uint32_t prevBandwidth;
	uint32_t freq;
	int gain;
	enum trigExpr trigCrit;
	double refLevel;
	double refLevelTol;
	int numMeas;
	int numBlockTrigger;
	char * command;
	char * args;
	double levelSum;
	int numSummed;
	int omitFirstFreqLevels;
	int waitTrigger[FREQUENCIES_LIMIT];
	int statNumLevels[FREQUENCIES_LIMIT];
	uint32_t statFreq[FREQUENCIES_LIMIT];
	double statSumLevels[FREQUENCIES_LIMIT];
	float statMinLevel[FREQUENCIES_LIMIT];
	float statMaxLevel[FREQUENCIES_LIMIT];
};

struct dongle_state
{
	int	  exit_flag;
	pthread_t thread;
	rtlsdr_dev_t *dev;
	int	  dev_index;
	uint32_t userFreq;
	uint32_t freq;
	uint32_t rate;
	uint32_t bandwidth;
	int	  gain;
	int16_t  buf16[MAXIMUM_BUF_LENGTH];
	uint32_t buf_len;
	int	  ppm_error;
	int	  offset_tuning;
	int	  direct_sampling;
	int	  mute;
	struct demod_state *demod_target;
	double samplePowSum;
	int samplePowCount;
	unsigned char sampleMax;
};

struct demod_state
{
	int	  exit_flag;
	pthread_t thread;
	int16_t  lowpassed[MAXIMUM_BUF_LENGTH];
	int	  lp_len;
	int16_t  lp_i_hist[10][6];
	int16_t  lp_q_hist[10][6];
	int16_t  result[MAXIMUM_BUF_LENGTH];
	int16_t  droop_i_hist[9];
	int16_t  droop_q_hist[9];
	int	  result_len;
	int	  rate_in;
	int	  rate_out;
	int	  rate_out2;
	int	  now_r, now_j;
	int	  pre_r, pre_j;
	int	  prev_index;
	int	  downsample;	/* min 1, max 256 */
	int	  post_downsample;
	int	  output_scale;
	int	  squelch_level, conseq_squelch, squelch_hits, terminate_on_squelch;
	int	  downsample_passes;
	int	  comp_fir_size;
	int	  custom_atan;
	int	  deemph, deemph_a;
	int	  now_lpr;
	int	  prev_lpr_index;
	int	  dc_block_audio, dc_avg, adc_block_const;
	int	  dc_block_raw, dc_avgI, dc_avgQ, rdc_block_const;
	void	 (*mode_demod)(struct demod_state*);
	pthread_rwlock_t rw;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
	struct output_state *output_target;
	struct cmd_state *cmd;
};

struct output_state
{
	int	  exit_flag;
	pthread_t thread;
	FILE	 *file;
	char	 *filename;
	int16_t  result[MAXIMUM_BUF_LENGTH];
	int	  result_len;
	int	  rate;
	pthread_rwlock_t rw;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
};

struct controller_state
{
	int	  exit_flag;
	pthread_t thread;
	uint32_t freqs[FREQUENCIES_LIMIT];
	int	  freq_len;
	int	  freq_now;
	int	  edge;
	int	  wb_mode;
	pthread_cond_t hop;
	pthread_mutex_t hop_m;
	struct cmd_state *cmd;
};

// multiple of these, eventually
struct dongle_state dongle;
struct demod_state demod;
struct output_state output;
struct controller_state controller;
struct cmd_state cmd;


void usage(void)
{
	fprintf(stderr,
		"rtl_fm, a simple narrow band FM demodulator for RTL2832 based DVB-T receivers\n\n"
		"Use:\trtl_fm -f freq [-options] [filename]\n"
		"\t-f frequency_to_tune_to [Hz]\n"
		"\t	use multiple -f for scanning (requires squelch)\n"
		"\t	ranges supported, -f 118M:137M:25k\n"
		"\t[-C command_filename: command file with comma seperated values (.csv). sets modulation 'raw']\n"
		"\t\tcommand file contains lines with: freq,gain,trig-crit,trig_level,trig_tolerance,#meas,#blocks,trigger_command,arguments\n"
		"\t\t with trig_crit one of 'in', 'out', 'lt' or 'gt'\n"
		"\t[-B num_samples at capture rate: remove that many samples at capture_rate after changing frequency (default: 4096)]\n"
		"\t[-m minimum_capture_rate Hz (default: 1m, min=900k, max=3.2m)]\n"
		"\t[-v increase verbosity (default: 0)]\n"
		"\t[-M modulation (default: fm)]\n"
		"\t	fm or nbfm or nfm, wbfm or wfm, raw or iq, am, usb, lsb\n"
		"\t	wbfm == -M fm -s 170k -o 4 -A fast -r 32k -l 0 -E deemp\n"
		"\t	raw mode outputs 2x16 bit IQ pairs\n"
		"\t[-s sample_rate (default: 24k)]\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-T enable bias-T on GPIO PIN 0 (works for rtl-sdr.com v3 dongles)]\n"
		"\t[-D direct_sampling_mode (default: 0, 1 = I, 2 = Q, 3 = I below threshold, 4 = Q below threshold)]\n"
		"\t[-D direct_sampling_threshold_frequency (default: 0 use tuner specific frequency threshold for 3 and 4)]\n"
		"\t[-g tuner_gain (default: automatic)]\n"
		"\t[-w tuner_bandwidth in Hz (default: automatic)]\n"
		"\t[-W length of single buffer in units of 512 samples (default: 32 was 256)]\n"
		"\t[-l squelch_level (default: 0/off)]\n"
		"\t[-L N  prints levels every N calculations]\n"
		"\t	output are comma separated values (csv):\n"
		"\t	avg rms since last output, max rms since last output, overall max rms, squelch (paramed), rms, rms level, avg rms level\n"
		"\t[-c de-emphasis_time_constant in us for wbfm. 'us' or 'eu' for 75/50 us (default: us)]\n"
		//"\t	for fm squelch is inverted\n"
		"\t[-o oversampling (default: 1, 4 recommended)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\t[-E enable_option (default: none)]\n"
		"\t	use multiple -E to enable multiple options\n"
		"\t	edge:   enable lower edge tuning\n"
		"\t	rdc:    enable dc blocking filter on raw I/Q data at capture rate\n"
		"\t	adc:    enable dc blocking filter on demodulated audio\n"
		"\t	dc:     same as adc\n"
		"\t	rtlagc: enable rtl2832's digital agc (default: off)\n"
		"\t	agc:    same as rtlagc\n"
		"\t	deemp:  enable de-emphasis filter\n"
		"\t	direct: enable direct sampling (bypasses tuner, uses rtl2832 xtal)\n"
		"\t	offset: enable offset tuning (only e4000 tuner)\n"
		"\t[-q dc_avg_factor for option rdc (default: 9)]\n"
		"\t[-n disables demodulation output to stdout/file]\n"
		"\tfilename ('-' means stdout)\n"
		"\t	omitting the filename also uses stdout\n\n"
		"Experimental options:\n"
		"\t[-r resample_rate (default: none / same as -s)]\n"
		"\t[-t squelch_delay (default: 10)]\n"
		"\t	+values will mute/scan, -values will exit\n"
		"\t[-F fir_size (default: off)]\n"
		"\t	enables low-leakage downsample filter\n"
		"\t	size can be 0 or 9.  0 has bad roll off\n"
		"\t[-A std/fast/lut choose atan math (default: std)]\n"
		//"\t[-C clip_path (default: off)\n"
		//"\t (create time stamped raw clips, requires squelch)\n"
		//"\t (path must have '\%s' and will expand to date_time_freq)\n"
		//"\t[-H hop_fifo (default: off)\n"
		//"\t (fifo will contain the active frequency)\n"
		"\n"
		"Produces signed 16 bit ints, use Sox or aplay to hear them.\n"
		"\trtl_fm ... | play -t raw -r 24k -es -b 16 -c 1 -V1 -\n"
		"\t		   | aplay -r 24k -f S16_LE -t raw -c 1\n"
		"\t  -M wbfm  | play -r 32k ... \n"
		"\t  -s 22050 | multimon -t raw /dev/stdin\n\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dongle.dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	rtlsdr_cancel_async(dongle.dev);
}
#endif

/* more cond dumbness */
#define safe_cond_signal(n, m) pthread_mutex_lock(m); pthread_cond_signal(n); pthread_mutex_unlock(m)
#define safe_cond_wait(n, m) pthread_mutex_lock(m); pthread_cond_wait(n, m); pthread_mutex_unlock(m)

/* {length, coef, coef, coef}  and scaled by 2^15
   for now, only length 9, optimal way to get +85% bandwidth */
#define CIC_TABLE_MAX 10
int cic_9_tables[][10] = {
	{0,},
	{9, -156,  -97, 2798, -15489, 61019, -15489, 2798,  -97, -156},
	{9, -128, -568, 5593, -24125, 74126, -24125, 5593, -568, -128},
	{9, -129, -639, 6187, -26281, 77511, -26281, 6187, -639, -129},
	{9, -122, -612, 6082, -26353, 77818, -26353, 6082, -612, -122},
	{9, -120, -602, 6015, -26269, 77757, -26269, 6015, -602, -120},
	{9, -120, -582, 5951, -26128, 77542, -26128, 5951, -582, -120},
	{9, -119, -580, 5931, -26094, 77505, -26094, 5931, -580, -119},
	{9, -119, -578, 5921, -26077, 77484, -26077, 5921, -578, -119},
	{9, -119, -577, 5917, -26067, 77473, -26067, 5917, -577, -119},
	{9, -199, -362, 5303, -25505, 77489, -25505, 5303, -362, -199},
};

#if defined(_MSC_VER) && (_MSC_VER < 1800)
double log2(double n)
{
	return log(n) / log(2.0);
}
#endif


/* uint8_t negation = 255 - x */
#define NEG_U8( x )     ( 255 - x )
/* MUL_PLUS_J:    (a + j*b ) * j =  -b + j *  a */
/* MUL_MINUS_ONE: (a + j*b ) * -1 = -a + j * -b */
/* MUL_MINUS_J:   (a + j*b ) * -j =  b + j * -a */
#define MUL_PLUS_J_U8( X, J )	\
    tmp = X[J]; \
    X[J] = NEG_U8( X[J+1] ); \
    X[J+1] = tmp

#define MUL_MINUS_ONE_U8( X, J ) \
    X[J] = NEG_U8( X[J] ); \
    X[J+1] = NEG_U8( X[J+1] )

#define MUL_MINUS_J_U8( X, J ) \
    tmp = X[J]; \
    X[J] = X[J+1]; \
    X[J+1] = NEG_U8( tmp )


#define MUL_PLUS_J_INT( X, J )	\
    tmp = X[J]; \
    X[J] = - X[J+1]; \
    X[J+1] = tmp

#define MUL_MINUS_ONE_INT( X, J ) \
    X[J] = - X[J]; \
    X[J+1] = - X[J+1]

#define MUL_MINUS_J_INT( X, J ) \
    tmp = X[J]; \
    X[J] = X[J+1]; \
    X[J+1] = -tmp


void rotate16_90(int16_t *buf, uint32_t len)
{
	/* 90 degree rotation is 1, +j, -1, -j */
	uint32_t i;
	int16_t tmp;
	for (i=0; i<len; i+=8) {
		MUL_PLUS_J_INT( buf, i+2 );
		MUL_MINUS_ONE_INT( buf, i+4 );
		MUL_MINUS_J_INT( buf, i+6 );
	}
}

void rotate16_neg90(int16_t *buf, uint32_t len)
{
	/* -90 degree rotation is 1, -j, -1, +j */
	uint32_t i;
	int16_t tmp;
	for (i=0; i<len; i+=8) {
		MUL_MINUS_J_INT( buf, i+2 );
		MUL_MINUS_ONE_INT( buf, i+4 );
		MUL_PLUS_J_INT( buf, i+6 );
	}
}


void rotate_90(unsigned char *buf, uint32_t len)
{
	/* 90 degree rotation is 1, +j, -1, -j */
	uint32_t i;
	unsigned char tmp;
	for (i=0; i<len; i+=8) {
		MUL_PLUS_J_U8( buf, i+2 );
		MUL_MINUS_ONE_U8( buf, i+4 );
		MUL_MINUS_J_U8( buf, i+6 );
	}
}

void rotate_neg90(unsigned char *buf, uint32_t len)
{
	/* -90 degree rotation is 1, -j, -1, +j */
	uint32_t i;
	unsigned char tmp;
	for (i=0; i<len; i+=8) {
		MUL_MINUS_J_U8( buf, 2 );
		MUL_MINUS_ONE_U8( buf, 4 );
		MUL_PLUS_J_U8( buf, 6 );
	}
}

void low_pass(struct demod_state *d)
/* simple square window FIR */
{
	int i=0, i2=0;
	while (i < d->lp_len) {
		d->now_r += d->lowpassed[i];
		d->now_j += d->lowpassed[i+1];
		i += 2;
		d->prev_index++;
		if (d->prev_index < d->downsample) {
			continue;
		}
		d->lowpassed[i2]   = d->now_r; // * d->output_scale;
		d->lowpassed[i2+1] = d->now_j; // * d->output_scale;
		d->prev_index = 0;
		d->now_r = 0;
		d->now_j = 0;
		i2 += 2;
	}
	d->lp_len = i2;
}

static char * trim(char * s) {
	char *p = s;
	int l = strlen(p);

	while(isspace(p[l - 1])) p[--l] = 0;
	while(*p && isspace(*p)) ++p;

	return p;
}


static void cmd_init(struct cmd_state *c)
{
	int k;
	c->filename = NULL;
	c->file = NULL;
	c->lineNo = 1;
	c->checkADCmax = 0;
	c->checkADCrms = 0;
	c->prevFreq = -1;
	c->prevGain = -200;
	c->prevBandwidth = -1;
	c->freq = 0;
	c->gain = 0;
	c->trigCrit = crit_IN;
	c->refLevel = 0.0;
	c->refLevelTol = 0.0;
	c->numMeas = 0;
	c->numBlockTrigger = 0;
	c->command = NULL;
	c->args = NULL;
	c->levelSum = 0.0;
	c->numSummed = 0;
	c->omitFirstFreqLevels = 3;
	for (k = 0; k < FREQUENCIES_LIMIT; k++) {
		c->waitTrigger[k] = 0;
		c->statNumLevels[k] = 0;
		c->statFreq[k] = 0;
		c->statSumLevels[k] = 0.0;
		c->statMinLevel[k] = 0.0F;
		c->statMaxLevel[k] = 0.0F;
	}
}

static int toNextCmdLine(struct cmd_state *c)
{
	const char * delim = ",";
	char * pLine = NULL;
	char * pCmdFreq = NULL;
	char * pCmdGain = NULL;
	char * pCmdTrigCrit = NULL;
	char * pCmdLevel = NULL;
	char * pCmdTol = NULL;
	char * pCmdNumMeas = NULL;
	char * pCmdNumBlockTrigger = NULL;
	int numValidLines = 1;  /* assume valid lines */
	while (1) {
		if (c->file && feof(c->file)) {
			if (!numValidLines) {
				fprintf(stderr, "error: command file '%s' does not contain any valid lines!\n", c->filename);
				return 0;
			}
			fclose(c->file);
			c->file = NULL;
		}
		if (!c->file) {
			c->file = fopen(c->filename, "r");
			numValidLines = 0;
			c->lineNo = 0;
		}
		if (!c->file)
			return 0;
		pLine = fgets(c->acLine, 4096, c->file);
		if (!pLine) {
			continue;
		}
		c->lineNo++;
		pLine = trim(c->acLine);
		if (pLine[0]=='#' || pLine[0]==0)
			continue;  /* detect comment lines and empty lines */

		pCmdFreq = strtok(pLine, delim);
		if (!pCmdFreq) { fprintf(stderr, "error parsing frequency in line %d of command file!\n", c->lineNo); continue; }
		pCmdFreq = trim(pCmdFreq);
		/* check keywords */
		if (!strcmp(pCmdFreq, "adc") || !strcmp(pCmdFreq, "adcmax")) {
			c->checkADCmax = 1;
			continue;
		}
		else if (!strcmp(pCmdFreq, "adcrms")) {
			c->checkADCrms = 1;
			continue;
		}
		c->freq = (uint32_t)atofs(pCmdFreq);

		pCmdGain = strtok(NULL, delim);
		if (!pCmdGain) { fprintf(stderr, "error parsing gain in line %d of command file!\n", c->lineNo); continue; }
		pCmdGain = trim(pCmdGain);
		if (!strcmp(pCmdGain,"auto") || !strcmp(pCmdGain,"a"))
			c->gain = AUTO_GAIN;
		else
			c->gain = (int)(atof(pCmdGain) * 10);

		pCmdTrigCrit = strtok(NULL, delim);
		if (!pCmdTrigCrit) { fprintf(stderr, "error parsing expr in line %d of command file!\n", c->lineNo); continue; }
		pCmdTrigCrit = trim(pCmdTrigCrit);
		if (!strcmp(pCmdTrigCrit,"in"))			c->trigCrit = crit_IN;
		else if (!strcmp(pCmdTrigCrit,"=="))	c->trigCrit = crit_IN;
		else if (!strcmp(pCmdTrigCrit,"out"))	c->trigCrit = crit_OUT;
		else if (!strcmp(pCmdTrigCrit,"!="))	c->trigCrit = crit_OUT;
		else if (!strcmp(pCmdTrigCrit,"<>"))	c->trigCrit = crit_OUT;
		else if (!strcmp(pCmdTrigCrit,"lt"))	c->trigCrit = crit_LT;
		else if (!strcmp(pCmdTrigCrit,"<"))		c->trigCrit = crit_LT;
		else if (!strcmp(pCmdTrigCrit,"gt"))	c->trigCrit = crit_GT;
		else if (!strcmp(pCmdTrigCrit,">"))		c->trigCrit = crit_GT;
		else { fprintf(stderr, "error parsing expr in line %d of command file!\n", c->lineNo); continue; }

		pCmdLevel = strtok(NULL, delim);
		if (!pCmdLevel) { fprintf(stderr, "error parsing level in line %d of command file!\n", c->lineNo); continue; }
		c->refLevel = atof(trim(pCmdLevel));

		pCmdTol = strtok(NULL, delim);
		if (!pCmdTol) { fprintf(stderr, "error parsing tolerance in line %d of command file!\n", c->lineNo); continue; }
		c->refLevelTol = atof(trim(pCmdTol));

		pCmdNumMeas = strtok(NULL, delim);
		if (!pCmdNumMeas) { fprintf(stderr, "error parsing #measurements in line %d of command file!\n", c->lineNo); continue; }
		c->numMeas = atoi(trim(pCmdNumMeas));
		if (c->numMeas <= 0) { fprintf(stderr, "warning: fixed #measurements from %d to 10 in line %d of command file!\n", c->numMeas, c->lineNo); c->numMeas=10; }

		pCmdNumBlockTrigger = strtok(NULL, delim);
		if (!pCmdNumBlockTrigger) { fprintf(stderr, "error parsing #blockTrigger in line %d of command file!\n", c->lineNo); continue; }
		c->numBlockTrigger = atoi(trim(pCmdNumBlockTrigger));

		c->command = strtok(NULL, delim);
		/* no check: allow empty string. just trim it */
		if (c->command)
			c->command = trim(c->command);

		c->args = strtok(NULL, delim);
		/* no check: allow empty string. just trim it */
		if (c->args)
			c->args = trim(c->args);

		if (verbosity >= 2)
			fprintf(stderr, "read from cmd file: freq %.3f kHz, gain %0.1f dB, level %s {%.1f +/- %.1f}, cmd '%s %s'\n",
				c->freq /1000.0, c->gain /10.0, 
				aCritStr[c->trigCrit], c->refLevel, c->refLevelTol,
				(c->command ? c->command : "%"), (c->args ? c->args : "") );

		numValidLines++;
		return 1;
	}

	return 0;
}

static int testTrigCrit(struct cmd_state *c, double level)
{
	switch(c->trigCrit)
	{
	case crit_IN:	return ( c->refLevel-c->refLevelTol <= level && level <= c->refLevel+c->refLevelTol );
	case crit_OUT:	return ( c->refLevel-c->refLevelTol > level || level > c->refLevel+c->refLevelTol );
	case crit_LT:	return ( level < c->refLevel-c->refLevelTol );
	case crit_GT:	return ( level > c->refLevel+c->refLevelTol );
	}
	return 0;
}

static void checkTriggerCommand(struct cmd_state *c, unsigned char adcSampleMax, double powerSum, int powerCount )
{
	char acRepFreq[32], acRepGain[32], acRepMLevel[32], acRepRefLevel[32], acRepRefTolerance[32];
	char * execSearchStrings[7] = { "!freq!", "!gain!", "!mlevel!", "!crit!", "!reflevel!", "!reftol!", NULL };
	char * execReplaceStrings[7] = { acRepFreq, acRepGain, acRepMLevel, NULL, acRepRefLevel, acRepRefTolerance, NULL };
	double triggerLevel;
	double adcRms = 0.0;
	int k, triggerCommand = 0;
	int adcMax = (int)adcSampleMax - 127;
	char adcText[128];

	if (c->numSummed != c->numMeas)
		return;

	if (c->omitFirstFreqLevels) {
		/* workaround: measured levels of first controlled frequency looks wrong! */
		c->omitFirstFreqLevels--;
		return;
	}

	/* decrease all counters */
	for ( k = 0; k < FREQUENCIES_LIMIT; k++ ) {
		if ( c->waitTrigger[k] > 0 ) {
			c->waitTrigger[k] -= c->numMeas;
			if ( c->waitTrigger[k] < 0 )
				c->waitTrigger[k] = 0;
		}
	}
	triggerLevel = 20.0 * log10( 1E-10 + c->levelSum / c->numSummed );
	triggerCommand = testTrigCrit(c, triggerLevel);

	/* update statistics */
	if ( c->lineNo < FREQUENCIES_LIMIT ) {
		if ( c->statNumLevels[c->lineNo] == 0 ) {
			++c->statNumLevels[c->lineNo];
			c->statFreq[c->lineNo] = c->freq;
			c->statSumLevels[c->lineNo] = triggerLevel;
			c->statMinLevel[c->lineNo] = (float)triggerLevel;
			c->statMaxLevel[c->lineNo] = (float)triggerLevel;
		} else if ( c->statFreq[c->lineNo] == c->freq ) {
			++c->statNumLevels[c->lineNo];
			c->statSumLevels[c->lineNo] += triggerLevel;
			if ( c->statMinLevel[c->lineNo] > (float)triggerLevel )
				c->statMinLevel[c->lineNo] = (float)triggerLevel;
			if ( c->statMaxLevel[c->lineNo] < (float)triggerLevel )
				c->statMaxLevel[c->lineNo] = (float)triggerLevel;
		}
	}

	adcText[0] = 0;
	if (c->checkADCmax && c->checkADCrms) {
		adcRms = (powerCount >0) ? sqrt( powerSum / powerCount ) : -1.0;
		sprintf(adcText, "adc max %3d%s rms %5.1f ", adcMax, (adcMax>=64 ? (adcMax>=120 ? "!!" : "! " ) : "  "), adcRms );
	}
	else if (c->checkADCmax)
		sprintf(adcText, "adc max %3d%s ", adcMax, (adcMax>=64 ? (adcMax>=120 ? "!!" : "! " ) : "  ") );
	else if (c->checkADCrms) {
		adcRms = (powerCount >0) ? sqrt( powerSum / powerCount ) : -1.0;
		sprintf(adcText, "adc rms %5.1f ", adcRms );
	}

	if ( c->lineNo < FREQUENCIES_LIMIT && c->waitTrigger[c->lineNo] <= 0 ) {
			c->waitTrigger[c->lineNo] = triggerCommand ? c->numBlockTrigger : 0;
			if (verbosity)
				fprintf(stderr, "%.3f kHz: gain %4.1f + level %4.1f dB %s=> %s\n",
					(double)c->freq /1000.0, 0.1*c->gain, triggerLevel, adcText,
					(triggerCommand ? "activates trigger" : "does not trigger") );
			if (triggerCommand && c->command && c->command[0]) {
				fprintf(stderr, "command to trigger is '%s %s'\n", c->command, c->args);
				/* prepare search/replace of special parameters for command arguments */
				snprintf(acRepFreq, 32, "%u", c->freq);
				snprintf(acRepGain, 32, "%d", c->gain);
				snprintf(acRepMLevel, 32, "%d", (int)(0.5 + triggerLevel*10.0) );
				execReplaceStrings[3] = aCritStr[c->trigCrit];
				snprintf(acRepRefLevel, 32, "%d", (int)(0.5 + c->refLevel*10.0) );
				snprintf(acRepRefTolerance, 32, "%d", (int)(0.5 + c->refLevelTol*10.0) );
				executeInBackground( c->command, c->args, execSearchStrings, execReplaceStrings );
			}
	} else if (verbosity) {
		fprintf(stderr, "%.3f kHz: gain %4.1f + level %4.1f dB %s=> %s, blocks for %d\n",
			(double)c->freq /1000.0, 0.1*c->gain, triggerLevel, adcText, (triggerCommand ? "would trigger" : "does not trigger"),
			(c->lineNo < FREQUENCIES_LIMIT ? c->waitTrigger[c->lineNo] : -1 ) );
	}
	c->numSummed++;
}


int low_pass_simple(int16_t *signal2, int len, int step)
// no wrap around, length must be multiple of step
{
	int i, i2, sum;
	for(i=0; i < len; i+=step) {
		sum = 0;
		for(i2=0; i2<step; i2++) {
			sum += (int)signal2[i + i2];
		}
		//signal2[i/step] = (int16_t)(sum / step);
		signal2[i/step] = (int16_t)(sum);
	}
	signal2[i/step + 1] = signal2[i/step];
	return len / step;
}

void low_pass_real(struct demod_state *s)
/* simple square window FIR */
// add support for upsampling?
{
	int i=0, i2=0;
	int fast = (int)s->rate_out;
	int slow = s->rate_out2;
	while (i < s->result_len) {
		s->now_lpr += s->result[i];
		i++;
		s->prev_lpr_index += slow;
		if (s->prev_lpr_index < fast) {
			continue;
		}
		s->result[i2] = (int16_t)(s->now_lpr / (fast/slow));
		s->prev_lpr_index -= fast;
		s->now_lpr = 0;
		i2 += 1;
	}
	s->result_len = i2;
}

void fifth_order(int16_t *data, int length, int16_t *hist)
/* for half of interleaved data */
{
	int i;
	int16_t a, b, c, d, e, f;
	a = hist[1];
	b = hist[2];
	c = hist[3];
	d = hist[4];
	e = hist[5];
	f = data[0];
	/* a downsample should improve resolution, so don't fully shift */
	data[0] = (a + (b+e)*5 + (c+d)*10 + f) >> 4;
	for (i=4; i<length; i+=4) {
		a = c;
		b = d;
		c = e;
		d = f;
		e = data[i-2];
		f = data[i];
		data[i/2] = (a + (b+e)*5 + (c+d)*10 + f) >> 4;
	}
	/* archive */
	hist[0] = a;
	hist[1] = b;
	hist[2] = c;
	hist[3] = d;
	hist[4] = e;
	hist[5] = f;
}

void generic_fir(int16_t *data, int length, int *fir, int16_t *hist)
/* Okay, not at all generic.  Assumes length 9, fix that eventually. */
{
	int d, temp, sum;
	for (d=0; d<length; d+=2) {
		temp = data[d];
		sum = 0;
		sum += (hist[0] + hist[8]) * fir[1];
		sum += (hist[1] + hist[7]) * fir[2];
		sum += (hist[2] + hist[6]) * fir[3];
		sum += (hist[3] + hist[5]) * fir[4];
		sum +=			hist[4]  * fir[5];
		data[d] = sum >> 15 ;
		hist[0] = hist[1];
		hist[1] = hist[2];
		hist[2] = hist[3];
		hist[3] = hist[4];
		hist[4] = hist[5];
		hist[5] = hist[6];
		hist[6] = hist[7];
		hist[7] = hist[8];
		hist[8] = temp;
	}
}

/* define our own complex math ops
   because ARMv5 has no hardware float */

void multiply(int ar, int aj, int br, int bj, int *cr, int *cj)
{
	*cr = ar*br - aj*bj;
	*cj = aj*br + ar*bj;
}

int polar_discriminant(int ar, int aj, int br, int bj)
{
	int cr, cj;
	double angle;
	multiply(ar, aj, br, -bj, &cr, &cj);
	angle = atan2((double)cj, (double)cr);
	return (int)(angle / 3.14159 * (1<<14));
}

int fast_atan2(int y, int x)
/* pre scaled for int16 */
{
	int yabs, angle;
	int pi4=(1<<12), pi34=3*(1<<12);  // note pi = 1<<14
	if (x==0 && y==0) {
		return 0;
	}
	yabs = y;
	if (yabs < 0) {
		yabs = -yabs;
	}
	if (x >= 0) {
		angle = pi4  - pi4 * (x-yabs) / (x+yabs);
	} else {
		angle = pi34 - pi4 * (x+yabs) / (yabs-x);
	}
	if (y < 0) {
		return -angle;
	}
	return angle;
}

int polar_disc_fast(int ar, int aj, int br, int bj)
{
	int cr, cj;
	multiply(ar, aj, br, -bj, &cr, &cj);
	return fast_atan2(cj, cr);
}

int atan_lut_init(void)
{
	int i = 0;

	atan_lut = malloc(atan_lut_size * sizeof(int));

	for (i = 0; i < atan_lut_size; i++) {
		atan_lut[i] = (int) (atan((double) i / (1<<atan_lut_coef)) / 3.14159 * (1<<14));
	}

	return 0;
}

int polar_disc_lut(int ar, int aj, int br, int bj)
{
	int cr, cj, x, x_abs;

	multiply(ar, aj, br, -bj, &cr, &cj);

	/* special cases */
	if (cr == 0 || cj == 0) {
		if (cr == 0 && cj == 0)
			{return 0;}
		if (cr == 0 && cj > 0)
			{return 1 << 13;}
		if (cr == 0 && cj < 0)
			{return -(1 << 13);}
		if (cj == 0 && cr > 0)
			{return 0;}
		if (cj == 0 && cr < 0)
			{return 1 << 14;}
	}

	/* real range -32768 - 32768 use 64x range -> absolute maximum: 2097152 */
	x = (cj << atan_lut_coef) / cr;
	x_abs = abs(x);

	if (x_abs >= atan_lut_size) {
		/* we can use linear range, but it is not necessary */
		return (cj > 0) ? 1<<13 : -(1<<13);
	}

	if (x > 0) {
		return (cj > 0) ? atan_lut[x] : atan_lut[x] - (1<<14);
	} else {
		return (cj > 0) ? (1<<14) - atan_lut[-x] : -atan_lut[-x];
	}

	return 0;
}

void fm_demod(struct demod_state *fm)
{
	int i, pcm;
	int16_t *lp = fm->lowpassed;
	pcm = polar_discriminant(lp[0], lp[1],
		fm->pre_r, fm->pre_j);
	fm->result[0] = (int16_t)pcm;
	for (i = 2; i < (fm->lp_len-1); i += 2) {
		switch (fm->custom_atan) {
		case 0:
			pcm = polar_discriminant(lp[i], lp[i+1],
				lp[i-2], lp[i-1]);
			break;
		case 1:
			pcm = polar_disc_fast(lp[i], lp[i+1],
				lp[i-2], lp[i-1]);
			break;
		case 2:
			pcm = polar_disc_lut(lp[i], lp[i+1],
				lp[i-2], lp[i-1]);
			break;
		}
		fm->result[i/2] = (int16_t)pcm;
	}
	fm->pre_r = lp[fm->lp_len - 2];
	fm->pre_j = lp[fm->lp_len - 1];
	fm->result_len = fm->lp_len/2;
}

void am_demod(struct demod_state *fm)
// todo, fix this extreme laziness
{
	int i, pcm;
	int16_t *lp = fm->lowpassed;
	int16_t *r  = fm->result;
	for (i = 0; i < fm->lp_len; i += 2) {
		// hypot uses floats but won't overflow
		//r[i/2] = (int16_t)hypot(lp[i], lp[i+1]);
		pcm = lp[i] * lp[i];
		pcm += lp[i+1] * lp[i+1];
		r[i/2] = (int16_t)sqrt(pcm) * fm->output_scale;
	}
	fm->result_len = fm->lp_len/2;
	// lowpass? (3khz)  highpass?  (dc)
}

void usb_demod(struct demod_state *fm)
{
	int i, pcm;
	int16_t *lp = fm->lowpassed;
	int16_t *r  = fm->result;
	for (i = 0; i < fm->lp_len; i += 2) {
		pcm = lp[i] + lp[i+1];
		r[i/2] = (int16_t)pcm * fm->output_scale;
	}
	fm->result_len = fm->lp_len/2;
}

void lsb_demod(struct demod_state *fm)
{
	int i, pcm;
	int16_t *lp = fm->lowpassed;
	int16_t *r  = fm->result;
	for (i = 0; i < fm->lp_len; i += 2) {
		pcm = lp[i] - lp[i+1];
		r[i/2] = (int16_t)pcm * fm->output_scale;
	}
	fm->result_len = fm->lp_len/2;
}

void raw_demod(struct demod_state *fm)
{
	int i;
	for (i = 0; i < fm->lp_len; i++) {
		fm->result[i] = (int16_t)fm->lowpassed[i];
	}
	fm->result_len = fm->lp_len;
}

void deemph_filter(struct demod_state *fm)
{
	static int avg;  // cheating...
	int i, d;
	// de-emph IIR
	// avg = avg * (1 - alpha) + sample * alpha;
	for (i = 0; i < fm->result_len; i++) {
		d = fm->result[i] - avg;
		if (d > 0) {
			avg += (d + fm->deemph_a/2) / fm->deemph_a;
		} else {
			avg += (d - fm->deemph_a/2) / fm->deemph_a;
		}
		fm->result[i] = (int16_t)avg;
	}
}

void dc_block_audio_filter(struct demod_state *fm)
{
	int i, avg;
	int64_t sum = 0;
	for (i=0; i < fm->result_len; i++) {
		sum += fm->result[i];
	}
	avg = sum / fm->result_len;
	avg = (avg + fm->dc_avg * fm->adc_block_const) / ( fm->adc_block_const + 1 );
	for (i=0; i < fm->result_len; i++) {
		fm->result[i] -= avg;
	}
	fm->dc_avg = avg;
}

void dc_block_raw_filter(struct demod_state *fm, int16_t *buf, int len)
{
	/* derived from dc_block_audio_filter,
		running over the raw I/Q components
	*/
	int i, avgI, avgQ;
	int64_t sumI = 0;
	int64_t sumQ = 0;
	for (i = 0; i < len; i += 2) {
		sumI += buf[i];
		sumQ += buf[i+1];
	}
	avgI = sumI / ( len / 2 );
	avgQ = sumQ / ( len / 2 );
	avgI = (avgI + fm->dc_avgI * fm->rdc_block_const) / ( fm->rdc_block_const + 1 );
	avgQ = (avgQ + fm->dc_avgQ * fm->rdc_block_const) / ( fm->rdc_block_const + 1 );
	for (i = 0; i < len; i += 2) {
		buf[i] -= avgI;
		buf[i+1] -= avgQ;
	}
	fm->dc_avgI = avgI;
	fm->dc_avgQ = avgQ;
}
int mad(int16_t *samples, int len, int step)
/* mean average deviation */
{
	int i=0, sum=0, ave=0;
	if (len == 0)
		{return 0;}
	for (i=0; i<len; i+=step) {
		sum += samples[i];
	}
	ave = sum / (len * step);
	sum = 0;
	for (i=0; i<len; i+=step) {
		sum += abs(samples[i] - ave);
	}
	return sum / (len / step);
}

int rms(int16_t *samples, int len, int step, int omitDCfix)
/* largely lifted from rtl_power */
{
	double dc, err;
	int i, num;
	int32_t t, s;
	uint32_t p;  /* use sign bit to prevent overflow */

	p = 0;
	t = 0L;
	while (len > step * 32768) /* 8 bit squared = 16 bit. limit to 2^16 for 32 bit squared sum */
		++step;  /* increase step to prevent overflow */
	for (i=0; i<len; i+=step) {
		s = (long)samples[i];
		t += s;
		p += s * s;
	}

	if (omitDCfix) {
		/* DC is already corrected. No need to do it again */
		num = len / step;
		return (int)sqrt( (double)(p) / num );
	}

	/* correct for dc offset in squares */
	dc = (double)(t*step) / (double)len;
	err = t * 2 * dc - dc * dc * len;

	return (int)sqrt((p-err) / len);
}

void arbitrary_upsample(int16_t *buf1, int16_t *buf2, int len1, int len2)
/* linear interpolation, len1 < len2 */
{
	int i = 1;
	int j = 0;
	int tick = 0;
	double frac;  // use integers...
	while (j < len2) {
		frac = (double)tick / (double)len2;
		buf2[j] = (int16_t)(buf1[i-1]*(1-frac) + buf1[i]*frac);
		j++;
		tick += len1;
		if (tick > len2) {
			tick -= len2;
			i++;
		}
		if (i >= len1) {
			i = len1 - 1;
			tick = len2;
		}
	}
}

void arbitrary_downsample(int16_t *buf1, int16_t *buf2, int len1, int len2)
/* fractional boxcar lowpass, len1 > len2 */
{
	int i = 1;
	int j = 0;
	int tick = 0;
	double remainder = 0;
	double frac;  // use integers...
	buf2[0] = 0;
	while (j < len2) {
		frac = 1.0;
		if ((tick + len2) > len1) {
			frac = (double)(len1 - tick) / (double)len2;}
		buf2[j] += (int16_t)((double)buf1[i] * frac + remainder);
		remainder = (double)buf1[i] * (1.0-frac);
		tick += len2;
		i++;
		if (tick > len1) {
			j++;
			buf2[j] = 0;
			tick -= len1;
		}
		if (i >= len1) {
			i = len1 - 1;
			tick = len1;
		}
	}
	for (j=0; j<len2; j++) {
		buf2[j] = buf2[j] * len2 / len1;}
}

void arbitrary_resample(int16_t *buf1, int16_t *buf2, int len1, int len2)
/* up to you to calculate lengths and make sure it does not go OOB
 * okay for buffers to overlap, if you are downsampling */
{
	if (len1 < len2) {
		arbitrary_upsample(buf1, buf2, len1, len2);
	} else {
		arbitrary_downsample(buf1, buf2, len1, len2);
	}
}

void full_demod(struct demod_state *d)
{
	struct cmd_state *c = d->cmd;
	double freqK, avgRms, rmsLevel, avgRmsLevel;
	int i, ds_p;
	int sr = 0;
	static int printBlockLen = 1;
	ds_p = d->downsample_passes;
	if (ds_p) {
		for (i=0; i < ds_p; i++) {
			fifth_order(d->lowpassed,   (d->lp_len >> i), d->lp_i_hist[i]);
			fifth_order(d->lowpassed+1, (d->lp_len >> i) - 1, d->lp_q_hist[i]);
		}
		d->lp_len = d->lp_len >> ds_p;
		/* droop compensation */
		if (d->comp_fir_size == 9 && ds_p <= CIC_TABLE_MAX) {
			generic_fir(d->lowpassed, d->lp_len,
				cic_9_tables[ds_p], d->droop_i_hist);
			generic_fir(d->lowpassed+1, d->lp_len-1,
				cic_9_tables[ds_p], d->droop_q_hist);
		}
	} else {
		low_pass(d);
	}
	/* power squelch */
	if (d->squelch_level) {
		sr = rms(d->lowpassed, d->lp_len, 1, d->dc_block_raw);
		if (sr >= 0) {
			if (sr < d->squelch_level) {
				d->squelch_hits++;
				for (i=0; i<d->lp_len; i++) {
					d->lowpassed[i] = 0;
				}
			} else {
				d->squelch_hits = 0;}
		}
	}

	if (printLevels) {
		if (!sr)
			sr = rms(d->lowpassed, d->lp_len, 1, d->dc_block_raw);
		--printLevelNo;
		if (printLevels && sr >= 0) {
			levelSum += sr;
			if (levelMax < sr)		levelMax = sr;
			if (levelMaxMax < sr)	levelMaxMax = sr;
			if  (!printLevelNo) {
				printLevelNo = printLevels;
				freqK = dongle.userFreq /1000.0;
				avgRms = levelSum / printLevels;
				rmsLevel = 20.0 * log10( 1E-10 + sr );
				avgRmsLevel = 20.0 * log10( 1E-10 + avgRms );
				fprintf(stderr, "%.3f kHz, %.1f avg rms, %d max rms, %d max max rms, %d squelch rms, %d rms, %.1f dB rms level, %.2f dB avg rms level\n",
					freqK, avgRms, levelMax, levelMaxMax, d->squelch_level, sr, rmsLevel, avgRmsLevel );
				levelMax = 0;
				levelSum = 0;
			}
		}
	}

	if (c->filename) {
		if (!sr)
			sr = rms(d->lowpassed, d->lp_len, 1, d->dc_block_raw);
		if ( printBlockLen && verbosity ) {
			fprintf(stderr, "block length for rms after decimation is %d samples\n", d->lp_len);
			if ( d->lp_len < 128 )
				fprintf(stderr, "\n  WARNING: increase block length with option -W\n\n");
			--printBlockLen;
		}
		if (!c->numSummed)
			c->levelSum = 0;
		if (c->numSummed < c->numMeas && sr >= 0) {
			c->levelSum += sr;
			c->numSummed++;
		}
	}

	d->mode_demod(d);  /* lowpassed -> result */
	if (d->mode_demod == &raw_demod) {
		return;
	}
	/* todo, fm noise squelch */
	// use nicer filter here too?
	if (d->post_downsample > 1) {
		d->result_len = low_pass_simple(d->result, d->result_len, d->post_downsample);}
	if (d->deemph) {
		deemph_filter(d);}
	if (d->dc_block_audio) {
		dc_block_audio_filter(d);}
	if (d->rate_out2 > 0) {
		low_pass_real(d);
		//arbitrary_resample(d->result, d->result, d->result_len, d->result_len * d->rate_out2 / d->rate_out);
	}
}

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	struct dongle_state *s = ctx;
	struct demod_state *d = s->demod_target;
	struct cmd_state *c = d->cmd;
	int i, muteLen = s->mute;
	unsigned char sampleMax;
	uint32_t sampleP, samplePowSum = 0.0;
	int samplePowCount = 0, step = 2;

	if (do_exit) {
		return;}
	if (!ctx) {
		return;}
	if (s->mute) {
		if(muteLen > (int)len)
			muteLen = len;
		s->mute -= muteLen;  /* we may need to mute multiple blocks */
		if(!c->filename) {
			for (i=0; i<muteLen; i++)
				buf[i] = 127;
		}
		/* reset adc max and power */
		s->samplePowSum = 0.0;
		s->samplePowCount = 0;
		s->sampleMax = 0;
	}
	/* OR all samples to allow checking overflow
	 * - before conversion to 16 bit and before DC filtering.
	 * we only get bitmask of positive samples (after -127) but that won't matter */
	if (c->checkADCmax ) {
		sampleMax = s->sampleMax;
		for (i=0; i<(int)len; i++) {
			if ( buf[i] > sampleMax )
				sampleMax = buf[i];
		}
		s->sampleMax = sampleMax;
	}
	if (c->checkADCrms ) {
		while ( len >= 16384 * step )
			step += 2;
		for (i=0; i<(int)len; i+= step) {
			sampleP  = ( (int)buf[i]   -127 ) * ( (int)buf[i]   -127 );  /* I^2 */
			sampleP += ( (int)buf[i+1] -127 ) * ( (int)buf[i+1] -127 );  /* Q^2 */
			samplePowSum += sampleP;
			++samplePowCount;
		}
		s->samplePowSum += (double)samplePowSum / samplePowCount;
		s->samplePowCount += 1;
	}
	/* 1st: convert to 16 bit - to allow easier calculation of DC */
	for (i=0; i<(int)len; i++) {
		s->buf16[i] = ( (int16_t)buf[i] - 127 );
	}
	/* 2nd: do DC filtering BEFORE up-mixing */
	if (d->dc_block_raw) {
		dc_block_raw_filter(d, s->buf16, (int)len);
	}
	if (muteLen && c->filename)
		return;	/* "mute" after the dc_block_raw_filter(), giving it time to remove the new DC */
	/* 3rd: down-mixing */
	if (!s->offset_tuning) {
		rotate16_neg90(s->buf16, (int)len);
	}
	pthread_rwlock_wrlock(&d->rw);
	memcpy(d->lowpassed, s->buf16, 2*len);
	d->lp_len = len;
	pthread_rwlock_unlock(&d->rw);
	safe_cond_signal(&d->ready, &d->ready_m);
}

static void *dongle_thread_fn(void *arg)
{
	struct dongle_state *s = arg;
	rtlsdr_read_async(s->dev, rtlsdr_callback, s, 0, s->buf_len);
	return 0;
}

static void *demod_thread_fn(void *arg)
{
	struct demod_state *d = arg;
	struct output_state *o = d->output_target;
	struct cmd_state *c = d->cmd;
	while (!do_exit) {
		safe_cond_wait(&d->ready, &d->ready_m);
		pthread_rwlock_wrlock(&d->rw);
		full_demod(d);
		pthread_rwlock_unlock(&d->rw);
		if (d->exit_flag) {
			do_exit = 1;
		}
		if (d->squelch_level && d->squelch_hits > d->conseq_squelch) {
			d->squelch_hits = d->conseq_squelch + 1;  /* hair trigger */
			safe_cond_signal(&controller.hop, &controller.hop_m);
			continue;
		}

		if (do_exit)
			break;

		if (c->filename && c->numSummed >= c->numMeas) {
			checkTriggerCommand(c, dongle.sampleMax, dongle.samplePowSum, dongle.samplePowCount);

			safe_cond_signal(&controller.hop, &controller.hop_m);
			continue;
		}

		if (OutputToStdout) {
			pthread_rwlock_wrlock(&o->rw);
			memcpy(o->result, d->result, 2*d->result_len);
			o->result_len = d->result_len;
			pthread_rwlock_unlock(&o->rw);
			safe_cond_signal(&o->ready, &o->ready_m);
		}
	}
	return 0;
}

static void *output_thread_fn(void *arg)
{
	struct output_state *s = arg;
	while (!do_exit) {
		// use timedwait and pad out under runs
		safe_cond_wait(&s->ready, &s->ready_m);
		pthread_rwlock_rdlock(&s->rw);
		fwrite(s->result, 2, s->result_len, s->file);
		pthread_rwlock_unlock(&s->rw);
	}
	return 0;
}

static void optimal_settings(uint32_t freq, uint32_t rate)
{
	// giant ball of hacks
	// seems unable to do a single pass, 2:1
	uint32_t capture_freq, capture_rate;
	struct dongle_state *d = &dongle;
	struct demod_state *dm = &demod;
	struct controller_state *cs = &controller;
	dm->downsample = (MinCaptureRate / dm->rate_in) + 1;
	if (dm->downsample_passes) {
		dm->downsample_passes = (int)log2(dm->downsample) + 1;
		dm->downsample = 1 << dm->downsample_passes;
	}
	if (verbosity >= 2) {
		fprintf(stderr, "downsample_passes = %d (= # of fifth_order() iterations), downsample = %d\n", dm->downsample_passes, dm->downsample );
	}
	capture_freq = freq;
	capture_rate = dm->downsample * dm->rate_in;
	if (verbosity >= 2)
		fprintf(stderr, "capture_rate = dm->downsample * dm->rate_in = %d * %d = %d\n", dm->downsample, dm->rate_in, capture_rate );
	if (!d->offset_tuning) {
		capture_freq = freq - capture_rate/4;
		if (verbosity >= 2)
			fprintf(stderr, "optimal_settings(freq = %u): capture_freq = freq - capture_rate/4 = %u\n", freq, capture_freq );
	}
	capture_freq += cs->edge * dm->rate_in / 2;
	if (verbosity >= 2)
		fprintf(stderr, "optimal_settings(freq = %u): capture_freq +=  cs->edge * dm->rate_in / 2 = %d * %d / 2 = %u\n", freq, cs->edge, dm->rate_in, capture_freq );
	dm->output_scale = (1<<15) / (128 * dm->downsample);
	if (dm->output_scale < 1) {
		dm->output_scale = 1;}
	if (dm->mode_demod == &fm_demod) {
		dm->output_scale = 1;}
	d->userFreq = freq;
	d->freq = capture_freq;
	d->rate = capture_rate;
	if (verbosity >= 2)
		fprintf(stderr, "optimal_settings(freq = %u) delivers freq %.0f, rate %.0f\n", freq, (double)d->freq, (double)d->rate );
}

static void *controller_thread_fn(void *arg)
{
	// thoughts for multiple dongles
	// might be no good using a controller thread if retune/rate blocks
	int i, r, execWaitHop = 1;
	struct controller_state *s = arg;
	struct cmd_state *c = s->cmd;

	if (s->wb_mode) {
		if (verbosity)
			fprintf(stderr, "wbfm: adding 16000 Hz to every input frequency\n");
		for (i=0; i < s->freq_len; i++) {
			s->freqs[i] += 16000;}
	}

	/* set up primary channel */
	if (c->filename) {
		dongle.mute = dongle.rate; /* over a second - until parametrized the dongle */
		toNextCmdLine(c);
		/*fprintf(stderr, "\nswitched to next command line. new freq %u\n", c->freq);*/
		s->freqs[0] = c->freq;
		execWaitHop = 0;
	}

	optimal_settings(s->freqs[0], demod.rate_in);
	if (dongle.direct_sampling) {
		verbose_direct_sampling(dongle.dev, 1);}
	if (dongle.offset_tuning) {
		verbose_offset_tuning(dongle.dev);}

	/* Set the frequency */
	if (verbosity) {
		fprintf(stderr, "verbose_set_frequency(%.3f kHz)\n", (double)dongle.userFreq /1000.0);
		if (!dongle.offset_tuning)
			fprintf(stderr, "  frequency is away from parametrized one, to avoid negative impact from dc\n");
	}
	verbose_set_frequency(dongle.dev, dongle.freq);
	fprintf(stderr, "Oversampling input by: %ix.\n", demod.downsample);
	fprintf(stderr, "Oversampling output by: %ix.\n", demod.post_downsample);
	fprintf(stderr, "Buffer size: %0.2fms\n",
		1000 * 0.5 * (float)ACTUAL_BUF_LENGTH / (float)dongle.rate);

	/* Set the sample rate */
	if (verbosity)
		fprintf(stderr, "verbose_set_sample_rate(%.0f Hz)\n", (double)dongle.rate);
	verbose_set_sample_rate(dongle.dev, dongle.rate);
	fprintf(stderr, "Output at %u Hz.\n", demod.rate_in/demod.post_downsample);

	while (!do_exit) {
		if (execWaitHop)
			safe_cond_wait(&s->hop, &s->hop_m);
		execWaitHop = 1;  /* execute following safe_cond_wait()'s */
		/* fprintf(stderr, "\nreceived hop condition\n"); */
		if (s->freq_len <= 1 && !c->filename) {
			continue;}
		if (!c->filename) {
			/* hacky hopping */
			s->freq_now = (s->freq_now + 1) % s->freq_len;
			optimal_settings(s->freqs[s->freq_now], demod.rate_in);
			rtlsdr_set_center_freq(dongle.dev, dongle.freq);
			dongle.mute = DEFAULT_BUFFER_DUMP;
		} else {
			dongle.mute = 2 * 3200000; /* over a second - until parametrized the dongle */
			c->numSummed = 0;

			toNextCmdLine(c);

			optimal_settings(c->freq, demod.rate_in);
			/* 1- set center frequency */
			if (c->prevFreq != dongle.freq) {
				rtlsdr_set_center_freq(dongle.dev, dongle.freq);
				c->prevFreq != dongle.freq;
			}
			/* 2- Set the tuner gain */
			if (c->prevGain != c->gain) {
				if (c->gain == AUTO_GAIN) {
					r = rtlsdr_set_tuner_gain_mode(dongle.dev, 0);
					if (r != 0)
						fprintf(stderr, "WARNING: Failed to set automatic tuner gain.\n");
					else
						c->prevGain = c->gain;
				} else {
					c->gain = nearest_gain(dongle.dev, c->gain);
					r = rtlsdr_set_tuner_gain_mode(dongle.dev, 1);
					if (r < 0)
						fprintf(stderr, "WARNING: Failed to enable manual gain.\n");
					else {
						r = rtlsdr_set_tuner_gain(dongle.dev, c->gain);
						if (r != 0)
							fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
						else
							c->prevGain = c->gain;
					}
				}
			}
			/* 3- Set tuner bandwidth */
			if (c->prevBandwidth != dongle.bandwidth) {
				r = rtlsdr_set_tuner_bandwidth(dongle.dev, dongle.bandwidth);
				if (r < 0)
					fprintf(stderr, "WARNING: Failed to set bandwidth.\n");
				else
					c->prevBandwidth != dongle.bandwidth;
			}
			/* 4- Set ADC samplerate *
			r = rtlsdr_set_sample_rate(dongle.dev, dongle.rate);
			if (r < 0)
				fprintf(stderr, "WARNING: Failed to set sample rate.\n");
			*/

			c->levelSum = 0;
			c->numSummed = 0;
			/* reset DC filters */
			demod.dc_avg = 0;
			demod.dc_avgI = 0;
			demod.dc_avgQ = 0;
			dongle.mute = BufferDump;
			/* reset adc max and power */
			dongle.samplePowSum = 0.0;
			dongle.samplePowCount = 0;
			dongle.sampleMax = 0;
		}

	}
	return 0;
}

void frequency_range(struct controller_state *s, char *arg)
{
	char *start, *stop, *step;
	int i;
	start = arg;
	stop = strchr(start, ':') + 1;
	stop[-1] = '\0';
	step = strchr(stop, ':') + 1;
	step[-1] = '\0';
	for(i=(int)atofs(start); i<=(int)atofs(stop); i+=(int)atofs(step))
	{
		s->freqs[s->freq_len] = (uint32_t)i;
		s->freq_len++;
		if (s->freq_len >= FREQUENCIES_LIMIT) {
			break;}
	}
	stop[-1] = ':';
	step[-1] = ':';
}

void dongle_init(struct dongle_state *s)
{
	s->rate = DEFAULT_SAMPLE_RATE;
	s->gain = AUTO_GAIN; /* tenths of a dB */
	s->mute = 0;
	s->direct_sampling = 0;
	s->offset_tuning = 0;
	s->demod_target = &demod;
	s->samplePowSum = 0.0;
	s->samplePowCount = 0;
	s->sampleMax = 0;
	s->bandwidth = 0;
	s->buf_len = 32 * 512;  /* see rtl_tcp */
}

void demod_init(struct demod_state *s)
{
	s->rate_in = DEFAULT_SAMPLE_RATE;
	s->rate_out = DEFAULT_SAMPLE_RATE;
	s->squelch_level = 0;
	s->conseq_squelch = 10;
	s->terminate_on_squelch = 0;
	s->squelch_hits = 11;
	s->downsample_passes = 0;
	s->comp_fir_size = 0;
	s->prev_index = 0;
	s->post_downsample = 1;	// once this works, default = 4
	s->custom_atan = 0;
	s->deemph = 0;
	s->rate_out2 = -1;	// flag for disabled
	s->mode_demod = &fm_demod;
	s->pre_j = s->pre_r = s->now_r = s->now_j = 0;
	s->prev_lpr_index = 0;
	s->deemph_a = 0;
	s->now_lpr = 0;
	s->dc_block_audio = 0;
	s->dc_avg = 0;
	s->adc_block_const = 9;
	s->dc_block_raw = 0;
	s->dc_avgI = 0;
	s->dc_avgQ = 0;
	s->rdc_block_const = 9;
	pthread_rwlock_init(&s->rw, NULL);
	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL);
	s->output_target = &output;
	s->cmd = &cmd;
}

void demod_cleanup(struct demod_state *s)
{
	pthread_rwlock_destroy(&s->rw);
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);
}

void output_init(struct output_state *s)
{
	s->rate = DEFAULT_SAMPLE_RATE;
	pthread_rwlock_init(&s->rw, NULL);
	pthread_cond_init(&s->ready, NULL);
	pthread_mutex_init(&s->ready_m, NULL);
}

void output_cleanup(struct output_state *s)
{
	pthread_rwlock_destroy(&s->rw);
	pthread_cond_destroy(&s->ready);
	pthread_mutex_destroy(&s->ready_m);
}

void controller_init(struct controller_state *s)
{
	s->freqs[0] = 100000000;
	s->freq_len = 0;
	s->edge = 0;
	s->wb_mode = 0;
	pthread_cond_init(&s->hop, NULL);
	pthread_mutex_init(&s->hop_m, NULL);
	s->cmd = &cmd;
}

void controller_cleanup(struct controller_state *s)
{
	pthread_cond_destroy(&s->hop);
	pthread_mutex_destroy(&s->hop_m);
}

void sanity_checks(void)
{
	if (controller.freq_len == 0) {
		fprintf(stderr, "Please specify a frequency.\n");
		exit(1);
	}

	if (controller.freq_len >= FREQUENCIES_LIMIT) {
		fprintf(stderr, "Too many channels, maximum %i.\n", FREQUENCIES_LIMIT);
		exit(1);
	}

	if (controller.freq_len > 1 && demod.squelch_level == 0) {
		fprintf(stderr, "Please specify a squelch level.  Required for scanning multiple frequencies.\n");
		exit(1);
	}

}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	int r, opt;
	int dev_given = 0;
	int custom_ppm = 0;
	int enable_biastee = 0;
	enum rtlsdr_ds_mode ds_mode = RTLSDR_DS_IQ;
	uint32_t ds_temp, ds_threshold = 0;
	int timeConstant = 75; /* default: U.S. 75 uS */
	int rtlagc = 0;
	dongle_init(&dongle);
	demod_init(&demod);
	output_init(&output);
	controller_init(&controller);
	cmd_init(&cmd);

	while ((opt = getopt(argc, argv, "d:f:C:B:m:g:s:b:l:L:o:t:r:p:E:q:F:A:M:c:h:w:W:D:Tnv")) != -1) {
		switch (opt) {
		case 'd':
			dongle.dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			if (controller.freq_len >= FREQUENCIES_LIMIT) {
				break;}
			if (strchr(optarg, ':'))
				{frequency_range(&controller, optarg);}
			else
			{
				controller.freqs[controller.freq_len] = (uint32_t)atofs(optarg);
				controller.freq_len++;
			}
			break;
		case 'C':
			cmd.filename = optarg;
			demod.mode_demod = &raw_demod;
			break;
		case 'm':
			MinCaptureRate = (int)atofs(optarg);
			break;
		case 'B':
			BufferDump = atoi(optarg);
			break;
		case 'n':
			OutputToStdout = 0;
			break;
		case 'g':
			dongle.gain = (int)(atof(optarg) * 10);
			break;
		case 'l':
			demod.squelch_level = (int)atof(optarg);
			break;
		case 'L':
			printLevels = (int)atof(optarg);
			break;
		case 's':
			demod.rate_in = (uint32_t)atofs(optarg);
			demod.rate_out = (uint32_t)atofs(optarg);
			break;
		case 'r':
			output.rate = (int)atofs(optarg);
			demod.rate_out2 = (int)atofs(optarg);
			break;
		case 'o':
			fprintf(stderr, "Warning: -o is very buggy\n");
			demod.post_downsample = (int)atof(optarg);
			if (demod.post_downsample < 1 || demod.post_downsample > MAXIMUM_OVERSAMPLE) {
				fprintf(stderr, "Oversample must be between 1 and %i\n", MAXIMUM_OVERSAMPLE);}
			break;
		case 't':
			demod.conseq_squelch = (int)atof(optarg);
			if (demod.conseq_squelch < 0) {
				demod.conseq_squelch = -demod.conseq_squelch;
				demod.terminate_on_squelch = 1;
			}
			break;
		case 'p':
			dongle.ppm_error = atoi(optarg);
			custom_ppm = 1;
			break;
		case 'E':
			if (strcmp("edge",  optarg) == 0) {
				controller.edge = 1;}
			if (strcmp("dc", optarg) == 0 || strcmp("adc", optarg) == 0) {
				demod.dc_block_audio = 1;}
			if (strcmp("rdc", optarg) == 0) {
				demod.dc_block_raw = 1;}
			if (strcmp("deemp",  optarg) == 0) {
				demod.deemph = 1;}
			if (strcmp("direct",  optarg) == 0) {
				dongle.direct_sampling = 1;}
			if (strcmp("offset",  optarg) == 0) {
				dongle.offset_tuning = 1;}
			if (strcmp("rtlagc", optarg) == 0 || strcmp("agc", optarg) == 0) {
				rtlagc = 1;}
			break;
		case 'q':
			demod.rdc_block_const = atoi(optarg);
			break;
		case 'F':
			demod.downsample_passes = 1;  /* truthy placeholder */
			demod.comp_fir_size = atoi(optarg);
			break;
		case 'A':
			if (strcmp("std",  optarg) == 0) {
				demod.custom_atan = 0;}
			if (strcmp("fast", optarg) == 0) {
				demod.custom_atan = 1;}
			if (strcmp("lut",  optarg) == 0) {
				atan_lut_init();
				demod.custom_atan = 2;}
			break;
		case 'M':
			if (strcmp("nbfm",  optarg) == 0 || strcmp("nfm",  optarg) == 0 || strcmp("fm",  optarg) == 0) {
				demod.mode_demod = &fm_demod;}
			if (strcmp("raw",  optarg) == 0 || strcmp("iq",  optarg) == 0) {
				demod.mode_demod = &raw_demod;}
			if (strcmp("am",  optarg) == 0) {
				demod.mode_demod = &am_demod;}
			if (strcmp("usb", optarg) == 0) {
				demod.mode_demod = &usb_demod;}
			if (strcmp("lsb", optarg) == 0) {
				demod.mode_demod = &lsb_demod;}
			if (strcmp("wbfm",  optarg) == 0 || strcmp("wfm",  optarg) == 0) {
				controller.wb_mode = 1;
				demod.mode_demod = &fm_demod;
				demod.rate_in = 170000;
				demod.rate_out = 170000;
				demod.rate_out2 = 32000;
				demod.custom_atan = 1;
				//demod.post_downsample = 4;
				demod.deemph = 1;
				demod.squelch_level = 0;}
			break;
		case 'c':
			if (strcmp("us",  optarg) == 0)
				timeConstant = 75;
			else if (strcmp("eu", optarg) == 0)
				timeConstant = 50;
			else
				timeConstant = (int)atof(optarg);
			break;
		case 'T':
			enable_biastee = 1;
			break;
		case 'D':
			ds_temp = (uint32_t)( atofs(optarg) + 0.5 );
			if (ds_temp <= RTLSDR_DS_Q_BELOW)
				ds_mode = (enum rtlsdr_ds_mode)ds_temp;
			else
				ds_threshold = ds_temp;
			break;
		case 'v':
			++verbosity;
			break;
		case 'w':
			dongle.bandwidth = (uint32_t)atofs(optarg);
			break;
		case 'W':
			dongle.buf_len = 512 * atoi(optarg);
			if (dongle.buf_len > MAXIMUM_BUF_LENGTH)
				dongle.buf_len = MAXIMUM_BUF_LENGTH;
			break;
		case 'h':
		case '?':
		default:
			usage();
			break;
		}
	}

	if (verbosity)
		fprintf(stderr, "verbosity set to %d\n", verbosity);

	/* quadruple sample_rate to limit to Δθ to ±π/2 */
	demod.rate_in *= demod.post_downsample;

	if (!output.rate) {
		output.rate = demod.rate_out;}

	sanity_checks();

	if (controller.freq_len > 1) {
		demod.terminate_on_squelch = 0;}

	if (optind < argc) {
		output.filename = argv[optind];
	} else {
		output.filename = "-";
	}

	ACTUAL_BUF_LENGTH = lcm_post[demod.post_downsample] * DEFAULT_BUF_LENGTH;

	if (!dev_given) {
		dongle.dev_index = verbose_device_search("0");
	}

	if (dongle.dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dongle.dev, (uint32_t)dongle.dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dongle.dev_index);
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

	if (demod.deemph) {
		double tc = (double)timeConstant * 1e-6;
		demod.deemph_a = (int)round(1.0/((1.0-exp(-1.0/(demod.rate_out * tc)))));
		if (verbosity)
			fprintf(stderr, "using wbfm deemphasis filter with time constant %d us\n", timeConstant );
	}

	/* Set the tuner gain */
	if (dongle.gain == AUTO_GAIN) {
		verbose_auto_gain(dongle.dev);
	} else {
		dongle.gain = nearest_gain(dongle.dev, dongle.gain);
		verbose_gain_set(dongle.dev, dongle.gain);
	}

	rtlsdr_set_agc_mode(dongle.dev, rtlagc);

	rtlsdr_set_bias_tee(dongle.dev, enable_biastee);
	if (enable_biastee)
		fprintf(stderr, "activated bias-T on GPIO PIN 0\n");

	verbose_ppm_set(dongle.dev, dongle.ppm_error);

	/* Set direct sampling with threshold */
	rtlsdr_set_ds_mode(dongle.dev, ds_mode, ds_threshold);

	verbose_set_bandwidth(dongle.dev, dongle.bandwidth);

	if (verbosity && dongle.bandwidth)
	{
		int r;
		uint32_t in_bw, out_bw, last_bw = 0;
		fprintf(stderr, "Supported bandwidth values in kHz:\n");
		for ( in_bw = 1; in_bw < 3200; ++in_bw )
		{
			r = rtlsdr_set_and_get_tuner_bandwidth(dongle.dev, in_bw*1000, &out_bw, 0 /* =apply_bw */);
			if ( r == 0 && out_bw != 0 && ( out_bw != last_bw || in_bw == 1 ) )
				fprintf(stderr, "%s%.1f", (in_bw==1 ? "" : ", "), out_bw/1000.0 );
			last_bw = out_bw;
		}
		fprintf(stderr,"\n");
	}

	if (strcmp(output.filename, "-") == 0) { /* Write samples to stdout */
		output.file = stdout;
#ifdef _WIN32
		_setmode(_fileno(output.file), _O_BINARY);
#endif
	} else {
		output.file = fopen(output.filename, "wb");
		if (!output.file) {
			fprintf(stderr, "Failed to open %s\n", output.filename);
			exit(1);
		}
		else
			fprintf(stderr, "Open %s for write\n", output.filename);
	}

	//r = rtlsdr_set_testmode(dongle.dev, 1);

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dongle.dev);

	pthread_create(&controller.thread, NULL, controller_thread_fn, (void *)(&controller));
	usleep(1000000); /* it looks, that startup of dongle level takes some time at startup! */
	pthread_create(&output.thread, NULL, output_thread_fn, (void *)(&output));
	pthread_create(&demod.thread, NULL, demod_thread_fn, (void *)(&demod));
	pthread_create(&dongle.thread, NULL, dongle_thread_fn, (void *)(&dongle));

	while (!do_exit) {
		usleep(100000);
	}

	if (do_exit) {
		fprintf(stderr, "\nUser cancel, exiting...\n");}
	else {
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);}

	rtlsdr_cancel_async(dongle.dev);
	pthread_join(dongle.thread, NULL);
	safe_cond_signal(&demod.ready, &demod.ready_m);
	pthread_join(demod.thread, NULL);
	safe_cond_signal(&output.ready, &output.ready_m);
	pthread_join(output.thread, NULL);
	safe_cond_signal(&controller.hop, &controller.hop_m);
	pthread_join(controller.thread, NULL);

	//dongle_cleanup(&dongle);
	demod_cleanup(&demod);
	output_cleanup(&output);
	controller_cleanup(&controller);

	if (cmd.filename) {
		int k;
		/* output scan statistics */
		for (k = 0; k < FREQUENCIES_LIMIT; k++) {
			if (cmd.statNumLevels[k] > 0)
				fprintf(stderr, "%u, %.1f, %.2f, %.1f\n", cmd.statFreq[k], cmd.statMinLevel[k], cmd.statSumLevels[k] / cmd.statNumLevels[k], cmd.statMaxLevel[k] );
		}
	}

	if (output.file != stdout) {
		fclose(output.file);}

	rtlsdr_close(dongle.dev);
	return r >= 0 ? r : -r;
}

// vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab
