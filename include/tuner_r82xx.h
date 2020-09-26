/*
 * Rafael Micro R820T/R828D driver
 *
 * Copyright (C) 2013 Mauro Carvalho Chehab <mchehab@redhat.com>
 * Copyright (C) 2013 Steve Markgraf <steve@steve-m.de>
 *
 * This driver is a heavily modified version of the driver found in the
 * Linux kernel:
 * http://git.linuxtv.org/linux-2.6.git/history/HEAD:/drivers/media/tuners/r820t.c
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

#ifndef R82XX_H
#define R82XX_H

#define R820T_I2C_ADDR		0x34
#define R828D_I2C_ADDR		0x74
#define R828D_XTAL_FREQ		16000000

#define R82XX_CHECK_ADDR	0x00
#define R82XX_CHECK_VAL		0x69

#define R82XX_IF_FREQ		3570000

#define REG_SHADOW_START	5
#define NUM_REGS			32
#define NUM_IMR				5
#define IMR_TRIAL			9

#define VER_NUM				49

#define USE_R82XX_ENV_VARS	0

enum r82xx_chip {
	CHIP_R820T,
	CHIP_R620D,
	CHIP_R828D,
	CHIP_R828,
	CHIP_R828S,
	CHIP_R820C,
};

enum r82xx_tuner_type {
	TUNER_RADIO = 1,
	TUNER_ANALOG_TV,
	TUNER_DIGITAL_TV
};

enum r82xx_xtal_cap_value {
	XTAL_LOW_CAP_30P = 0,
	XTAL_LOW_CAP_20P,
	XTAL_LOW_CAP_10P,
	XTAL_LOW_CAP_0P,
	XTAL_HIGH_CAP_0P
};

struct r82xx_config {
	uint8_t i2c_addr;
	uint8_t vco_curr_min;  /* VCO min/max current for R18/0x12 bits [7:5] in 0 .. 7. use 0xff for default */
	uint8_t vco_curr_max;  /* value is inverted: programmed is 7-value, that 0 is lowest current */
	uint8_t vco_algo;
	int harmonic;
	uint32_t xtal;
	enum r82xx_chip rafael_chip;
	unsigned int max_i2c_msg_len;
	int use_predetect;
	int verbose;
};

struct r82xx_priv {
	struct r82xx_config		*cfg;

	uint8_t						regs[NUM_REGS];
	uint8_t						buf[NUM_REGS + 1];
	uint8_t						override_data[NUM_REGS];
	uint8_t						override_mask[NUM_REGS];
	enum r82xx_xtal_cap_value	xtal_cap_sel;
	uint16_t					pll;	/* kHz */
	uint64_t					rf_freq;  /* frequency from r82xx_set_freq() */
	uint32_t					int_freq; /* if frequency at which to deliver towards RTL2832U */
	int32_t						if_band_center_freq;	/* frequency relative to zero IF,
														 * on which the band center shall be positioned */
	uint8_t						fil_cal_code;
	uint8_t						input;
	uint8_t						last_vco_curr;
	int							has_lock;
	int							tuner_pll_set;
	int							tuner_harmonic;
	int							init_done;
	int							sideband;
	int							disable_dither;

	/* Store current mode */
	uint32_t				delsys;
	enum r82xx_tuner_type	type;
	uint32_t				bw;	/* in MHz */
	void 					*rtl_dev;

	int				last_if_mode;
	int				last_manual_gain;
	int				last_extended_mode;
	int				last_LNA_value;
	int				last_Mixer_value;
	int				last_VGA_value;

#if USE_R82XX_ENV_VARS
	/* store some environment variables */
	int printI2C;
	unsigned int filterCenter;
	unsigned int haveR9, valR9;
	unsigned int haveR10L, valR10L;
	unsigned int haveR10H, valR10H;
	unsigned int haveR11L, valR11L;
	unsigned int haveR11H, valR11H;
	unsigned int haveR13L, valR13L;
	unsigned int haveR13H, valR13H;
	unsigned int haveR14L, valR14L;
	unsigned int haveR14H, valR14H;
	unsigned int haveR30H, valR30H;
	unsigned int haveR30L, valR30L;
#endif
};

struct r82xx_freq_range {
	uint32_t	freq;
	uint8_t		open_d;
	uint8_t		rf_mux_ploy;
	uint8_t		tf_c;
	uint8_t		xtal_cap20p;
	uint8_t		xtal_cap10p;
	uint8_t		xtal_cap0p;
};

enum r82xx_delivery_system {
	SYS_UNDEFINED,
	SYS_DVBT,
	SYS_DVBT2,
	SYS_ISDBT,
};

int r82xx_standby(struct r82xx_priv *priv);
int r82xx_init(struct r82xx_priv *priv);
int r82xx_set_freq(struct r82xx_priv *priv, uint32_t freq);
int r82xx_set_freq64(struct r82xx_priv *priv, uint64_t freq);
int r82xx_is_tuner_locked(struct r82xx_priv *priv);
int r82xx_set_gain(struct r82xx_priv *priv, int set_manual_gain, int gain, int extended_mode, int lna_gain, int mixer_gain, int vga_gain, int *rtl_vga_control);
int r82xx_get_rf_gain(struct r82xx_priv *priv);
int r82xx_get_if_gain(struct r82xx_priv *priv);

int r82xx_set_if_mode(struct r82xx_priv *priv, int if_mode, int *rtl_vga_control);

int r82xx_set_i2c_register(struct r82xx_priv *priv, unsigned i2c_register, unsigned data, unsigned mask);
int r82xx_get_i2c_register(struct r82xx_priv *priv, unsigned char* data, int len);
int r82xx_set_i2c_override(struct r82xx_priv *priv, unsigned i2c_register, unsigned data, unsigned mask);

int r82xx_set_bandwidth(struct r82xx_priv *priv, int bandwidth,  uint32_t rate, uint32_t * applied_bw, int apply);
int r82xx_set_bw_center(struct r82xx_priv *priv, int32_t if_band_center_freq);
/* Mixer Sideband:  0: lower, 1: upper */
int r82xx_set_sideband(struct r82xx_priv *priv, int sideband);
int r82xx_get_sideband(struct r82xx_priv *priv);
/* should rtlsdr flip the spectrum? */
int r82xx_flip_rtl_sideband(struct r82xx_priv *priv);
int r82xx_set_dither(struct r82xx_priv *priv, int dither);

int r82xx_read_cache_reg(struct r82xx_priv *priv, int reg);
int r82xx_write_reg_mask(struct r82xx_priv *priv, uint8_t reg, uint8_t val,uint8_t bit_mask);
int r82xx_write_reg_mask_ext(struct r82xx_priv *priv, uint8_t reg, uint8_t val, uint8_t bit_mask, const char * func_name);

#endif

