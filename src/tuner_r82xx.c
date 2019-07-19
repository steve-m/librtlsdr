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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "rtlsdr_i2c.h"
#include "tuner_r82xx.h"

#define WITH_ASYM_FILTER	1


#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define MHZ(x)		((x)*1000*1000)
#define KHZ(x)		((x)*1000)

/*
 * Static constants
 */

/* Those initial values start from REG_SHADOW_START */
static const uint8_t r82xx_init_array[NUM_REGS] = {
	0x83, 0x32, 0x75,				/* 05 to 07 */
	0xc0, 0x40, 0xd6, 0x6c,			/* 08 to 0b */
	0xf5, 0x63, 0x75, 0x68,			/* 0c to 0f */
	0x6c, 0x83, 0x80, 0x00,			/* 10 to 13 */
	0x0f, 0x00, 0xc0, 0x30,			/* 14 to 17 */
	0x48, 0xcc, 0x60, 0x00,			/* 18 to 1b */
	0x54, 0xae, 0x4a, 0xc0			/* 1c to 1f */
};

/* Tuner frequency ranges */
static const struct r82xx_freq_range freq_ranges[] = {
	{
	/* .freq = */			0,	/* Start freq, in MHz */
	/* .open_d = */			0x08,	/* low */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0xdf,	/* R27[7:0]  band2,band0 */
	/* .xtal_cap20p = */	0x02,	/* R16[1:0]  20pF (10)   */
	/* .xtal_cap10p = */	0x01,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			50,	/* Start freq, in MHz */
	/* .open_d = */			0x08,	/* low */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0xbe,	/* R27[7:0]  band4,band1  */
	/* .xtal_cap20p = */	0x02,	/* R16[1:0]  20pF (10)   */
	/* .xtal_cap10p = */	0x01,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			55,	/* Start freq, in MHz */
	/* .open_d = */			0x08,	/* low */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x8b,	/* R27[7:0]  band7,band4 */
	/* .xtal_cap20p = */	0x02,	/* R16[1:0]  20pF (10)   */
	/* .xtal_cap10p = */	0x01,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			60,	/* Start freq, in MHz */
	/* .open_d = */			0x08,	/* low */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x7b,	/* R27[7:0]  band8,band4 */
	/* .xtal_cap20p = */	0x02,	/* R16[1:0]  20pF (10)   */
	/* .xtal_cap10p = */	0x01,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			65,	/* Start freq, in MHz */
	/* .open_d = */			0x08,	/* low */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x69,	/* R27[7:0]  band9,band6 */
	/* .xtal_cap20p = */	0x02,	/* R16[1:0]  20pF (10)   */
	/* .xtal_cap10p = */	0x01,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			70,	/* Start freq, in MHz */
	/* .open_d = */			0x08,	/* low */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x58,	/* R27[7:0]  band10,band7 */
	/* .xtal_cap20p = */	0x02,	/* R16[1:0]  20pF (10)   */
	/* .xtal_cap10p = */	0x01,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			75,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x44,	/* R27[7:0]  band11,band11 */
	/* .xtal_cap20p = */	0x02,	/* R16[1:0]  20pF (10)   */
	/* .xtal_cap10p = */	0x01,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			80,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x44,	/* R27[7:0]  band11,band11 */
	/* .xtal_cap20p = */	0x02,	/* R16[1:0]  20pF (10)   */
	/* .xtal_cap10p = */	0x01,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			90,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x34,	/* R27[7:0]  band12,band11 */
	/* .xtal_cap20p = */	0x01,	/* R16[1:0]  10pF (01)   */
	/* .xtal_cap10p = */	0x01,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			100,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x34,	/* R27[7:0]  band12,band11 */
	/* .xtal_cap20p = */	0x01,	/* R16[1:0]  10pF (01)    */
	/* .xtal_cap10p = */	0x01,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			110,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x24,	/* R27[7:0]  band13,band11 */
	/* .xtal_cap20p = */	0x01,	/* R16[1:0]  10pF (01)   */
	/* .xtal_cap10p = */	0x01,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			120,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x24,	/* R27[7:0]  band13,band11 */
	/* .xtal_cap20p = */	0x01,	/* R16[1:0]  10pF (01)   */
	/* .xtal_cap10p = */	0x01,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			140,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x14,	/* R27[7:0]  band14,band11 */
	/* .xtal_cap20p = */	0x01,	/* R16[1:0]  10pF (01)   */
	/* .xtal_cap10p = */	0x01,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			180,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x13,	/* R27[7:0]  band14,band12 */
	/* .xtal_cap20p = */	0x00,	/* R16[1:0]  0pF (00)   */
	/* .xtal_cap10p = */	0x00,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			220,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x13,	/* R27[7:0]  band14,band12 */
	/* .xtal_cap20p = */	0x00,	/* R16[1:0]  0pF (00)   */
	/* .xtal_cap10p = */	0x00,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			250,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x11,	/* R27[7:0]  highest,highest */
	/* .xtal_cap20p = */	0x00,	/* R16[1:0]  0pF (00)   */
	/* .xtal_cap10p = */	0x00,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			280,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x02,	/* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
	/* .tf_c = */			0x00,	/* R27[7:0]  highest,highest */
	/* .xtal_cap20p = */	0x00,	/* R16[1:0]  0pF (00)   */
	/* .xtal_cap10p = */	0x00,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			310,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x41,	/* R26[7:6]=1 (bypass)  R26[1:0]=1 (middle) */
	/* .tf_c = */			0x00,	/* R27[7:0]  highest,highest */
	/* .xtal_cap20p = */	0x00,	/* R16[1:0]  0pF (00)   */
	/* .xtal_cap10p = */	0x00,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			450,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x41,	/* R26[7:6]=1 (bypass)  R26[1:0]=1 (middle) */
	/* .tf_c = */			0x00,	/* R27[7:0]  highest,highest */
	/* .xtal_cap20p = */	0x00,	/* R16[1:0]  0pF (00)   */
	/* .xtal_cap10p = */	0x00,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			588,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x40,	/* R26[7:6]=1 (bypass)  R26[1:0]=0 (highest) */
	/* .tf_c = */			0x00,	/* R27[7:0]  highest,highest */
	/* .xtal_cap20p = */	0x00,	/* R16[1:0]  0pF (00)   */
	/* .xtal_cap10p = */	0x00,
	/* .xtal_cap0p = */		0x00,
	}, {
	/* .freq = */			650,	/* Start freq, in MHz */
	/* .open_d = */			0x00,	/* high */
	/* .rf_mux_ploy = */	0x40,	/* R26[7:6]=1 (bypass)  R26[1:0]=0 (highest) */
	/* .tf_c = */			0x00,	/* R27[7:0]  highest,highest */
	/* .xtal_cap20p = */	0x00,	/* R16[1:0]  0pF (00)   */
	/* .xtal_cap10p = */	0x00,
	/* .xtal_cap0p = */		0x00,
	}
};

static int r82xx_xtal_capacitor[][2] = {
	{ 0x0b, XTAL_LOW_CAP_30P },
	{ 0x02, XTAL_LOW_CAP_20P },
	{ 0x01, XTAL_LOW_CAP_10P },
	{ 0x00, XTAL_LOW_CAP_0P  },
	{ 0x10, XTAL_HIGH_CAP_0P },
};

/*
 * I2C read/write code and shadow registers logic
 */
static void shadow_store(struct r82xx_priv *priv, uint8_t reg, const uint8_t *val,
			 int len)
{
	int r = reg - REG_SHADOW_START;

	if (r < 0) {
		len += r;
		r = 0;
	}
	if (len <= 0)
		return;
	if (len > NUM_REGS - r)
		len = NUM_REGS - r;

	memcpy(&priv->regs[r], val, len);
}

static int r82xx_write_arr(struct r82xx_priv *priv, uint8_t reg, const uint8_t *val,
			   unsigned int len)
{
	int rc, size, k, regOff, regIdx, bufIdx, pos = 0;

	/* Store the shadow registers */
	shadow_store(priv, reg, val, len);

	do {
		if (len > priv->cfg->max_i2c_msg_len - 1)
			size = priv->cfg->max_i2c_msg_len - 1;
		else
			size = len;

		/* Fill I2C buffer */
		priv->buf[0] = reg;
		memcpy(&priv->buf[1], &val[pos], size);

		/* override data in buffer */
		for ( k = 0; k < size; ++k ) {
			regOff = pos + k;
			regIdx = reg - REG_SHADOW_START + regOff;
			if ( priv->override_mask[regIdx] ) {
				uint8_t oldBuf = priv->buf[1 + k];
				bufIdx = 1 + k;
				priv->buf[bufIdx] = ( priv->buf[bufIdx] & (~ priv->override_mask[regIdx]) )
								| ( priv->override_mask[regIdx] & priv->override_data[regIdx] );
				fprintf(stderr, "override writing register %d = x%02X value x%02X  by data x%02X mask x%02X => new value x%02X\n"
						, regIdx + REG_SHADOW_START
						, regIdx + REG_SHADOW_START
						, oldBuf
						, priv->override_data[regIdx]
						, priv->override_mask[regIdx]
						, priv->buf[bufIdx]
						);
			}
		}

		rc = rtlsdr_i2c_write_fn(priv->rtl_dev, priv->cfg->i2c_addr,
					 priv->buf, size + 1);

		if (rc != size + 1) {
			fprintf(stderr, "%s: i2c wr failed=%d reg=%02x len=%d\n",
				   __FUNCTION__, rc, reg, size);
			if (rc < 0)
				return rc;
			return -1;
		}

		reg += size;
		len -= size;
		pos += size;
	} while (len > 0);

	return 0;
}

static int r82xx_write_reg(struct r82xx_priv *priv, uint8_t reg, uint8_t val)
{
	return r82xx_write_arr(priv, reg, &val, 1);
}

int r82xx_read_cache_reg(struct r82xx_priv *priv, int reg)
{
	reg -= REG_SHADOW_START;

	if (reg >= 0 && reg < NUM_REGS)
		return priv->regs[reg];
	else
		return -1;
}

int r82xx_write_reg_mask(struct r82xx_priv *priv, uint8_t reg, uint8_t val, uint8_t bit_mask)
{
	int rc = r82xx_read_cache_reg(priv, reg);

	if (rc < 0)
		return rc;

	val = (rc & ~bit_mask) | (val & bit_mask);

	return r82xx_write_arr(priv, reg, &val, 1);
}

int r82xx_write_reg_mask_ext(struct r82xx_priv *priv, uint8_t reg, uint8_t val,
	uint8_t bit_mask, const char * func_name)
{
	int r;
#if USE_R82XX_ENV_VARS
	if (priv->printI2C) {
		fprintf(stderr, "%s: setting I2C register %02X: old value = %02X, new value: %02X with mask %02X\n"
			, func_name, reg
			, r82xx_read_cache_reg(priv, reg)
			, val, bit_mask );
	}
#endif
	r = r82xx_write_reg_mask(priv, reg, val, bit_mask);
	return r;
}



static uint8_t r82xx_bitrev(uint8_t byte)
{
	const uint8_t lut[16] = { 0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
				  0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };

	return (lut[byte & 0xf] << 4) | lut[byte >> 4];
}

static int r82xx_read(struct r82xx_priv *priv, uint8_t reg, uint8_t *val, int len)
{
	int rc, i;
	uint8_t *p = &priv->buf[1];

	priv->buf[0] = reg;

	rc = rtlsdr_i2c_write_fn(priv->rtl_dev, priv->cfg->i2c_addr, priv->buf, 1);
	if (rc < 1)
		return rc;

	rc = rtlsdr_i2c_read_fn(priv->rtl_dev, priv->cfg->i2c_addr, p, len);

	if (rc != len) {
		fprintf(stderr, "%s: i2c rd failed=%d reg=%02x len=%d\n",
			   __FUNCTION__, rc, reg, len);
		if (rc < 0)
			return rc;
		return -1;
	}

	/* Copy data to the output buffer */
	for (i = 0; i < len; i++)
		val[i] = r82xx_bitrev(p[i]);

	return 0;
}

/*
 * r82xx tuning logic
 */

static int r82xx_set_mux(struct r82xx_priv *priv, uint32_t freq)
{
	const struct r82xx_freq_range *range;
	int rc;
	unsigned int i;
	uint8_t val;

	/* Get the proper frequency range */
	freq = freq / 1000000;
	for (i = 0; i < ARRAY_SIZE(freq_ranges) - 1; i++) {
		if (freq < freq_ranges[i + 1].freq)
			break;
	}
	range = &freq_ranges[i];

	/* Open Drain */
	rc = r82xx_write_reg_mask(priv, 0x17, range->open_d, 0x08);
	if (rc < 0)
		return rc;

	/* RF_MUX,Polymux */
	rc = r82xx_write_reg_mask(priv, 0x1a, range->rf_mux_ploy, 0xc3);
	if (rc < 0)
		return rc;

	/* TF BAND */
	rc = r82xx_write_reg(priv, 0x1b, range->tf_c);
	if (rc < 0)
		return rc;

	/* XTAL CAP & Drive */
	switch (priv->xtal_cap_sel) {
	case XTAL_LOW_CAP_30P:
	case XTAL_LOW_CAP_20P:
		val = range->xtal_cap20p | 0x08;
		break;
	case XTAL_LOW_CAP_10P:
		val = range->xtal_cap10p | 0x08;
		break;
	case XTAL_HIGH_CAP_0P:
		val = range->xtal_cap0p | 0x00;
		break;
	default:
	case XTAL_LOW_CAP_0P:
		val = range->xtal_cap0p | 0x08;
		break;
	}
	rc = r82xx_write_reg_mask(priv, 0x10, val, 0x0b);
	if (rc < 0)
		return rc;

	rc = r82xx_write_reg_mask(priv, 0x08, 0x00, 0x3f);
	if (rc < 0)
		return rc;

	rc = r82xx_write_reg_mask(priv, 0x09, 0x00, 0x3f);

	return rc;
}

static int r82xx_set_pll(struct r82xx_priv *priv, uint32_t freq)
{
	int rc, i;
	uint64_t vco_freq;
	uint64_t vco_div;
	uint32_t vco_min = 1770000; /* kHz */
	uint32_t vco_max = vco_min * 2; /* kHz */
	uint32_t freq_khz, pll_ref;
	uint32_t sdm = 0;
	uint8_t mix_div = 2;
	uint8_t div_buf = 0;
	uint8_t div_num = 0;
	uint8_t vco_power_ref = 2;
	uint8_t refdiv2 = 0;
	uint8_t ni, si, nint, vco_fine_tune, val;
	uint8_t data[5];

	/* Frequency in kHz */
	freq_khz = (freq + 500) / 1000;
	pll_ref = priv->cfg->xtal;

	rc = r82xx_write_reg_mask(priv, 0x10, refdiv2, 0x10);
	if (rc < 0)
		return rc;

	/* set pll autotune = 128kHz */
	rc = r82xx_write_reg_mask(priv, 0x1a, 0x00, 0x0c);
	if (rc < 0)
		return rc;

	/* set VCO current = 100 */
	rc = r82xx_write_reg_mask(priv, 0x12, 0x80, 0xe0);
	if (rc < 0)
		return rc;

	/* Calculate divider */
	while (mix_div <= 64) {
		if (((freq_khz * mix_div) >= vco_min) &&
		   ((freq_khz * mix_div) < vco_max)) {
			div_buf = mix_div;
			while (div_buf > 2) {
				div_buf = div_buf >> 1;
				div_num++;
			}
			break;
		}
		mix_div = mix_div << 1;
	}

	rc = r82xx_read(priv, 0x00, data, sizeof(data));
	if (rc < 0)
		return rc;

	if (priv->cfg->rafael_chip == CHIP_R828D)
		vco_power_ref = 1;

	vco_fine_tune = (data[4] & 0x30) >> 4;

	if (vco_fine_tune > vco_power_ref)
		div_num = div_num - 1;
	else if (vco_fine_tune < vco_power_ref)
		div_num = div_num + 1;

	rc = r82xx_write_reg_mask(priv, 0x10, div_num << 5, 0xe0);
	if (rc < 0)
		return rc;

	vco_freq = (uint64_t)freq * (uint64_t)mix_div;

	/*
	 * We want to approximate:
	 *
	 *  vco_freq / (2 * pll_ref)
	 *
	 * in the form:
	 *
	 *  nint + sdm/65536
	 *
	 * where nint,sdm are integers and 0 < nint, 0 <= sdm < 65536
	 * 
	 * Scaling to fixed point and rounding:
	 *
	 *  vco_div = 65536*(nint + sdm/65536) = int( 0.5 + 65536 * vco_freq / (2 * pll_ref) )
	 *  vco_div = 65536*nint + sdm         = int( (pll_ref + 65536 * vco_freq) / (2 * pll_ref) )
	 */
        
	vco_div = (pll_ref + 65536 * vco_freq) / (2 * pll_ref);
        nint = (uint32_t) (vco_div / 65536);
	sdm = (uint32_t) (vco_div % 65536);

#if 0
	{
	  uint64_t actual_vco = (uint64_t)2 * pll_ref * nint + (uint64_t)2 * pll_ref * sdm / 65536;
	  fprintf(stderr, "[R82XX] requested %uHz; selected mix_div=%u vco_freq=%lu nint=%u sdm=%u; actual_vco=%lu; tuning error=%+dHz\n",
		  freq, mix_div, vco_freq, nint, sdm, actual_vco, (int32_t) (actual_vco - vco_freq) / mix_div);
	}
#endif

	if (nint > ((128 / vco_power_ref) - 1)) {
		fprintf(stderr, "[R82XX] No valid PLL values for %u Hz!\n", freq);
		return -1;
	}

	ni = (nint - 13) / 4;
	si = nint - 4 * ni - 13;

	rc = r82xx_write_reg(priv, 0x14, ni + (si << 6));
	if (rc < 0)
		return rc;

	/* pw_sdm */
	if (sdm == 0)
		val = 0x08;
	else
		val = 0x00;

	rc = r82xx_write_reg_mask(priv, 0x12, val, 0x08);
	if (rc < 0)
		return rc;

	rc = r82xx_write_reg(priv, 0x16, sdm >> 8);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg(priv, 0x15, sdm & 0xff);
	if (rc < 0)
		return rc;

	for (i = 0; i < 2; i++) {
//		usleep_range(sleep_time, sleep_time + 1000);

		/* Check if PLL has locked */
		rc = r82xx_read(priv, 0x00, data, 3);
		if (rc < 0)
			return rc;
		if (data[2] & 0x40)
			break;

		if (!i) {
			/* Didn't lock. Increase VCO current */
			rc = r82xx_write_reg_mask(priv, 0x12, 0x60, 0xe0);
			if (rc < 0)
				return rc;
		}
	}

	if (!(data[2] & 0x40)) {
		fprintf(stderr, "[R82XX] PLL not locked for %u Hz!\n", freq);
		priv->has_lock = 0;
		return 0;
	}

	priv->has_lock = 1;

	/* set pll autotune = 8kHz */
	rc = r82xx_write_reg_mask(priv, 0x1a, 0x08, 0x08);

	return rc;
}

static int r82xx_sysfreq_sel(struct r82xx_priv *priv, uint32_t freq,
				 enum r82xx_tuner_type type,
				 uint32_t delsys)
{
	int rc;
	uint8_t mixer_top, lna_top, cp_cur, div_buf_cur, lna_vth_l, mixer_vth_l;
	uint8_t air_cable1_in, cable2_in, pre_dect, lna_discharge, filter_cur;

	switch (delsys) {
	case SYS_DVBT:
		if ((freq == 506000000) || (freq == 666000000) ||
		   (freq == 818000000)) {
			mixer_top = 0x14;	/* mixer top:14 , top-1, low-discharge */
			lna_top = 0xe5;		/* detect bw 3, lna top:4, predet top:2 */
			cp_cur = 0x28;		/* 101, 0.2 */
			div_buf_cur = 0x20;	/* 10, 200u */
		} else {
			mixer_top = 0x24;	/* mixer top:13 , top-1, low-discharge */
			lna_top = 0xe5;		/* detect bw 3, lna top:4, predet top:2 */
			cp_cur = 0x38;		/* 111, auto */
			div_buf_cur = 0x30;	/* 11, 150u */
		}
		lna_vth_l = 0x53;		/* lna vth 0.84	,  vtl 0.64 */
		mixer_vth_l = 0x75;		/* mixer vth 1.04, vtl 0.84 */
		air_cable1_in = 0x00;
		cable2_in = 0x00;
		pre_dect = 0x40;
		lna_discharge = 14;
		filter_cur = 0x40;		/* 10, low */
		break;
	case SYS_DVBT2:
		mixer_top = 0x24;	/* mixer top:13 , top-1, low-discharge */
		lna_top = 0xe5;		/* detect bw 3, lna top:4, predet top:2 */
		lna_vth_l = 0x53;	/* lna vth 0.84	,  vtl 0.64 */
		mixer_vth_l = 0x75;	/* mixer vth 1.04, vtl 0.84 */
		air_cable1_in = 0x00;
		cable2_in = 0x00;
		pre_dect = 0x40;
		lna_discharge = 14;
		cp_cur = 0x38;		/* 111, auto */
		div_buf_cur = 0x30;	/* 11, 150u */
		filter_cur = 0x40;	/* 10, low */
		break;
	case SYS_ISDBT:
		mixer_top = 0x24;	/* mixer top:13 , top-1, low-discharge */
		lna_top = 0xe5;		/* detect bw 3, lna top:4, predet top:2 */
		lna_vth_l = 0x75;	/* lna vth 1.04	,  vtl 0.84 */
		mixer_vth_l = 0x75;	/* mixer vth 1.04, vtl 0.84 */
		air_cable1_in = 0x00;
		cable2_in = 0x00;
		pre_dect = 0x40;
		lna_discharge = 14;
		cp_cur = 0x38;		/* 111, auto */
		div_buf_cur = 0x30;	/* 11, 150u */
		filter_cur = 0x40;	/* 10, low */
		break;
	default: /* DVB-T 8M */
		mixer_top = 0x24;	/* mixer top:13 , top-1, low-discharge */
		lna_top = 0xe5;		/* detect bw 3, lna top:4, predet top:2 */
		lna_vth_l = 0x53;	/* lna vth 0.84	,  vtl 0.64 */
		mixer_vth_l = 0x75;	/* mixer vth 1.04, vtl 0.84 */
		air_cable1_in = 0x00;
		cable2_in = 0x00;
		pre_dect = 0x40;
		lna_discharge = 14;
		cp_cur = 0x38;		/* 111, auto */
		div_buf_cur = 0x30;	/* 11, 150u */
		filter_cur = 0x40;	/* 10, low */
		break;
	}

	if (priv->cfg->use_predetect) {
		rc = r82xx_write_reg_mask(priv, 0x06, pre_dect, 0x40);
		if (rc < 0)
			return rc;
	}

	rc = r82xx_write_reg_mask(priv, 0x1d, lna_top, 0xc7);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg_mask(priv, 0x1c, mixer_top, 0xf8);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg(priv, 0x0d, lna_vth_l);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg(priv, 0x0e, mixer_vth_l);
	if (rc < 0)
		return rc;

	priv->input = air_cable1_in;

	/* Air-IN only for Astrometa */
	rc = r82xx_write_reg_mask(priv, 0x05, air_cable1_in, 0x60);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg_mask(priv, 0x06, cable2_in, 0x08);
	if (rc < 0)
		return rc;

	rc = r82xx_write_reg_mask(priv, 0x11, cp_cur, 0x38);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg_mask(priv, 0x17, div_buf_cur, 0x30);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg_mask_ext(priv, 0x0a, filter_cur, 0x60, __FUNCTION__);
	if (rc < 0)
		return rc;

	/*
	 * Set LNA
	 */

	if (type != TUNER_ANALOG_TV) {
		/* LNA TOP: lowest */
		rc = r82xx_write_reg_mask(priv, 0x1d, 0, 0x38);
		if (rc < 0)
			return rc;

		/* 0: normal mode */
		rc = r82xx_write_reg_mask(priv, 0x1c, 0, 0x04);
		if (rc < 0)
			return rc;

		/* 0: PRE_DECT off */
		rc = r82xx_write_reg_mask(priv, 0x06, 0, 0x40);
		if (rc < 0)
			return rc;

		/* agc clk 250hz */
		rc = r82xx_write_reg_mask(priv, 0x1a, 0x30, 0x30);
		if (rc < 0)
			return rc;

//		msleep(250);

		/* write LNA TOP = 3 */
		rc = r82xx_write_reg_mask(priv, 0x1d, 0x18, 0x38);
		if (rc < 0)
			return rc;

		/*
		 * write discharge mode
		 * FIXME: IMHO, the mask here is wrong, but it matches
		 * what's there at the original driver
		 */
		rc = r82xx_write_reg_mask(priv, 0x1c, mixer_top, 0x04);
		if (rc < 0)
			return rc;

		/* LNA discharge current */
		rc = r82xx_write_reg_mask(priv, 0x1e, lna_discharge, 0x1f);
		if (rc < 0)
			return rc;

		/* agc clk 60hz */
		rc = r82xx_write_reg_mask(priv, 0x1a, 0x20, 0x30);
		if (rc < 0)
			return rc;
	} else {
		/* PRE_DECT off */
		rc = r82xx_write_reg_mask(priv, 0x06, 0, 0x40);
		if (rc < 0)
			return rc;

		/* write LNA TOP */
		rc = r82xx_write_reg_mask(priv, 0x1d, lna_top, 0x38);
		if (rc < 0)
			return rc;

		/*
		 * write discharge mode
		 * FIXME: IMHO, the mask here is wrong, but it matches
		 * what's there at the original driver
		 */
		rc = r82xx_write_reg_mask(priv, 0x1c, mixer_top, 0x04);
		if (rc < 0)
			return rc;

		/* LNA discharge current */
		rc = r82xx_write_reg_mask(priv, 0x1e, lna_discharge, 0x1f);
		if (rc < 0)
			return rc;

		/* agc clk 1Khz, external det1 cap 1u */
		rc = r82xx_write_reg_mask(priv, 0x1a, 0x00, 0x30);
		if (rc < 0)
			return rc;

		rc = r82xx_write_reg_mask(priv, 0x10, 0x00, 0x04);
		if (rc < 0)
			return rc;
	 }
	 return 0;
}

static int r82xx_set_tv_standard(struct r82xx_priv *priv,
				 unsigned bw,
				 enum r82xx_tuner_type type,
				 uint32_t delsys)

{
	int rc, i;
	uint32_t if_khz, filt_cal_lo;
	uint8_t data[5];
	uint8_t filt_gain, img_r, filt_q, hp_cor, ext_enable, loop_through;
	uint8_t lt_att, flt_ext_widest, polyfil_cur;
	int need_calibration;

	/* BW < 6 MHz */
	if_khz = 3570;
	filt_cal_lo = 56000;	/* 52000->56000 */
	filt_gain = 0x10;	/* +3db, 6mhz on */
	img_r = 0x00;		/* image negative */
	filt_q = 0x10;		/* r10[4]:low q(1'b1) */
	hp_cor = 0x6b;		/* 1.7m disable, +2cap, 1.0mhz */
	ext_enable = 0x60;	/* r30[6]=1 ext enable; r30[5]:1 ext at lna max-1 */
	loop_through = 0x01;	/* r5[7], lt off */
	lt_att = 0x00;		/* r31[7], lt att enable */
	flt_ext_widest = 0x00;	/* r15[7]: flt_ext_wide off */
	polyfil_cur = 0x60;	/* r25[6:5]:min */

	/* Initialize the shadow registers */
	memcpy(priv->regs, r82xx_init_array, sizeof(r82xx_init_array));

	/* Init Flag & Xtal_check Result (inits VGA gain, needed?)*/
	rc = r82xx_write_reg_mask(priv, 0x0c, 0x00, 0x0f);
	if (rc < 0)
		return rc;

	/* version */
	rc = r82xx_write_reg_mask(priv, 0x13, VER_NUM, 0x3f);
	if (rc < 0)
		return rc;

	/* for LT Gain test */
	if (type != TUNER_ANALOG_TV) {
		rc = r82xx_write_reg_mask(priv, 0x1d, 0x00, 0x38);
		if (rc < 0)
			return rc;
//		usleep_range(1000, 2000);
	}
	priv->if_band_center_freq = 0;
	priv->int_freq = if_khz * 1000;

	/* Check if standard changed. If so, filter calibration is needed */
	/* as we call this function only once in rtlsdr, force calibration */
	need_calibration = 1;

	if (need_calibration) {
		for (i = 0; i < 2; i++) {
			/* Set filt_cap */
			rc = r82xx_write_reg_mask_ext(priv, 0x0b, hp_cor, 0x60, __FUNCTION__);
			if (rc < 0)
				return rc;

			/* set cali clk =on */
			rc = r82xx_write_reg_mask(priv, 0x0f, 0x04, 0x04);
			if (rc < 0)
				return rc;

			/* X'tal cap 0pF for PLL */
			rc = r82xx_write_reg_mask(priv, 0x10, 0x00, 0x03);
			if (rc < 0)
				return rc;

			rc = r82xx_set_pll(priv, filt_cal_lo * 1000);
			if (rc < 0 || !priv->has_lock)
				return rc;

			/* Start Trigger */
			rc = r82xx_write_reg_mask_ext(priv, 0x0b, 0x10, 0x10, __FUNCTION__);
			if (rc < 0)
				return rc;

//			usleep_range(1000, 2000);

			/* Stop Trigger */
			rc = r82xx_write_reg_mask_ext(priv, 0x0b, 0x00, 0x10, __FUNCTION__);
			if (rc < 0)
				return rc;

			/* set cali clk =off */
			rc = r82xx_write_reg_mask(priv, 0x0f, 0x00, 0x04);
			if (rc < 0)
				return rc;

			/* Check if calibration worked */
			rc = r82xx_read(priv, 0x00, data, sizeof(data));
			if (rc < 0)
				return rc;

			priv->fil_cal_code = data[4] & 0x0f;
			if (priv->fil_cal_code && priv->fil_cal_code != 0x0f)
				break;
		}
		/* narrowest */
		if (priv->fil_cal_code == 0x0f)
			priv->fil_cal_code = 0;
	}

	rc = r82xx_write_reg_mask_ext(priv, 0x0a,
				  filt_q | priv->fil_cal_code, 0x1f, __FUNCTION__);
	if (rc < 0)
		return rc;

	/* Set BW, Filter_gain, & HP corner */
	rc = r82xx_write_reg_mask_ext(priv, 0x0b, hp_cor, 0xef, __FUNCTION__);
	if (rc < 0)
		return rc;

	/* Set Img_R */
	rc = r82xx_write_reg_mask(priv, 0x07, img_r, 0x80);
	if (rc < 0)
		return rc;

	/* Set filt_3dB, V6MHz */
	rc = r82xx_write_reg_mask(priv, 0x06, filt_gain, 0x30);
	if (rc < 0)
		return rc;

	/* channel filter extension */
	rc = r82xx_write_reg_mask_ext(priv, 0x1e, ext_enable, 0x60, __FUNCTION__);
	if (rc < 0)
		return rc;

	/* Loop through */
	rc = r82xx_write_reg_mask(priv, 0x05, loop_through, 0x80);
	if (rc < 0)
		return rc;

	/* Loop through attenuation */
	rc = r82xx_write_reg_mask(priv, 0x1f, lt_att, 0x80);
	if (rc < 0)
		return rc;

	/* filter extension widest */
	rc = r82xx_write_reg_mask(priv, 0x0f, flt_ext_widest, 0x80);
	if (rc < 0)
		return rc;

	/* RF poly filter current */
	rc = r82xx_write_reg_mask(priv, 0x19, polyfil_cur, 0x60);
	if (rc < 0)
		return rc;

	/* Store current standard. If it changes, re-calibrate the tuner */
	priv->delsys = delsys;
	priv->type = type;
	priv->bw = bw;

	return 0;
}

/* measured with a Racal 6103E GSM test set at 928 MHz with -60 dBm
 * input power, for raw results see:
 * http://steve-m.de/projects/rtl-sdr/gain_measurement/r820t/
 */

#define VGA_BASE_GAIN	-47
static const int r82xx_vga_gain_steps[]  = {
	0, 26, 26, 30, 42, 35, 24, 13, 14, 32, 36, 34, 35, 37, 35, 36
};

static const int r82xx_lna_gain_steps[]  = {
	0, 9, 13, 40, 38, 13, 31, 22, 26, 31, 26, 14, 19, 5, 35, 13
};

static const int r82xx_mixer_gain_steps[]  = {
	0, 5, 10, 10, 19, 9, 10, 25, 17, 10, 8, 16, 13, 6, 3, -8
};

static void r82xx_get_gain_index(int gain, int *ptr_lna_index, int *ptr_mix_index)
{
	int i, total_gain = 0;
	int mix_index = 0, lna_index = 0;

	for (i = 0; i < 15; i++) {
		if (total_gain >= gain)
			break;
		total_gain += r82xx_lna_gain_steps[++lna_index];
		if (total_gain >= gain)
			break;
		total_gain += r82xx_mixer_gain_steps[++mix_index];
	}

	*ptr_lna_index = lna_index;
	*ptr_mix_index = mix_index;
}

int r82xx_set_gain(struct r82xx_priv *priv, int set_manual_gain, int gain,
  int extended_mode, int lna_gain, int mixer_gain, int vga_gain, int *rtl_vga_control)
{
	int rc, i, new_agc_state = 0;
	uint8_t data[4];

	if (rtl_vga_control)
		*rtl_vga_control = 0;

	if ( !extended_mode && !set_manual_gain ) {
		new_agc_state = 1;
		/* map some agc modes to extended_mode */
		if ( priv->agc_mode == -1 ) {
			/* LNA/Mixer = last value from prev rtlsdr_set_tuner_gain; VGA = auto */
			lna_gain = priv->last_LNA_value;
			mixer_gain = priv->last_Mixer_value;
			vga_gain = priv->last_VGA_value | 0x10;  /* should activate AGC */
			extended_mode = 1;
		}
		else if ( priv->agc_mode > 0 ) {
			/* LNA/Mixer = from rtlsdr_set_tuner_gain(tunerAgcMode); VGA = auto */
			r82xx_get_gain_index(priv->agc_mode, &lna_gain, &mixer_gain);
			vga_gain = priv->last_VGA_value | 0x10;  /* should activate AGC */
			extended_mode = 1;
		}
	}
	else if ( !extended_mode && set_manual_gain ) {
		/* map set_manual_gain to extended_mode */
		r82xx_get_gain_index(gain, &lna_gain, &mixer_gain);
		vga_gain = 0x08;  /* = fixed 16.3 dB == -12 dB + 8 * 3.5 dB */
		extended_mode = 1;
	}

	if (extended_mode) {
		/* LNA auto off == manual */
		rc = r82xx_write_reg_mask(priv, 0x05, 0x10, 0x10);
		if (rc < 0)
			return rc;

		/* Mixer auto off == manual mode */
		rc = r82xx_write_reg_mask(priv, 0x07, 0, 0x10);
		if (rc < 0)
			return rc;

		rc = r82xx_read(priv, 0x00, data, sizeof(data));
		if (rc < 0)
			return rc;

		/* Set LNA */
		rc = r82xx_write_reg_mask(priv, 0x05, lna_gain, 0x0f);
		if (rc < 0)
			return rc;
		priv->last_LNA_value = lna_gain;

		/* Set Mixer */
		rc = r82xx_write_reg_mask(priv, 0x07, mixer_gain, 0x0f);
		if (rc < 0)
			return rc;
		priv->last_Mixer_value = mixer_gain;

		/* Set VGA */
		rc = r82xx_write_reg_mask(priv, 0x0c, vga_gain, 0x9f);
		if (rc < 0)
			return rc;
		priv->last_VGA_value = vga_gain & 0x0f;
		priv->last_AGC_state = new_agc_state;
		if ( (vga_gain & 0x10) && rtl_vga_control )
			*rtl_vga_control = 1;
	}
	else
	{
		/* agc_mode == 0: LNA/Mixer = auto; VGA = fixed 26.5 dB ==> auto */

		/* LNA auto on = AGC */
		rc = r82xx_write_reg_mask(priv, 0x05, 0, 0x10);
		if (rc < 0)
			return rc;

		/* Mixer auto on = AGC */
		rc = r82xx_write_reg_mask(priv, 0x07, 0x10, 0x10);
		if (rc < 0)
			return rc;

		/* set fixed VGA gain for now (26.5 dB == -12 dB + 0x0b * 3.5 dB) */
		/* rc = r82xx_write_reg_mask(priv, 0x0c, 0x0b, 0x9f); */
		/* set VGA gain auto */
		rc = r82xx_write_reg_mask(priv, 0x0c, 0x1b, 0x9f);
		if (rc < 0)
			return rc;

		priv->last_AGC_state = new_agc_state;
		if ( rtl_vga_control )
			*rtl_vga_control = 1;
	}

	return 0;
}

int r82xx_set_agc_mode(struct r82xx_priv *priv, int agc_mode, int *rtl_vga_control)
{
	int rc = 0;
	priv->agc_mode = agc_mode;
	*rtl_vga_control = 0;
	if ( priv->last_AGC_state )
		rc = r82xx_set_gain(priv, 0, 0, 0
			, priv->last_LNA_value, priv->last_Mixer_value
			, priv->last_VGA_value
			, rtl_vga_control);
	return rc;
}

/* expose/permit tuner specific i2c register hacking! */
int r82xx_set_i2c_register(struct r82xx_priv *priv, unsigned i2c_register, unsigned data, unsigned mask)
{
	uint8_t reg = i2c_register & 0xFF;
	uint8_t reg_mask = mask & 0xFF;
	uint8_t reg_val = data & 0xFF;
	return r82xx_write_reg_mask(priv, reg, reg_val, reg_mask);
}

int r82xx_set_i2c_override(struct r82xx_priv *priv, unsigned i2c_register, unsigned data, unsigned mask)
{
	uint8_t reg = i2c_register & 0xFF;
	uint8_t reg_mask = mask & 0xFF;
	uint8_t reg_val = data & 0xFF;
	fprintf(stderr, "%s: register %d = %02X. mask %02X, data %03X\n"
			, __FUNCTION__, i2c_register, i2c_register, mask, data );

	if ( REG_SHADOW_START <= reg && reg < REG_SHADOW_START + NUM_REGS ) {
		uint8_t oldMask = priv->override_mask[reg - REG_SHADOW_START];
		uint8_t oldData = priv->override_data[reg - REG_SHADOW_START];
		if ( data & ~0xFF ) {
			priv->override_mask[reg - REG_SHADOW_START] &= ~reg_mask;
			priv->override_data[reg - REG_SHADOW_START] &= ~reg_mask;
			fprintf(stderr, "%s: subtracted override mask for register %02X. old mask %02X, old data %02X. new mask is %02X, new data %02X\n"
					, __FUNCTION__
					, i2c_register
					, oldMask, oldData
					, priv->override_mask[reg - REG_SHADOW_START]
					, priv->override_data[reg - REG_SHADOW_START]
					);
		}
		else
		{
			priv->override_mask[reg - REG_SHADOW_START] |= reg_mask;
			priv->override_data[reg - REG_SHADOW_START] &= (~reg_mask);

			fprintf(stderr, "override_data[] &= ( ~(mask %02X) = %02X ) => %02X\n", reg_mask, ~reg_mask, priv->override_data[reg - REG_SHADOW_START] );
			priv->override_data[reg - REG_SHADOW_START] |= (reg_mask & reg_val);
			fprintf(stderr, "override_data[] |= ( mask %02X & val %02X )\n", reg_mask, reg_val );
			fprintf(stderr, "%s: added override mask for register %d = %02X. old mask %02X, old data %02X. new mask is %02X, new data %02X\n"
					, __FUNCTION__
					, i2c_register, i2c_register
					, oldMask, oldData
					, priv->override_mask[reg - REG_SHADOW_START]
					, priv->override_data[reg - REG_SHADOW_START]
					);
		}
		return r82xx_write_reg_mask_ext(priv, reg, 0, 0, __FUNCTION__);
	}
	else
		return -1;
}


/* Bandwidth contribution by low-pass filter. */
static const int r82xx_if_low_pass_bw_table[] = {
	1700000, 1600000, 1550000, 1450000, 1200000, 900000, 700000, 550000, 450000, 350000
};


struct IFinfo
{
	int	sharpCorner;	/* 1 = at LSB (lower side band); 2 = at USB (upper side band); 3 = both LSB+USB */
	int	bw;				/* 3-dB bandwidth in kHz - available around IF frequency, which becomes center frequency */
	int	fif;			/* IF frequency in kHz for the RTL2832 */
	int	fc;				/* IF frequency correction in kHz */
	uint8_t reg10Lo;	/* low-part of I2C register 0x0A */
	uint8_t	reg11;		/* content of I2C register 0x0B */
	uint8_t	reg30Hi;		/* R30 = 0x1E: channel filter extension "on weak signal" */
};

/* narrowest IF bandpass with reg10/reg11/reg30 = 0x0F, 0xEF, 0x60:
 *   539 .. 2002 kHz (mirrored from tuner)
 * calculate IF center frequency from that bandpass edges
 */
#define IFA(BW)		(2002 - BW / 2)
#define IFB(BW)		(539 + BW / 2)

/* duplicated lower bandwidths to allow "sideband" selection: which is filtered with help of IF corner */
static const struct IFinfo IFi[] = {
#if (WITH_ASYM_FILTER)
	{ 1,  200+1, IFA( 200),  33, 0x0F, 0xEF, 0x60 },	/* steep low  freq edge */
	{ 2,  200+2, IFB( 200),   3, 0x0F, 0xEF, 0x60 },	/* steep high freq edge */
#endif

	{ 3,  290+0,      1950, -25, 0x0F, 0xE7, 0x00 },	/* centered with hpf - high pass filter */
#if (WITH_ASYM_FILTER)
	{ 1,  290+1, IFA( 290),  26, 0x0F, 0xEF, 0x60 },	/* steep low  freq edge */
	{ 2,  290+2, IFB( 290),   2, 0x0F, 0xEF, 0x60 },	/* steep high freq edge */
#endif

	{ 3,  375+0,      1870, -13, 0x0F, 0xE8, 0x00 },	/* centered with hpf */
#if (WITH_ASYM_FILTER)
	{ 1,  375+1, IFA( 375),  23, 0x0F, 0xEF, 0x60 },	/* steep low  freq edge */
	{ 2,  375+2, IFB( 375),   3, 0x0F, 0xEF, 0x60 },	/* steep high freq edge */
#endif

	{ 3,  420+0,      2100,  21, 0x0F, 0xD7, 0x00 },	/* centered with hpf */
#if (WITH_ASYM_FILTER)
	{ 1,  420+1, IFA( 420),  23, 0x0F, 0xEF, 0x60 },	/* steep low  freq edge */
	{ 2,  420+2, IFB( 420),   3, 0x0F, 0xEF, 0x60 },	/* steep high freq edge */
#endif

	{ 3,  470+0,      1800, -12, 0x0F, 0xE9, 0x00 },	/* centered with hpf */
#if (WITH_ASYM_FILTER)
	{ 1,  470+1, IFA( 470),  18, 0x0F, 0xEF, 0x60 },	/* steep low  freq edge */
	{ 2,  470+2, IFB( 470),   2, 0x0F, 0xEF, 0x60 },	/* steep high freq edge */
#endif

	{ 3,  600+0,      1700,   6, 0x0F, 0xEA, 0x00 },	/* centered with hpf */
#if (WITH_ASYM_FILTER)
	{ 1,  600+1, IFA( 600),  16, 0x0F, 0xEF, 0x60 },	/* steep low  freq edge */
	{ 2,  600+2, IFB( 600),   3, 0x0F, 0xEF, 0x60 },	/* steep high freq edge */
#endif

	{ 3,  860+0,      1550,   8, 0x0F, 0xEB, 0x00 },	/* centered with hpf */
#if (WITH_ASYM_FILTER)
	{ 1,  860+1, IFA( 860),  17, 0x0F, 0xEF, 0x60 },	/* steep low  freq edge */
	{ 2,  860+2, IFB( 860), -12, 0x0F, 0xEF, 0x60 },	/* steep high freq edge */
#endif

	{ 3,  950+0,      2200,   5, 0x0F, 0x88, 0x00 },	/* centered with hpf */
#if (WITH_ASYM_FILTER)
	{ 1,  950+1, IFA( 950),   6, 0x0F, 0xEF, 0x60 },	/* steep low  freq edge */
	{ 2,  950+2, IFB( 950),   0, 0x0F, 0xEF, 0x60 },	/* steep high freq edge */
#endif

	{ 3, 1100+0,      2100,  25, 0x0F, 0x89, 0x00 },	/* centered with hpf */
#if (WITH_ASYM_FILTER)
	{ 1, 1100+1, IFA(1100),  24, 0x0F, 0xEF, 0x60 },	/* steep low  freq edge */
	{ 2, 1100+2, IFB(1100),   0, 0x0F, 0xEF, 0x60 },	/* steep high freq edge */
#endif

	{ 3, 1300+0,      2050,  -7, 0x0F, 0x8A, 0x00 },	/* centered with hpf */
#if (WITH_ASYM_FILTER)
	{ 1, 1300+1, IFA(1300),  26, 0x0F, 0xEF, 0x60 },	/* steep low  freq edge */
	{ 2, 1300+2, IFB(1300),   0, 0x0F, 0xEF, 0x60 },	/* steep high freq edge */
#endif

	{ 3, 1500+3,      1300, -24, 0x0F, 0xEF, 0x60 },
	{ 3, 1600+0,      1900,   0, 0x0F, 0x8B, 0x00 },	/* centered with hpf */
	{ 3, 1750+3,      1400,  12, 0x0F, 0xCF, 0x60 },	/* 20 */
	{ 3, 1950+3,      1500,  30, 0x0F, 0x8F, 0x60 }
};

static const int r82xx_bw_tablen = sizeof(IFi) / sizeof(IFi[0]);


#define FILT_HP_BW1 350000
#define FILT_HP_BW2 380000
int r82xx_set_bandwidth(struct r82xx_priv *priv, int bw, uint32_t rate, uint32_t * applied_bw, int apply)
{
	int rc;
	unsigned int i;
	int real_bw = 0;
#if USE_R82XX_ENV_VARS
	uint8_t reg_09, reg_0d =0, reg_0e =0;
#endif
	uint8_t reg_mask;
	uint8_t reg_0a;
	uint8_t reg_0b;
	uint8_t reg_1e = 0x60;		/* default: Enable Filter extension under weak signal */

	if (bw > 7000000) {
		// BW: 8 MHz
		*applied_bw = 8000000;
		reg_0a = 0x10;
		reg_0b = 0x0b;
		if (apply)
			priv->int_freq = 4570000;
	} else if (bw > 6000000) {
		// BW: 7 MHz
		*applied_bw = 7000000;
		reg_0a = 0x10;
		reg_0b = 0x2a;
		if (apply)
			priv->int_freq = 4570000;
	} else if (bw > 4500000) {
		// BW: 6 MHz
		*applied_bw = 6000000;
		reg_0a = 0x10;
		reg_0b = 0x6b;
		if (apply)
			priv->int_freq = 3570000;
	} else {
		for(i=0; i < r82xx_bw_tablen-1; ++i) {
			const int bwnext = IFi[i+1].bw * 1000 + ( IFi[i+1].sharpCorner == 2 ? 400 : 0 );
			const int bwcurr = IFi[i].bw * 1000 + ( IFi[i].sharpCorner == 2 ? 400 : 0 );
			/* bandwidth is compared to median of the current and next available bandwidth in the table */
			if (bw < (bwnext + bwcurr)/2)
				break;
		}

		reg_0a = IFi[i].reg10Lo;
		reg_0b = IFi[i].reg11;
		reg_1e = IFi[i].reg30Hi;
		real_bw = IFi[i].bw * 1000;   /* kHz part */
#if 0
		fprintf(stderr, "%s: selected idx %d: R10 = %02X, R11 = %02X, Bw %d, IF %d\n"
			, __FUNCTION__, i
			, (unsigned)reg_0a
			, (unsigned)reg_0b
			, real_bw
			, IFi[i].fif * 1000 + IFi[i].fc
			);
#endif
		*applied_bw = real_bw;
		if (apply)
			priv->int_freq = ( IFi[i].fif + IFi[i].fc ) * 1000;
	}

#if USE_R82XX_ENV_VARS
	// hacking RTLSDR IF-Center frequency - on environment variable
	if ( priv->filterCenter && apply )
	{
#if 0
		fprintf(stderr, "*** applied IF center %d\n", priv->filterCenter);
#endif
		priv->int_freq = priv->filterCenter;
	}
#endif

	if (!apply)
		return 0;

#if USE_R82XX_ENV_VARS
	/* Register 0x9 = R9: IF Filter Power/Current */
	if ( priv->haveR9 )
	{
		fprintf(stderr, "*** in function %s: PREV I2C register %02X value: %02X\n", __FUNCTION__, 0x09, r82xx_read_cache_reg(priv, 0x09) );
		reg_09 = priv->valR9 << 6;
		reg_mask = 0xC0;	// 1100 0000
		fprintf(stderr, "*** read R9 from environment: %d\n", priv->valR9);
		rc = r82xx_write_reg_mask(priv, 0x09, reg_09, 0xc0);
		if (rc < 0)
			fprintf(stderr, "ERROR setting I2C register 0x09 to value %02X with mask %02X\n", (unsigned)reg_09, (unsigned)reg_mask);
	}
#endif

	/* Register 0xA = R10 */
	reg_mask = 0x0F;	// default: 0001 0000
#if USE_R82XX_ENV_VARS
	if ( priv->haveR10H ) {
		reg_0a = ( reg_0a & ~0xf0 ) | ( priv->valR10H << 4 );
		reg_mask = 0xf0;
	}
	if ( priv->haveR10L ) {
		reg_0a = ( reg_0a & ~0x0f ) | priv->valR10L;
		reg_mask |= 0x0f;
	}
#endif
	rc = r82xx_write_reg_mask_ext(priv, 0x0a, reg_0a, reg_mask, __FUNCTION__);
	if (rc < 0) {
		fprintf(stderr, "ERROR setting I2C register 0x0A to value %02X with mask %02X\n", (unsigned)reg_0a, (unsigned)reg_mask);
		return rc;
	}

	/* Register 0xB = R11 with undocumented Bit 7 for filter bandwidth for Hi-part FILT_BW */
	reg_mask = 0xEF;	// default: 1110 1111
#if USE_R82XX_ENV_VARS
	if ( priv->haveR11H ) {
		reg_0b = ( reg_0b & ~0xE0 ) | ( priv->valR11H << 5 );
	}
	if ( priv->haveR11L ) {
		reg_0b = ( reg_0b & ~0x0F ) | priv->valR11L;
	}
#endif
	rc = r82xx_write_reg_mask_ext(priv, 0x0b, reg_0b, reg_mask, __FUNCTION__);
	if (rc < 0) {
		fprintf(stderr, "ERROR setting I2C register 0x0B to value %02X with mask %02X\n"
			, (unsigned)reg_0b, (unsigned)reg_mask);
		return rc;
	}


	/* Register 0xD = R13: LNA agc power detector voltage threshold high + low setting */
	reg_mask = 0x00;	// default: 0000 0000
#if USE_R82XX_ENV_VARS
	if ( priv->haveR13H ) {
		reg_0d = ( reg_0d & ~0xf0 ) | ( priv->valR13H << 4 );
		reg_mask |= 0xF0;
	}
	if ( priv->haveR13L ) {
		reg_0d = ( reg_0d & ~0x0f ) | priv->valR13L;
		reg_mask |= 0x0F;
	}
	if ( reg_mask ) {
		rc = r82xx_write_reg_mask_ext(priv, 0x0d, reg_0d, reg_mask, __FUNCTION__);
		if (rc < 0)
			fprintf(stderr, "ERROR setting I2C register 0x0D to value %02X with mask %02X\n"
				, (unsigned)reg_0d, (unsigned)reg_mask);
	}
#endif

	/* Register 0xE = R14: MIXER agc power detector voltage threshold high + low setting */
	reg_mask = 0x00;	// default: 0000 0000
#if USE_R82XX_ENV_VARS
	if ( priv->haveR14H ) {
		reg_0e = ( reg_0e & ~0xf0 ) | ( priv->valR14H << 4 );
		reg_mask |= 0xF0;
	}
	if ( priv->haveR14L ) {
		reg_0e = ( reg_0e & ~0x0f ) | priv->valR14L;
		reg_mask |= 0x0F;
	}
	if ( reg_mask ) {
		rc = r82xx_write_reg_mask_ext(priv, 0x0e, reg_0e, reg_mask, __FUNCTION__);
		if (rc < 0)
			fprintf(stderr, "%s: ERROR setting I2C register 0x0E to value %02X with mask %02X\n"
			, __FUNCTION__, (unsigned)reg_0e, (unsigned)reg_mask);
	}
#endif

	/* channel filter extension */
	reg_mask = 0x60;
#if USE_R82XX_ENV_VARS
	if ( priv->haveR30H ) {
		reg_1e = ( priv->valR30H << 4 );
	}
	if ( priv->haveR30L ) {
		reg_1e = reg_1e | priv->valR30L;
		reg_mask = reg_mask | 0x1F;
	}
#endif
	rc = r82xx_write_reg_mask_ext(priv, 0x1e, reg_1e, reg_mask, __FUNCTION__);
	if (rc < 0)
		fprintf(stderr, "%s: ERROR setting I2C register 0x1E to value %02X with mask %02X\n"
		, __FUNCTION__, (unsigned)reg_1e, (unsigned)reg_mask);

	return priv->int_freq;
}
#undef FILT_HP_BW1
#undef FILT_HP_BW2

int r82xx_set_bw_center(struct r82xx_priv *priv, int32_t if_band_center_freq)
{
	priv->if_band_center_freq = if_band_center_freq;
	return priv->int_freq;
}

int r82xx_set_freq(struct r82xx_priv *priv, uint32_t freq)
{
	int rc = -1;
	uint32_t lo_freq = freq + priv->int_freq + priv->if_band_center_freq;
#if 0
	fprintf(stderr, "%s(freq = %u) --> intfreq %u, ifcenter %d --> f %u\n"
			, __FUNCTION__, (unsigned)freq
			, (unsigned)priv->int_freq, (int)priv->if_band_center_freq
			, (unsigned)lo_freq );
#endif
	uint8_t air_cable1_in;

	rc = r82xx_set_mux(priv, lo_freq);
	if (rc < 0)
		goto err;

	rc = r82xx_set_pll(priv, lo_freq);
	if (rc < 0 || !priv->has_lock)
		goto err;

	/* switch between 'Cable1' and 'Air-In' inputs on sticks with
	 * R828D tuner. We switch at 345 MHz, because that's where the
	 * noise-floor has about the same level with identical LNA
	 * settings. The original driver used 320 MHz. */
	air_cable1_in = (freq > MHZ(345)) ? 0x00 : 0x60;

	if ((priv->cfg->rafael_chip == CHIP_R828D) &&
		(air_cable1_in != priv->input)) {
		priv->input = air_cable1_in;
		rc = r82xx_write_reg_mask(priv, 0x05, air_cable1_in, 0x60);
	}

err:
	if (rc < 0)
		fprintf(stderr, "%s: failed=%d\n", __FUNCTION__, rc);
	return rc;
}

/*
 * r82xx standby logic
 */

int r82xx_standby(struct r82xx_priv *priv)
{
	int rc;

	/* If device was not initialized yet, don't need to standby */
	if (!priv->init_done)
		return 0;

	rc = r82xx_write_reg(priv, 0x06, 0xb1);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg(priv, 0x05, 0xa0);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg(priv, 0x07, 0x3a);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg(priv, 0x08, 0x40);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg(priv, 0x09, 0xc0);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg(priv, 0x0a, 0x36);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg(priv, 0x0c, 0x35);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg(priv, 0x0f, 0x68);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg(priv, 0x11, 0x03);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg(priv, 0x17, 0xf4);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg(priv, 0x19, 0x0c);

	/* Force initial calibration */
	priv->type = -1;

	return rc;
}

/*
 * r82xx device init logic
 */

static int r82xx_xtal_check(struct r82xx_priv *priv)
{
	int rc;
	unsigned int i;
	uint8_t data[3], val;

	/* Initialize the shadow registers */
	memcpy(priv->regs, r82xx_init_array, sizeof(r82xx_init_array));

	/* cap 30pF & Drive Low */
	rc = r82xx_write_reg_mask(priv, 0x10, 0x0b, 0x0b);
	if (rc < 0)
		return rc;

	/* set pll autotune = 128kHz */
	rc = r82xx_write_reg_mask(priv, 0x1a, 0x00, 0x0c);
	if (rc < 0)
		return rc;

	/* set manual initial reg = 111111;  */
	rc = r82xx_write_reg_mask(priv, 0x13, 0x7f, 0x7f);
	if (rc < 0)
		return rc;

	/* set auto */
	rc = r82xx_write_reg_mask(priv, 0x13, 0x00, 0x40);
	if (rc < 0)
		return rc;

	/* Try several xtal capacitor alternatives */
	for (i = 0; i < ARRAY_SIZE(r82xx_xtal_capacitor); i++) {
		rc = r82xx_write_reg_mask(priv, 0x10,
					  r82xx_xtal_capacitor[i][0], 0x1b);
		if (rc < 0)
			return rc;

//		usleep_range(5000, 6000);

		rc = r82xx_read(priv, 0x00, data, sizeof(data));
		if (rc < 0)
			return rc;
		if (!(data[2] & 0x40))
			continue;

		val = data[2] & 0x3f;

		if (priv->cfg->xtal == 16000000 && (val > 29 || val < 23))
			break;

		if (val != 0x3f)
			break;
	}

	if (i == ARRAY_SIZE(r82xx_xtal_capacitor))
		return -1;

	return r82xx_xtal_capacitor[i][1];
}

int r82xx_init(struct r82xx_priv *priv)
{
	int rc;

	/* TODO: R828D might need r82xx_xtal_check() */
	priv->xtal_cap_sel = XTAL_HIGH_CAP_0P;

	priv->if_band_center_freq = 0;

	priv->agc_mode = 0;
	priv->last_AGC_state = 0;
	priv->last_LNA_value = 0;
	priv->last_Mixer_value = 0;

	/* Initialize override registers */
	memset( &(priv->override_data[0]), 0, NUM_REGS * sizeof(uint8_t) );
	memset( &(priv->override_mask[0]), 0, NUM_REGS * sizeof(uint8_t) );

	/* Initialize registers */
	rc = r82xx_write_arr(priv, 0x05,
			 r82xx_init_array, sizeof(r82xx_init_array));

	rc = r82xx_set_tv_standard(priv, 3, TUNER_DIGITAL_TV, 0);
	if (rc < 0)
		goto err;

	rc = r82xx_sysfreq_sel(priv, 0, TUNER_DIGITAL_TV, SYS_DVBT);

#if USE_R82XX_ENV_VARS
	priv->printI2C = 0;
	priv->filterCenter = 0;
	priv->haveR9 = priv->valR9 = 0;
	priv->haveR10L = priv->valR10L = 0;
	priv->haveR10H = priv->valR10H = 0;
	priv->haveR11L = priv->valR11L = 0;
	priv->haveR11H = priv->valR11H = 0;
	priv->haveR13L = priv->valR13L = 0;
	priv->haveR13H = priv->valR13H = 0;
	priv->haveR14L = priv->valR14L = 0;
	priv->haveR14H = priv->valR14H = 0;
	priv->haveR30H = priv->valR30H = 0;
	priv->haveR30L = priv->valR30L = 0;
#endif

	priv->init_done = 1;

#if USE_R82XX_ENV_VARS
	// read environment variables
	if (1) {
		char *pacPrintI2C;
		char *pacFilterCenter, *pacR9;
		char *pacR10Hi, *pacR10Lo, *pacR11Hi, *pacR11Lo;
		char *pacR13Hi, *pacR13Lo, *pacR14Hi, *pacR14Lo;
		char *pacR30Hi, *pacR30Lo;

		pacPrintI2C = getenv("RTL_R820_PRINT_I2C");
		if ( pacPrintI2C )
			priv->printI2C = atoi(pacPrintI2C);

		pacFilterCenter = getenv("RTL_R820_IF_CENTER");
		if ( pacFilterCenter )
			priv->filterCenter = atoi(pacFilterCenter);

		pacR9 = getenv("RTL_R820_R9_76");
		if ( pacR9 ) {
			priv->haveR9 = 1;
			priv->valR9 = atoi(pacR9);
			if ( priv->valR9 > 3 ) {
				fprintf(stderr, "*** read R9 from environment: %d - but value should be 0 - 3 for bit [7:6]\n", priv->valR9);
				priv->haveR9 = 0;
			}
			fprintf(stderr, "*** read R9 from environment: %d\n", priv->valR9);
		}

		pacR10Hi = getenv("RTL_R820_R10_HI");
		if ( pacR10Hi ) {
			priv->haveR10H = 1;
			priv->valR10H = atoi(pacR10Hi);
			if ( priv->valR10H > 15 ) {
				fprintf(stderr, "*** read R10_HI from environment: %d - but value should be 0 - 15 for bit [7:4]\n", priv->valR10H);
				priv->haveR10H = 0;
			}
			fprintf(stderr, "*** read R10_HI from environment: %d\n", priv->valR10H);
		}

		pacR10Lo = getenv("RTL_R820_R10_LO");
		if ( pacR10Lo ) {
			priv->haveR10L = 1;
			priv->valR10L = atoi(pacR10Lo);
			if ( priv->valR10L > 15 ) {
				fprintf(stderr, "*** read R10_LO from environment: %d - but value should be 0 - 15 for bit [3:0]\n", priv->valR10L);
				priv->haveR10L = 0;
			}
			fprintf(stderr, "*** read R10_LO from environment: %d\n", priv->valR10L);
		}

		pacR11Hi = getenv("RTL_R820_R11_HI");
		if ( pacR11Hi ) {
			priv->haveR11H = 1;
			priv->valR11H = atoi(pacR11Hi);
			if ( priv->valR11H > 7 ) {
				fprintf(stderr, "*** read R11_HI from environment: %d - but value should be 0 - 7 for bit [6:5]\n", priv->valR11H);
				priv->haveR11H = 0;
			}
			fprintf(stderr, "*** read R11_HI from environment: %d\n", priv->valR11H);
		}

		pacR11Lo = getenv("RTL_R820_R11_LO");
		if ( pacR11Lo ) {
			priv->haveR11L = 1;
			priv->valR11L = atoi(pacR11Lo);
			if ( priv->valR11L > 15 ) {
				fprintf(stderr, "*** read R11_LO from environment: %d - but value should be 0 - 15 for bit [3:0]\n", priv->valR11L);
				priv->haveR11L = 0;
			}
			fprintf(stderr, "*** read R11_LO from environment: %d\n", priv->valR11L);
		}


		pacR13Hi = getenv("RTL_R820_R13_HI");
		if ( pacR13Hi ) {
			priv->haveR13H = 1;
			priv->valR13H = atoi(pacR13Hi);
			if ( priv->valR13H > 15 ) {
				fprintf(stderr, "*** read R13_HI from environment: %d - but value should be 0 - 15 for bit [7:4]\n", priv->valR13H);
				priv->haveR13H = 0;
			}
			fprintf(stderr, "*** read R13_HI from environment: %d\n", priv->valR13H);
		}

		pacR13Lo = getenv("RTL_R820_R13_LO");
		if ( pacR13Lo ) {
			priv->haveR13L = 1;
			priv->valR13L = atoi(pacR13Lo);
			if ( priv->valR13L > 15 ) {
				fprintf(stderr, "*** read R13_LO from environment: %d - but value should be 0 - 15 for bit [3:0]\n", priv->valR13L);
				priv->haveR13L = 0;
			}
			fprintf(stderr, "*** read R13_LO from environment: %d\n", priv->valR13L);
		}


		pacR14Hi = getenv("RTL_R820_R14_HI");
		if ( pacR14Hi ) {
			priv->haveR14H = 1;
			priv->valR14H = atoi(pacR14Hi);
			if ( priv->valR14H > 15 ) {
				fprintf(stderr, "*** read R14_HI from environment: %d - but value should be 0 - 15 for bit [7:4]\n", priv->valR14H);
				priv->haveR14H = 0;
			}
			fprintf(stderr, "*** read R14_HI from environment: %d\n", priv->valR14H);
		}

		pacR14Lo = getenv("RTL_R820_R14_LO");
		if ( pacR14Lo ) {
			priv->haveR14L = 1;
			priv->valR14L = atoi(pacR14Lo);
			if ( priv->valR14L > 15 ) {
				fprintf(stderr, "*** read R14_LO from environment: %d - but value should be 0 - 15 for bits [3:0]\n", priv->valR14L);
				priv->haveR14L = 0;
			}
			fprintf(stderr, "*** read R14_LO from environment: %d\n", priv->valR14L);
		}

		pacR30Hi = getenv("RTL_R820_R30_HI");
		if ( pacR30Hi ) {
			priv->haveR30H = 1;
			priv->valR30H = atoi(pacR30Hi) & 0x06;
			if ( priv->valR30H > 6 ) {
				fprintf(stderr, "*** read R30_HI from environment: %d - but value should be 2 - 6 for bit [6:5]\n", priv->valR30H);
				priv->haveR30H = 0;
			}
			fprintf(stderr, "*** read R30_HI from environment: %d\n", priv->valR30H);
		}

		pacR30Lo = getenv("RTL_R820_R30_LO");
		if ( pacR30Lo ) {
			priv->haveR30L = 1;
			priv->valR30L = atoi(pacR30Lo);
			if ( priv->valR30L > 31 ) {
				fprintf(stderr, "*** read R30_LO from environment: %d - but value should be 0 - 31 for bit [4:0]\n", priv->valR30L);
				priv->haveR30L = 0;
			}
			fprintf(stderr, "*** read R30_LO from environment: %d\n", priv->valR30L);
		}

	}
#endif

err:
	if (rc < 0)
		fprintf(stderr, "%s: failed=%d\n", __FUNCTION__, rc);
	return rc;
}

#if 0
/* Not used, for now */
static int r82xx_gpio(struct r82xx_priv *priv, int enable)
{
	return r82xx_write_reg_mask(priv, 0x0f, enable ? 1 : 0, 0x01);
}
#endif
