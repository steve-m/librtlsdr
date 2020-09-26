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

#define WITH_ASYM_FILTER	0
#define PRINT_PLL_ERRORS	0
#define PRINT_VGA_REG		0
#define PRINT_INITIAL_REGISTERS		0
#define PRINT_ACTUAL_VCO_AND_ERR	0

/* use fifth harmonic above this frequency in kHz, when PLL does NOT lock */
#define FIFTH_HARM_FRQ_THRESH_KHZ	1770000
#define RETRY_WITH_FIFTH_HARM_KHZ	1760000
#define DEFAULT_HARMONIC			5
#define PRINT_HARMONICS				0


/* #define VGA_FOR_AGC_MODE	16 */
#define DEFAULT_IF_VGA_VAL	11


#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define MHZ(x)		((x)*1000*1000)
#define KHZ(x)		((x)*1000)

/*
Register Description
many updates from https://github.com/old-dab/rtlsdr

Reg		Bitmap	Symbol			Description
------------------------------------------------------------------------------------
R0		[7:0]	CHIP_ID			reference check point for read mode: 0x96
0x00
------------------------------------------------------------------------------------
R1		[7:6]					10
0x01	[5:0]	ADC				Analog-Digital Converter for detector 3
------------------------------------------------------------------------------------
R2		[7]						1
0x02	[6]		VCO_INDICATOR	0: PLL has not locked, 1: PLL has locked
		[5:0]					Analog-Digital Converter for VCO
	 							000000: min (1.75 GHz), 111111: max (3.6 GHz)
------------------------------------------------------------------------------------
R3		[7:4]	RF_INDICATOR	Mixer gain
0x03							0: Lowest, 15: Highest
		[3:0]					LNA gain
								0: Lowest, 15: Highest
------------------------------------------------------------------------------------
R4		[5:4]					vco_fine_tune
0x04	[3:0]					fil_cal_code
------------------------------------------------------------------------------------
R5		[7] 	LOOP_THROUGH	Loop through ON/OFF
0x05							0: on, 1: off
		[6:5]	AIR_CABLE1_IN	0 (only R828D)
		[5] 	PWD_LNA1		LNA 1 power control
								0:on, 1:off
		[4] 	LNA_GAIN_MODE	LNA gain mode switch
								0: auto, 1: manual
		[3:0] 	LNA_GAIN		LNA manual gain control
								15: max gain, 0: min gain
------------------------------------------------------------------------------------
R6		[7] 	PWD_PDET1		Power detector 1 on/off
0x06							0: on, 1: off
		[6] 	PWD_PDET3		Power detector 3 on/off
								0: off, 1: on
		[5] 	FILT_GAIN		Filter gain 3db
								0:0db, 1:+3db
		[4]						1
		[3]		CABLE2_IN		0 (only R828D)
		[2:0]	PW_LNA			LNA power control
								000: max, 111: min
------------------------------------------------------------------------------------
R7		[7]		IMG_R			Mixer Sideband
0x07							0: lower, 1: upper
		[6] 	PWD_MIX			Mixer power
								0:off, 1:on
		[5] 	PW0_MIX			Mixer current control
								0:max current, 1:normal current
		[4] 	MIXGAIN_MODE	Mixer gain mode
								0:manual mode, 1:auto mode
		[3:0] 	MIX_GAIN		Mixer manual gain control
								0000->min, 1111->max
------------------------------------------------------------------------------------
R8		[7] 	PWD_AMP			Mixer buffer power on/off
0x08							0: off, 1:on
		[6] 	PW0_AMP			Mixer buffer current setting
								0: high current, 1: low current
		[5]						0: Q, 1: I
		[4:0] 	IMR_G			Image Gain Adjustment
								0: min, 31: max
------------------------------------------------------------------------------------
R9		[7] 	PWD_IFFILT		IF Filter power on/off
0x09							0: filter on, 1: off
		[6] 	PW1_IFFILT		IF Filter current
								0: high current, 1: low current
		[5]						0: Q, 1: I
		[4:0] 	IMR_P			Image Phase Adjustment
								0: min, 31: max
------------------------------------------------------------------------------------
R10		[7] 	PWD_FILT		Filter power on/off
0x0A							0: channel filter off, 1: on
		[6:5] 	PW_FILT			Filter power control
								00: highest power, 11: lowest power
		[4]		FILT_Q			1
		[3:0] 	FILT_CODE		Filter bandwidth manual fine tune
								0000 Widest, 1111 narrowest
------------------------------------------------------------------------------------
R11		[7:5] 	FILT_BW			Filter bandwidth manual course tunnel
0x0B							000: widest
								010 or 001: middle
								111: narrowest
		[4]		CAL_TRIGGER		0
		[3:0] 	HP_COR			High pass filter corner control
								0000: highest
								1111: lowest
------------------------------------------------------------------------------------
R12		[7]		SW_ADC			Switch Analog-Digital Converter for detector 3 (see R1)
								0: on, 1: off
0x0C	[6] 	PWD_VGA			VGA power control
								0: vga power off, 1: vga power on
		[5]						1
		[4] 	VGA_MODE		VGA GAIN manual / pin selector
								1: IF vga gain controlled by vagc pin
								0: IF vga gain controlled by vga_code[3:0]
		[3:0] 	VGA_CODE		IF vga manual gain control
								0000: -12.0 dB
								1111: +40.5 dB; -3.5dB/step
------------------------------------------------------------------------------------
R13		[7:4]	LNA_VTHH		LNA agc power detector voltage threshold high setting
0x0D							1111: 1.94 V
								0000: 0.34 V, ~0.1 V/step
		[3:0] 	LNA_VTHL		LNA agc power detector voltage threshold low setting
								1111: 1.94 V
								0000: 0.34 V, ~0.1 V/step
------------------------------------------------------------------------------------
R14 	[7:4] 	MIX_VTH_H		MIXER agc power detector voltage threshold high setting
0x0E							1111: 1.94 V
								0000: 0.34 V, ~0.1 V/step
		[3:0] 	MIX_VTH_L		MIXER agc power detector voltage threshold low setting
								1111: 1.94 V
								0000: 0.34 V, ~0.1 V/step
------------------------------------------------------------------------------------
R15		[7]		FLT_EXT_WIDEST	filter extension widest
0x0F							0: off, 1: on
		[4] 	CLK_OUT_ENB		Clock out pin control
								0: clk output on, 1: off
		[3]						ring clk
								1: off, 0: on
		[2]						set cali clk
								0: off, 1: on
		[1] 	CLK_AGC_ENB		AGC clk control
								0: internal agc clock on, 1: off
		[0]		GPIO			0
------------------------------------------------------------------------------------
R16		[7:5] 	SEL_DIV			PLL to Mixer divider number control
0x10							000: mixer in = vco out / 2
								001: mixer in = vco out / 4
								010: mixer in = vco out / 8
								011: mixer in = vco out / 16
								100: mixer in = vco out / 32
								101: mixer in = vco out / 64
		[4] 	REFDIV			PLL Reference frequency Divider
								0 -> fref=xtal_freq
								1 -> fref=xta_freql / 2 (for Xtal >24MHz)
		[3]						X'tal Drive
								0: High, 1: Low
		[2]						1
		[1:0] 	CAPX			Internal xtal cap setting
								00->no cap
								01->10pF
								10->20pF
								11->30pF
------------------------------------------------------------------------------------
R17		[7:6] 	PW_LDO_A		PLL analog low drop out regulator switch
0x11							00: off
								01: 2.1V
								10: 2.0V
								11: 1.9V
		[5:3]	CP_CUR			cp_cur
								101: 0.2, 111: auto
		[2:0]					011
------------------------------------------------------------------------------------
R18		[7:5] 					set VCO current
0x12	[4]						0: enable dithering, 1: disable dithering
		[3]		PW_SDM			0: Enable frac pll, 1: Disable frac pll
		[2:0]					000
------------------------------------------------------------------------------------
R19		[7]						0
0x13	[6]						VCO control mode
								0: auto mode, VCO controlled by PLL
								1: manual mode, VCO controlled by DAC code[5:0]
		[5:0]	VCO_DAC			DAC for VCO
	 							000000: min (1.75 GHz), 111111: max (3.6 GHz)
------------------------------------------------------------------------------------
R20		[7:6] 	SI2C			PLL integer divider number input Si2c
0x14							Nint=4*Ni2c+Si2c+13
								PLL divider number Ndiv = (Nint + Nfra)*2
		[5:0] 	NI2C			PLL integer divider number input Ni2c
------------------------------------------------------------------------------------
R21		[7:0] 	SDM_IN[8:1]		PLL fractional divider number input SDM[16:1]
0x15							Nfra=SDM_IN[16]*2^-1+SDM_IN[15]*2^-2+...
R22		[7:0] 	SDM_IN[16:9]	+SDM_IN[2]*2^-15+SDM_IN[1]*2^-16
0x16
------------------------------------------------------------------------------------
R23		[7:6] 	PW_LDO_D		PLL digital low drop out regulator supply current switch
0x17							00: 1.8V,8mA
								01: 1.8V,4mA
								10: 2.0V,8mA
								11: OFF
		[5:4]	DIV_BUF_CUR		div_buf_cur
								10: 200u, 11: 150u
		[3] 	OPEN_D			Open drain
								0: High-Z, 1: Low-Z
		[2:0]					100
------------------------------------------------------------------------------------
R24		[7:6]					01
		[5]		ring_div[0]		ring_div bit 0, see ring_div[2:1] in R25
0x18	[4] 					ring power
								0: off, 1:on
		[3:0]					n_ring
								ring_vco = (16+n_ring)*8*pll_ref, n_ring = 9...14
------------------------------------------------------------------------------------
R25		[7] 	PWD_RFFILT		RF Filter power
0x19							0: off, 1:on
		[6:5]	POLYFIL_CUR		RF poly filter current
								00: min
		[4] 	SW_AGC			Switch agc_pin
								0:agc=agc_in
								1:agc=agc_in2
		[3:2]					11
		[1:0]	ring_div[2:1]	cal_freq = ring_vco / divisor; see ring_div[0] in R24
								000: ring_freq = ring_vco / 4
								001: ring_freq = ring_vco / 6
								010: ring_freq = ring_vco / 8
								011: ring_freq = ring_vco / 12
								100: ring_freq = ring_vco / 16
								101: ring_freq = ring_vco / 24
								110: ring_freq = ring_vco / 32
								111: ring_freq = ring_vco / 48
------------------------------------------------------------------------------------
R26		[7:6] 	RF_MUX_POLY		Tracking Filter switch
0x1A							00: TF on
								01: Bypass
		[5:4]					AGC clk
								00: 300ms, 01: 300ms, 10: 80ms, 11: 20ms
		[3:2]	PLL_AUTO_CLK	PLL auto tune clock rate
								00: 128 kHz
								01: 32 kHz
								10: 8 kHz
		[1:0]	RFFILT			RF FILTER band selection
								00: highest band
								01: med band
								10: low band
------------------------------------------------------------------------------------
R27		[7:4]	TF_NCH			0000 highest corner for LPNF
0x1B							1111 lowest corner for LPNF
		[3:0]	TF_LP			0000 highest corner for LPF
								1111 lowest corner for LPF
------------------------------------------------------------------------------------
R28		[7:4]	MIXER_TOP		Power detector 3 (Mixer) TOP(take off point) control
0x1C							0: Highest, 15: Lowest
		[3]						discharge mode
								0: on
		[2]						1
		[1]						1: from ring = ring pll in
		[0]						0
------------------------------------------------------------------------------------
R29		[7:6]					11
0x1D	[5:3]	LNA_TOP			Power detector 1 (LNA) TOP(take off point) control
								0: Highest, 7: Lowest
		[2:0] 	PDET2_GAIN		Power detector 2 TOP(take off point) control
								0: Highest, 7: Lowest
------------------------------------------------------------------------------------
R30		[7]						sw_pdect
0x1E							1: sw_pdect = det3
	 	[6]		FILTER_EXT		Filter extension under weak signal
								0: Disable, 1: Enable
		[5:0]	PDET_CLK		Power detector timing control (LNA discharge current)
	 							111111: max, 000000: min
------------------------------------------------------------------------------------
R31		[7]		LT_ATT			Loop through attenuation
0x1F							0: Enable, 1: Disable
		[6:2]					10000
		[1:0]					pw_ring
								0: -5dB, 1: 0dB, 2: -8dB, 3: -3dB
------------------------------------------------------------------------------------
R0...R4 read, R5...R15 read/write, R16..R31 write
*/


/*
 * Static constants
 */

/* Those initial values start from REG_SHADOW_START */
static const uint8_t r82xx_init_array[] = {
	0x80,	/* Reg 0x05 */
	0x13,	/* Reg 0x06 */
	0x70,	/* Reg 0x07 */

	0xc0,	/* Reg 0x08 */
	0x40,	/* Reg 0x09 */
	0xdb,	/* Reg 0x0a */
	0x6b,	/* Reg 0x0b */

	/* Reg 0x0c:
	 * for manual gain was: set fixed VGA gain for now (16.3 dB): 0x08
	 * with active agc was: set fixed VGA gain for now (26.5 dB): 0x0b */
	0xe0 | DEFAULT_IF_VGA_VAL, /* Reg 0x0c */
	0x53,	/* Reg 0x0d */
	0x75,	/* Reg 0x0e */
	0x68,	/* Reg 0x0f */

	0x6c,	/* Reg 0x10 */
	0xbb,	/* Reg 0x11 */
	0x80,	/* Reg 0x12 */
	VER_NUM & 0x3f,	/* Reg 0x13 */

	0x0f,	/* Reg 0x14 */
	0x00,	/* Reg 0x15 */
	0xc0,	/* Reg 0x16 */
	0x30,	/* Reg 0x17 */

	0x48,	/* Reg 0x18 */
	0xec,	/* Reg 0x19 */
	0x60,	/* Reg 0x1a */
	0x00,	/* Reg 0x1b */

	0x24,	/* Reg 0x1c */
	0xdd,	/* Reg 0x1d */
	0x0e,	/* Reg 0x1e */
	0x40	/* Reg 0x1f */
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

static void print_registers(struct r82xx_priv *priv)
{
	uint8_t data[5];
	int rc;
	unsigned int i;

	rc = r82xx_read(priv, 0x00, data, sizeof(data));
	if (rc < 0)
		return;
	for(i=0; i<sizeof(data); i++)
		printf("%02x ", data[i]);
	printf("\n");
	for(i=sizeof(data); i<32; i++)
		printf("%02x ", r82xx_read_cache_reg(priv, i));
	printf("\n");
}

/*
 * r82xx tuning logic
 */

static int r82xx_set_mux(struct r82xx_priv *priv, uint64_t freq)
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

	return rc;
}


/* function of Youssef (AirSpy) and Carl (RTL-SDR) */
static int r82xx_set_pll_yc(struct r82xx_priv *priv, uint32_t freq)
{
  const uint32_t vco_min = 1770000000;
  const uint32_t vco_max = 3900000000U;
  uint32_t pll_ref = (priv->cfg->xtal);
  uint32_t pll_ref_2x = (pll_ref * 2);

  int rc;
  uint32_t vco_exact;
  uint32_t vco_frac;
  uint32_t con_frac;
  uint32_t div_num;
  uint32_t n_sdm;
  uint16_t sdm;
  uint8_t ni;
  uint8_t si;
  uint8_t nint;
  uint8_t val_dith;
  uint8_t data[5];

  /* Calculate divider */
  for (div_num = 0; div_num < 5; div_num++)
  {
    vco_exact = freq << (div_num + 1);
    if (vco_exact >= vco_min && vco_exact <= vco_max)
    {
      break;
    }
  }

  vco_exact = freq << (div_num + 1);
  nint = (uint8_t) ((vco_exact + (pll_ref >> 16)) / pll_ref_2x);
  vco_frac = vco_exact - pll_ref_2x * nint;

  nint -= 13;
  ni = (nint >> 2);
  si = nint - (ni << 2);

  /* Set the phase splitter */
  rc = r82xx_write_reg_mask(priv, 0x10, (uint8_t) (div_num << 5), 0xe0);
  if(rc < 0) {
    if (priv->cfg->verbose)
      fprintf(stderr, "r82xx_set_pll_yc(): error writing 'phase splitter' into i2c reg 0x10\n");
    return rc;
  }

  /* Disable Dither */
  val_dith = (priv->disable_dither) ? 0x10 : 0x00;
  rc = r82xx_write_reg_mask(priv, 0x12, val_dith, 0x18);
  if (rc < 0) {
    if (priv->cfg->verbose)
      fprintf(stderr, "r82xx_set_pll_yc(): error writing 'dither' into i2c reg 0x12\n");
    return rc;
  }

  /* Set the rough VCO frequency */
  rc = r82xx_write_reg(priv, 0x14, (uint8_t) (ni + (si << 6)));
  if(rc < 0) {
    if (priv->cfg->verbose)
      fprintf(stderr, "r82xx_set_pll_yc(): error writing 'rough VCO frequency' into i2c reg 0x14\n");
    return rc;
  }

  if (vco_frac == 0) {
    /* Disable frac pll */
    rc = r82xx_write_reg_mask(priv, 0x12, 0x08, 0x08);
    if(rc < 0) {
      if (priv->cfg->verbose)
        fprintf(stderr, "r82xx_set_pll_yc(): error writing 'disable frac pll' into i2c reg 0x12\n");
      return rc;
    }
  }
  else
  {
    vco_frac += pll_ref >> 16;
    sdm = 0;
    for(n_sdm = 0; n_sdm < 16; n_sdm++)
    {
        con_frac = pll_ref >> n_sdm;
        if (vco_frac >= con_frac)
        {
            sdm |= (uint16_t) (0x8000 >> n_sdm);
            vco_frac -= con_frac;
            if (vco_frac == 0)
                break;
        }
    }

/*
    actual_freq = (((nint << 16) + sdm) * (uint64_t) pll_ref_2x) >> (div_num + 1 + 16);
    delta = freq - actual_freq
    if (actual_freq != freq)
    {
      fprintf(stderr,"Tunning delta: %d Hz", delta);
    }
*/
    rc = r82xx_write_reg(priv, 0x15, (uint8_t)(sdm & 0xff));
    if (rc < 0) {
      if (priv->cfg->verbose)
        fprintf(stderr, "r82xx_set_pll_yc(): error writing 'sdm lo' into i2c reg 0x15\n");
      return rc;
    }

    rc = r82xx_write_reg(priv, 0x16, (uint8_t)(sdm >> 8));
    if (rc < 0) {
      if (priv->cfg->verbose)
        fprintf(stderr, "r82xx_set_pll_yc(): error writing 'sdm hi' into i2c reg 0x16\n");
      return rc;
    }

    /* Enable frac pll */
    rc = r82xx_write_reg_mask(priv, 0x12, 0x00, 0x08);
    if (rc < 0) {
      if (priv->cfg->verbose)
        fprintf(stderr, "r82xx_set_pll_yc(): error writing 'enable frac pll' into i2c reg 0x12\n");
      return rc;
    }
  }

  /* all PLL stuff / registers set for this frequency */
  priv->tuner_pll_set = 1;

/***/

  /* Check if PLL has locked */
  rc = r82xx_read(priv, 0x00, data, 3);
  if (rc < 0) {
      if (priv->cfg->verbose)
        fprintf(stderr, "r82xx_set_pll_yc(): error reading 'pll lock status' from i2c reg 0x00..0x02\n");
    return rc;
  }
  if (!(data[2] & 0x40)) {
    if (priv->cfg->verbose || PRINT_PLL_ERRORS)
      //fprintf(stderr, "r82xx_set_pll_yc(): error writing 'sdm lo' into i2c reg 0x15\n");
      fprintf(stderr, "[R82XX] PLL not locked at Tuner LO %u Hz for RF %f MHz!\n",
        freq, priv->rf_freq * 1E-6);
    priv->has_lock = 0;
    return -1;
  }
  priv->has_lock = 1;

  return rc;

}


static int r82xx_set_pll(struct r82xx_priv *priv, uint32_t freq)
{
	/* freq == tuner's LO frequency */
	int rc, i;
	uint64_t vco_freq;
	uint64_t vco_div;
	uint32_t vco_min = 1770000; /* kHz */
	uint32_t vco_max = (priv->cfg->vco_algo == 0) ? (vco_min * 2) : 3900000; /* kHz */
	uint32_t freq_khz, pll_ref;
	uint32_t sdm = 0;
	uint8_t mix_div = 2;
	uint8_t div_buf = 0;
	uint8_t div_num = 0;
	uint8_t vco_power_ref = 2;
	uint8_t refdiv2 = 0;
	uint8_t ni, si, nint, vco_fine_tune, val;
	uint8_t vco_curr_min = (priv->cfg->vco_curr_min == 0xff) ? 0x80 : ( priv->cfg->vco_curr_min << 5 );
	uint8_t vco_curr_max = (priv->cfg->vco_curr_max == 0xff) ? 0x60 : ( priv->cfg->vco_curr_max << 5 );
	/* devt->r82xx_c.vco_min = 0xff;  * VCO min/max current for R18/0x12 bits [7:5] in 0 .. 7. use 0xff for default */
	/* devt->r82xx_c.vco_max = 0xff;  * value is inverted: programmed is 7-value, that 0 is lowest current */
	uint8_t data[5];

	priv->tuner_pll_set = 0;

	if (priv->cfg->vco_algo == 2)
	{
		/* r82xx_set_pll_yc() assumes fixed maximum current */
		if (priv->last_vco_curr != vco_curr_max) {
			rc = r82xx_write_reg_mask(priv, 0x12, vco_curr_max, 0xe0);
			if (rc < 0) {
				if (priv->cfg->verbose)
					fprintf(stderr, "r82xx_set_pll(): error writing 'vco current' into i2c reg 0x12\n");
				return rc;
			}
			priv->last_vco_curr = vco_curr_max;
		}
		return r82xx_set_pll_yc(priv, freq);
	}

	/* Frequency in kHz */
	freq_khz = (freq + 500) / 1000;
	pll_ref = priv->cfg->xtal;

	rc = r82xx_write_reg_mask(priv, 0x10, refdiv2, 0x10);
	if (rc < 0) {
		if (priv->cfg->verbose)
			fprintf(stderr, "r82xx_set_pll(): error writing 'refdiv2' into i2c reg 0x10\n");
		return rc;
	}

	/* set pll autotune = 128kHz */
	rc = r82xx_write_reg_mask(priv, 0x1a, 0x00, 0x0c);
	if (rc < 0) {
		if (priv->cfg->verbose)
			fprintf(stderr, "r82xx_set_pll(): error writing 'pll autotune 128kHz' into i2c reg 0x1a\n");
		return rc;
	}

	/* set VCO current = 100 */
	if (priv->last_vco_curr != vco_curr_min) {
		rc = r82xx_write_reg_mask(priv, 0x12, vco_curr_min, 0xe0);
		if (rc < 0) {
			if (priv->cfg->verbose)
				fprintf(stderr, "r82xx_set_pll(): error writing 'vco current min' into i2c reg 0x12\n");
			return rc;
		}
		priv->last_vco_curr = vco_curr_min;
	}

#if 0
	fprintf(stderr, "vco_last = 0x%02x; vcocmin << 5 = 0x%02x; vcocmax << 5 = 0x%02x\n",
		(unsigned)priv->last_vco_curr, (unsigned)vco_curr_min, (unsigned)vco_curr_max);
#endif

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
	if (rc < 0) {
		if (priv->cfg->verbose)
			fprintf(stderr, "r82xx_set_pll(): error reading 'status' from i2c reg 0x00 .. 0x04\n");
		return rc;
	}

	if (priv->cfg->rafael_chip == CHIP_R828D)
		vco_power_ref = 1;

	vco_fine_tune = (data[4] & 0x30) >> 4;

	if (vco_fine_tune > vco_power_ref)
		div_num = div_num - 1;
	else if (vco_fine_tune < vco_power_ref)
		div_num = div_num + 1;

	rc = r82xx_write_reg_mask(priv, 0x10, div_num << 5, 0xe0);
	if (rc < 0) {
		if (priv->cfg->verbose)
			fprintf(stderr, "r82xx_set_pll(): error writing 'div_num' into i2c reg 0x10\n");
		return rc;
	}

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

#if PRINT_ACTUAL_VCO_AND_ERR
	{
	  uint64_t actual_vco = (uint64_t)2 * pll_ref * nint + (uint64_t)2 * pll_ref * sdm / 65536;
	  fprintf(stderr, "[R82XX] requested %u Hz; selected mix_div=%u vco_freq=%lu nint=%u sdm=%u; actual_vco=%lu; tuning error=%+dHz\n",
		  freq, mix_div, vco_freq, nint, sdm, actual_vco, (int32_t) (actual_vco - vco_freq) / mix_div);
	}
#endif

	if (nint > ((128 / vco_power_ref) - 1)) {
		if (priv->cfg->verbose || PRINT_PLL_ERRORS)
			fprintf(stderr, "[R82XX] No valid PLL values for %u Hz!\n", freq);
		return -1;
	}

	ni = (nint - 13) / 4;
	si = nint - 4 * ni - 13;

	rc = r82xx_write_reg(priv, 0x14, ni + (si << 6));
	if (rc < 0) {
		if (priv->cfg->verbose)
			fprintf(stderr, "r82xx_set_pll(): error writing 'ni+(si<<6)' into i2c reg 0x14\n");
		return rc;
	}

	/* pw_sdm */
	if (sdm == 0)
		val = 0x08;
	else
		val = 0x00;

	if (priv->disable_dither)
		val |= 0x10;

	rc = r82xx_write_reg_mask(priv, 0x12, val, 0x18);
	if (rc < 0) {
		if (priv->cfg->verbose)
			fprintf(stderr, "r82xx_set_pll(): error writing 'dither' into i2c reg 0x12\n");
		return rc;
	}

	rc = r82xx_write_reg(priv, 0x16, sdm >> 8);
	if (rc < 0) {
		if (priv->cfg->verbose)
			fprintf(stderr, "r82xx_set_pll(): error writing 'sdm hi' into i2c reg 0x16\n");
		return rc;
	}
	rc = r82xx_write_reg(priv, 0x15, sdm & 0xff);
	if (rc < 0) {
		if (priv->cfg->verbose)
			fprintf(stderr, "r82xx_set_pll(): error writing 'sdm lo' into i2c reg 0x12\n");
		return rc;
	}

	/* all PLL stuff / registers set for this frequency - except 8 kHz pll autotune */
	priv->tuner_pll_set = 1;

	for (i = 0; i < 2; i++) {

		/* Check if PLL has locked */
		rc = r82xx_read(priv, 0x00, data, 3);
		if (rc < 0) {
			if (priv->cfg->verbose)
				fprintf(stderr, "r82xx_set_pll(): error reading 'pll lock status' from i2c reg 0x00 .. 0x02\n");
			return rc;
		}
		if ( (data[2] & 0x40) || vco_curr_max == vco_curr_min )
			break;

		if (!i) {
			/* Didn't lock. Increase VCO current */
			if (priv->last_vco_curr != vco_curr_max) {
				rc = r82xx_write_reg_mask(priv, 0x12, vco_curr_max, 0xe0);
				if (rc < 0) {
					if (priv->cfg->verbose)
						fprintf(stderr, "r82xx_set_pll(): error writing 'vco current max' into i2c reg 0x12\n");
					return rc;
				}
				priv->last_vco_curr = vco_curr_max;
			}
		}
	}

	if (!(data[2] & 0x40)) {
		if (priv->cfg->verbose || PRINT_PLL_ERRORS)
			fprintf(stderr, "[R82XX] PLL not locked at Tuner LO %u Hz for RF %f MHz!\n",
				freq, priv->rf_freq * 1E-6);
		priv->has_lock = 0;
		return -1;
	}
#if 0
	else
		fprintf(stderr, "[R82XX] PLL locked at Tuner LO %u Hz for RF %f MHz!\n", freq, priv->rf_freq * 1E-6);
#endif

	priv->has_lock = 1;

	/* set pll autotune = 8kHz */
	rc = r82xx_write_reg_mask(priv, 0x1a, 0x08, 0x08);
	if (rc < 0 && priv->cfg->verbose)
		fprintf(stderr, "r82xx_set_pll(): error writing 'pll autotune 8kHz' into i2c reg 0x1a\n");

	return rc;
}


int r82xx_is_tuner_locked(struct r82xx_priv *priv)
{
	int rc;
	uint8_t data[5];

	/* was all PLL stuff set for last frequency? */
	if (! priv->tuner_pll_set)
		return 1;

	/* Check if PLL has locked */
	rc = r82xx_read(priv, 0x00, data, sizeof(data));
	if (rc < 0)
		return -3;
	if (!(data[2] & 0x40)) {
		if (priv->cfg->verbose || PRINT_PLL_ERRORS)
			fprintf(stderr, "[R82XX] PLL not locked at check!\n");
		return 1;
	}
	return 0;
}


static int r82xx_sysfreq_sel(struct r82xx_priv *priv,
				 enum r82xx_tuner_type type)
{
	int rc;

	uint8_t lna_top = 0xe5;		/* detect bw 3, lna top:4, predet top:2 */
	uint8_t pre_dect = 0x40;
	uint8_t air_cable1_in = 0x00;
	uint8_t cable2_in = 0x00;

	if (priv->cfg->use_predetect) {
		rc = r82xx_write_reg_mask(priv, 0x06, pre_dect, 0x40);
		if (rc < 0)
			return rc;
	}

	priv->input = air_cable1_in;

	/* Air-IN only for Astrometa */
	rc = r82xx_write_reg_mask(priv, 0x05, air_cable1_in, 0x60);
	if (rc < 0)
		return rc;
	rc = r82xx_write_reg_mask(priv, 0x06, cable2_in, 0x08);
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

		/* 0: PRE_DECT off */
		rc = r82xx_write_reg_mask(priv, 0x06, 0, 0x40);
		if (rc < 0)
			return rc;

		/* agc clk 250hz */
		rc = r82xx_write_reg_mask(priv, 0x1a, 0x30, 0x30);
		if (rc < 0)
			return rc;

		/* write LNA TOP = 3 */
		rc = r82xx_write_reg_mask(priv, 0x1d, 0x18, 0x38);
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
				 enum r82xx_tuner_type type,
				 uint32_t delsys)

{
	int rc, i;
	uint8_t data[5];

	int need_calibration = 1;

	/* BW < 6 MHz */
	uint8_t filt_q = 0x10;		/* r10[4]:low q(1'b1) */

	/* for LT Gain test */
	if (type != TUNER_ANALOG_TV) {
		rc = r82xx_write_reg_mask(priv, 0x1d, 0x00, 0x38);
		if (rc < 0)
			return rc;
	}
	priv->rf_freq = 56000 * 1000;  /* set a default frequency */
	priv->if_band_center_freq = 0;
	priv->int_freq = 3570 * 1000;
	priv->sideband = 0;

	/* Check if standard changed. If so, filter calibration is needed */
	/* as we call this function only once in rtlsdr, force calibration */

	if (need_calibration) {
		for (i = 0; i < 2; i++) {

			/* set cali clk =on */
			rc = r82xx_write_reg_mask(priv, 0x0f, 0x04, 0x04);
			if (rc < 0)
				return rc;

			priv->tuner_pll_set = 0;
			rc = r82xx_set_pll(priv, priv->rf_freq);
			if (rc < 0 || !priv->has_lock)
				return rc;

			/* Start Trigger */
			rc = r82xx_write_reg_mask_ext(priv, 0x0b, 0x10, 0x10, __FUNCTION__);
			if (rc < 0)
				return rc;

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

	/* Store current standard. If it changes, re-calibrate the tuner */
	priv->delsys = delsys;
	priv->type = type;
	priv->bw = 3;

	return 0;
}

/* measured with a Racal 6103E GSM test set at 928 MHz with -60 dBm
 * input power, for raw results see:
 * http://steve-m.de/projects/rtl-sdr/gain_measurement/r820t/
 */

#define VGA_BASE_GAIN	-47
static const int r82xx_vga_gain_steps[]  = {
	0, 26, 26, 30, 42, 35, 24, 13, 14, 32, 36, 34, 35, 37, 35, 36
/* -47, -21, 5, 35, 77, 112, 136, 149, 163, 195, 231, 265, 300, 337, 372, 408 */
};

static const int r82xx_lna_gain_steps[]  = {
	0, 9, 13, 40, 38, 13, 31, 22, 26, 31, 26, 14, 19, 5, 35, 13
};

static const int r82xx_mixer_gain_steps[]  = {
	0, 5, 10, 10, 19, 9, 10, 25, 17, 10, 8, 16, 13, 6, 3, -8
};

static void r82xx_get_rf_gain_index(int gain, int *ptr_lna_index, int *ptr_mix_index)
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

static int r82xx_get_if_gain_index(int gain)
{
	int vga_index, total_gain = VGA_BASE_GAIN;

	for (vga_index = 0; vga_index < 15; vga_index++) {
		if (total_gain >= gain)
			break;
		total_gain += r82xx_vga_gain_steps[++vga_index];
	}

	return vga_index;
}

static int r82xx_get_lna_gain_from_index(int lna_index)
{
	int i, total_gain = 0;
	if ( lna_index < 0 || lna_index > 15 )
		return 0;
	for ( i = 0; i <= lna_index; ++i )
		total_gain += r82xx_lna_gain_steps[i];
	return total_gain;
}

static int r82xx_get_mixer_gain_from_index(int mixer_index)
{
	int i, total_gain = 0;
	if ( mixer_index < 0 || mixer_index > 15 )
		return 0;
	for ( i = 0; i <= mixer_index; ++i )
		total_gain += r82xx_mixer_gain_steps[i];
	return total_gain;
}

static int r82xx_get_vga_gain_from_index(int vga_index)
{
	int i, total_gain = VGA_BASE_GAIN;
	if ( vga_index < 0 || vga_index > 15 )
		return 0;
	for ( i = 0; i <= vga_index; ++i )
		total_gain += r82xx_vga_gain_steps[i];
	return total_gain;
}


/* set HF gain (LNA/Mixer) and pass through for IF gain (VGA) */
int r82xx_set_gain(struct r82xx_priv *priv, int set_manual_gain, int gain,
  int extended_mode, int lna_gain_idx, int mixer_gain_idx, int vga_gain_idx, int *rtl_vga_control)
{
	int rc;
	int new_if_mode = priv->last_if_mode;
	uint8_t data[4];

	if (extended_mode || set_manual_gain) {

		if (set_manual_gain) {
			r82xx_get_rf_gain_index(gain, &lna_gain_idx, &mixer_gain_idx);
		}

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
		rc = r82xx_write_reg_mask(priv, 0x05, lna_gain_idx, 0x0f);
		if (rc < 0)
			return rc;

		/* Set Mixer */
		rc = r82xx_write_reg_mask(priv, 0x07, mixer_gain_idx, 0x0f);
		if (rc < 0)
			return rc;

		/* save last values */
		priv->last_manual_gain = set_manual_gain;
		priv->last_extended_mode = extended_mode;
		priv->last_LNA_value = lna_gain_idx;
		priv->last_Mixer_value = mixer_gain_idx;

		/* prepare VGA */
		if (extended_mode) {
			new_if_mode = vga_gain_idx +10000;
		}

	} else {
		/* LNA auto on = AGC */
		rc = r82xx_write_reg_mask(priv, 0x05, 0, 0x10);
		if (rc < 0)
			return rc;

		/* Mixer auto on = AGC */
		rc = r82xx_write_reg_mask(priv, 0x07, 0x10, 0x10);
		if (rc < 0)
			return rc;

		/* save last values */
		priv->last_manual_gain = set_manual_gain;
		priv->last_extended_mode = extended_mode;

#ifdef VGA_FOR_AGC_MODE
		new_if_mode = VGA_FOR_AGC_MODE;
#endif
	}

	/* Set VGA */
	rc = r82xx_set_if_mode(priv, new_if_mode, rtl_vga_control);

	return rc;
}

int r82xx_get_rf_gain(struct r82xx_priv *priv)
{
	int lna_gain = r82xx_get_lna_gain_from_index(priv->last_LNA_value);
	int mix_gain = r82xx_get_mixer_gain_from_index(priv->last_Mixer_value);
	int gain = lna_gain + mix_gain;
	return gain;
}


int r82xx_get_if_gain(struct r82xx_priv *priv)
{
	int vga_gain = r82xx_get_vga_gain_from_index(priv->last_VGA_value);
	return vga_gain;
}


/* set IF gain (VGA) */
int r82xx_set_if_mode(struct r82xx_priv *priv, int if_mode, int *rtl_vga_control)
{
	int rc = 0, vga_gain_idx = 0;

	if (rtl_vga_control)
		*rtl_vga_control = 0;

	if ( 0 == if_mode || 10016 == if_mode ) {
		vga_gain_idx = 0x10;
	}
	else if ( -2500 <= if_mode && if_mode <= 2500 ) {
		vga_gain_idx = r82xx_get_if_gain_index(if_mode);
	}
	else if ( 2500 < if_mode && if_mode < 10000 ) {
		vga_gain_idx = r82xx_get_if_gain_index(if_mode - 5000);
	}
	else if ( 10000 <= if_mode && if_mode <= 10016+15 ) {
		vga_gain_idx = if_mode -10000;
	}
	else {	/* assume 0 == default */
		fprintf(stderr, "%s: invalid if_mode value %d; setting to default: %d\n",
			__FUNCTION__, if_mode, 10000 + DEFAULT_IF_VGA_VAL );
		/* was  26.5 dB == -12 dB + 0x0b * 3.5 dB  with AGC on
		 * and  16.3 dB == -12 dB + 8 * 3.5 dB     with AGC off
		 * BUT IT makes no sense to make this different!
		 */
		/* vga_gain_idx = (priv->last_extended_mode || priv->last_manual_gain) ? 0x08 : 0x0b; */
		if_mode = 10000 + DEFAULT_IF_VGA_VAL;
		vga_gain_idx = DEFAULT_IF_VGA_VAL;
	}

#if PRINT_VGA_REG
	fprintf(stderr, "%s: writing 0x%02X (=%.1f dB) into VGA register %sactivating RTL AGC control\n", __FUNCTION__,
		(vga_gain_idx & 0x0f), (-12.0 + 3.5 * (vga_gain_idx & 0x0f)),
		( (vga_gain_idx & 0x10) && rtl_vga_control ) ? "" : "de" );
#endif
	rc = r82xx_write_reg_mask(priv, 0x0c, vga_gain_idx, 0x1f);
	if (rc < 0)
		return rc;
	priv->last_if_mode = if_mode;
	priv->last_VGA_value = vga_gain_idx;
	if ( (vga_gain_idx & 0x10) && rtl_vga_control )
		*rtl_vga_control = 1;

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

//-cs-
int r82xx_get_i2c_register(struct r82xx_priv *priv, unsigned char* data, int len)
{
	int rc, i, len1;

	// The lower 5 I2C registers can be read with the normal read fct, the upper ones are read from the cache
	if(len < 5)
		len1 = len;
	else
		len1 = 5;
	rc = r82xx_read(priv, 0x00, data, len1);
	if (rc < 0)
		return rc;
	if(len > 5)
		for (i = 5; i < len; i++)
			data[i] = r82xx_read_cache_reg(priv, i);
	return 0;
}
//-cs- end

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

	{ 3, 1200+0,      1350,   0, 0x0F, 0xEE, 0x00 },	/* centered with hpf */

	{ 3, 1300+0,      2050,  -7, 0x0F, 0x8A, 0x00 },	/* centered with hpf */
#if (WITH_ASYM_FILTER)
	{ 1, 1300+1, IFA(1300),  26, 0x0F, 0xEF, 0x60 },	/* steep low  freq edge */
	{ 2, 1300+2, IFB(1300),   0, 0x0F, 0xEF, 0x60 },	/* steep high freq edge */
#endif

	{ 3, 1500+3,      1300, -24, 0x0F, 0xEF, 0x60 },
	{ 3, 1600+0,      1900,   0, 0x0F, 0x8B, 0x00 },	/* centered with hpf */
	{ 3, 1750+3,      1400,  12, 0x0F, 0xCF, 0x60 },	/* 20 */

	{ 3, 1800+0,      1400,   0, 0x0F, 0xAF, 0x00 },

	{ 3, 1950+3,      1500,  30, 0x0F, 0x8F, 0x60 },

	{ 3, 2200+0,      1600,   0, 0x0F, 0x8F, 0x00 },
	{ 3, 3000+0,      2000,   0, 0x04, 0x8F, 0x00 },
	{ 3, 5000+0,      3570,   0, 0x0B, 0x6B, 0x00 }

};


/* settings from Oldenburger:
static const int r82xx_bws[]=     {  300,  450,  600,  900, 1100, 1200, 1300, 1500, 1800, 2200, 3000, 5000 };
static const uint8_t r82xx_0xa[]= { 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0e, 0x0f, 0x0f, 0x04, 0x0b };
static const uint8_t r82xx_0xb[]= { 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xaf, 0x8f, 0x8f, 0x6b };
static const int r82xx_if[]  =    { 1700, 1650, 1600, 1500, 1400, 1350, 1320, 1270, 1400, 1600, 2000, 3570 };
*/


static const int r82xx_bw_tablen = sizeof(IFi) / sizeof(IFi[0]);


#define FILT_HP_BW1 350000
#define FILT_HP_BW2 380000
int r82xx_set_bandwidth(struct r82xx_priv *priv, int bw, uint32_t rate, uint32_t * applied_bw, int apply)
{
	int rc;
	int i;
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
	reg_mask = 0x40;
#if USE_R82XX_ENV_VARS
	if ( priv->haveR30H ) {
		reg_1e = ( priv->valR30H << 6 );
		reg_mask = reg_mask | 0xC0;
	}
	if ( priv->haveR30L ) {
		reg_1e = reg_1e | priv->valR30L;
		reg_mask = reg_mask | 0x3F;
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

int r82xx_set_sideband(struct r82xx_priv *priv, int sideband)
{
	int rc;
	priv->sideband = sideband;
	rc = r82xx_write_reg_mask(priv, 0x07, (sideband << 7) & 0x80, 0x80);
	if (rc < 0)
		return rc;
	return 0;
}

int r82xx_get_sideband(struct r82xx_priv *priv)
{
	return priv->sideband;
}


static const uint32_t harm_sideband_xor[17] = {
	0	/*  0 - should not happen */
,	0	/*  1 - default - as without harmonic */
,	0	/*  2: ( 2 * 90 deg) % 360 = 180 deg */
,	1	/*  3: ( 3 * 90 deg) % 360 = -90 deg */
,	0	/*  4: ( 4 * 90 deg) % 360 =   0 deg */
,	0	/*  5: ( 5 * 90 deg) % 360 = +90 deg */
,	0	/*  6: ( 6 * 90 deg) % 360 = 180 deg */
,	1	/*  7: ( 7 * 90 deg) % 360 = -90 deg */
,	0	/*  8: ( 8 * 90 deg) % 360 =   0 deg */
,	0	/*  9: ( 9 * 90 deg) % 360 = +90 deg */
,	0	/* 10: (10 * 90 deg) % 360 = 180 deg */
,	1	/* 11: (11 * 90 deg) % 360 = -90 deg */
,	0	/* 12: (12 * 90 deg) % 360 =   0 deg */
,	0	/* 13: (13 * 90 deg) % 360 = +90 deg */
,	0	/* 14: (14 * 90 deg) % 360 = 180 deg */
,	1	/* 15: (15 * 90 deg) % 360 = -90 deg */
,	0	/* 16: (16 * 90 deg) % 360 = 180 deg */
};

int r82xx_flip_rtl_sideband(struct r82xx_priv *priv)
{
	return harm_sideband_xor[priv->tuner_harmonic];
}


int r82xx_set_freq64(struct r82xx_priv *priv, uint64_t freq)
{
	int rc = -1;
	int nth_harm;
	int harm = (priv->cfg->harmonic <= 0) ? DEFAULT_HARMONIC : priv->cfg->harmonic;
	uint64_t lo_freq;
	uint32_t lo_freqHarm;
	uint8_t air_cable1_in;

	nth_harm = ( freq > FIFTH_HARM_FRQ_THRESH_KHZ * (uint64_t)1000 ) ? 1 : 0;
	for ( ; nth_harm < 2; ++nth_harm )
	{
		priv->tuner_pll_set = 0;
		priv->tuner_harmonic = ( nth_harm ) ? harm : 0;

		if (!freq)
			freq = priv->rf_freq;	/* ignore zero frequency; keep last one */
		else
			priv->rf_freq = freq;

		if ( priv->sideband ^ harm_sideband_xor[priv->tuner_harmonic] )
			lo_freq = freq - priv->int_freq + priv->if_band_center_freq;
		else
			lo_freq = freq + priv->int_freq + priv->if_band_center_freq;

		lo_freqHarm = (nth_harm) ? ( lo_freq / harm ) : lo_freq;

#if PRINT_HARMONICS
		fprintf(stderr, "%s(freq = %f MHz) @ %s--> intfreq %u Hz, ifcenter %d --> f %f MHz, PLL %f MHz\n"
			, __FUNCTION__, freq * 1E-6, (priv->sideband ? "USB" : "LSB")
			, (unsigned)priv->int_freq, (int)priv->if_band_center_freq
			, lo_freq * 1E-6, lo_freqHarm * 1E-6 );
#endif

		rc = r82xx_set_mux(priv, lo_freq);
		if (rc < 0) {
			if (priv->cfg->verbose)
				fprintf(stderr, "r82xx_set_freq(): error at r82xx_set_mux()\n");
			goto err;
		}

		rc = r82xx_set_pll(priv, lo_freqHarm);
		if (rc < 0 || !priv->has_lock)
		{
			if ( !nth_harm && lo_freq > RETRY_WITH_FIFTH_HARM_KHZ * 1000 )
				continue;
			goto err;
		}

		if ( nth_harm )
		{
#if 0
			fprintf(stderr, "r82xx_set_freq(): set up for %d-th harmonic\n", harm);
#endif
		}

		break;
	}

	/* switch between 'Cable1' and 'Air-In' inputs on sticks with
	 * R828D tuner. We switch at 345 MHz, because that's where the
	 * noise-floor has about the same level with identical LNA
	 * settings. The original driver used 320 MHz. */
	air_cable1_in = (freq > MHZ(345)) ? 0x00 : 0x60;

	if ((priv->cfg->rafael_chip == CHIP_R828D) &&
		(air_cable1_in != priv->input)) {
		priv->input = air_cable1_in;
		rc = r82xx_write_reg_mask(priv, 0x05, air_cable1_in, 0x60);
		if (rc < 0 && priv->cfg->verbose)
			fprintf(stderr, "r82xx_set_freq(): error writing R828D's 'input selection' into i2c reg 0x05\n");
	}

err:
#if PRINT_PLL_ERRORS
	if (rc < 0)
		fprintf(stderr, "%s: failed=%d\n", __FUNCTION__, rc);
#endif
	return rc;
}

int r82xx_set_freq(struct r82xx_priv *priv, uint32_t freq)
{
	return r82xx_set_freq64(priv, (uint64_t)freq);
}

int r82xx_set_dither(struct r82xx_priv *priv, int dither)
{
	priv->disable_dither = !dither;
	return 0;
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

int r82xx_init(struct r82xx_priv *priv)
{
	int rc;

#if PRINT_INITIAL_REGISTERS
#define INIT_NUM_READ_REGS 16
	uint8_t		initial_register_values[INIT_NUM_READ_REGS];	/* see what is 'default' */
	int k;
	/* get initial register values - just to see .. */
	memset( &(initial_register_values[0]), 0, sizeof(initial_register_values) );
	printf("R820T/2 initial register settings:\n");
	r82xx_read(priv, 0x00, initial_register_values, sizeof(initial_register_values));
	for (k=0; k < INIT_NUM_READ_REGS; ++k)
		printf("register 0x%02x: 0x%02x\n", k, initial_register_values[k]);
	printf("\n");
#endif

	/* TODO: R828D might need r82xx_xtal_check() */
	priv->xtal_cap_sel = XTAL_HIGH_CAP_0P;

	priv->rf_freq = 0;
	priv->if_band_center_freq = 0;

	priv->last_if_mode = 0;
	priv->last_manual_gain = 0;
	priv->last_extended_mode = 0;
	priv->last_LNA_value = 0;
	priv->last_Mixer_value = 0;
	priv->last_VGA_value = DEFAULT_IF_VGA_VAL;
	priv->last_vco_curr = 0xff;

	/* Initialize override registers */
	memset( &(priv->override_data[0]), 0, NUM_REGS * sizeof(uint8_t) );
	memset( &(priv->override_mask[0]), 0, NUM_REGS * sizeof(uint8_t) );

	/* Initialize registers */
	rc = r82xx_write_arr(priv, 0x05,
			 r82xx_init_array, sizeof(r82xx_init_array));

	priv->last_vco_curr = r82xx_init_array[0x12 - 0x05] & 0xe0;

	rc = r82xx_set_tv_standard(priv, TUNER_DIGITAL_TV, 0);
	if (rc < 0)
		goto err;

	rc = r82xx_sysfreq_sel(priv, TUNER_DIGITAL_TV);

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
			priv->valR30H = atoi(pacR30Hi);
			if ( priv->valR30H > 3 ) {
				fprintf(stderr, "*** read R30_HI from environment: %d - but value should be 0 - 3 for bit [7:6]\n", priv->valR30H);
				priv->haveR30H = 0;
			}
			fprintf(stderr, "*** read R30_HI from environment: %d\n", priv->valR30H);
		}

		pacR30Lo = getenv("RTL_R820_R30_LO");
		if ( pacR30Lo ) {
			priv->haveR30L = 1;
			priv->valR30L = atoi(pacR30Lo);
			if ( priv->valR30L > 63 ) {
				fprintf(stderr, "*** read R30_LO from environment: %d - but value should be 0 - 63 for bit [5:0]\n", priv->valR30L);
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
