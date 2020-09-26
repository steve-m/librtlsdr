/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012-2013 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Dimitri Stolnikov <horiz0n@gmx.net>
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

#ifndef __RTL_TCP_H
#define __RTL_TCP_H

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * This enum defines the possible commands in rtl_tcp
 * commands 0x01..0x0E are compatible to osmocom's rtlsdr
 * see https://github.com/osmocom/rtl-sdr/blob/master/src/rtl_tcp.c
 * commands >= 0x40 are extensions
 */
enum RTL_TCP_COMMANDS {
    SET_FREQUENCY             = 0x01,   /* sets frequency - amending hi word of SET_FREQ_HI32 if present */
    SET_FREQ_HI32             = 0x56,   /* in addition to SET_FREQUENCY */
    SET_SAMPLE_RATE           = 0x02,
    SET_GAIN_MODE             = 0x03,
    SET_GAIN                  = 0x04,
    SET_FREQUENCY_CORRECTION  = 0x05,
    SET_IF_STAGE              = 0x06,
    SET_TEST_MODE             = 0x07,
    SET_AGC_MODE              = 0x08,
    SET_DIRECT_SAMPLING       = 0x09,
    SET_OFFSET_TUNING         = 0x0A,
    SET_RTL_CRYSTAL           = 0x0B,
    SET_TUNER_CRYSTAL         = 0x0C,
    SET_TUNER_GAIN_BY_INDEX   = 0x0D,
#if 1
    /* development branch since 2018-10-03 */
    SET_BIAS_TEE              = 0x0E,
    SET_TUNER_BANDWIDTH       = 0x40,
#else
    /* prev code - used in ExtIO - to build compatible rtl_tcp.exe */
    SET_TUNER_BANDWIDTH       = 0x0E,
    SET_BIAS_TEE              = 0x0F
#endif
    UDP_ESTABLISH             = 0x41,
    UDP_TERMINATE             = 0x42,
    SET_I2C_TUNER_REGISTER    = 0x43,   /* for experiments: 32 bit data word:
                                         * 31 .. 20: register (12 bits)
                                         * 19 .. 12: mask (8 bits)
                                         * 11 ..  0: data (12 bits) */
    SET_I2C_TUNER_OVERRIDE    = 0x44,   /* encoding as with SET_I2C_TUNER_REGISTER
                                         * data (bits 11 .. 0) > 255 removes override */
    SET_TUNER_BW_IF_CENTER    = 0x45,   /* freq from SET_FREQUENCY stays in center;
                                         * the bandwidth (from SET_TUNER_BANDWIDTH)
                                         * is set to be centered at given IF frequency */
    SET_TUNER_IF_MODE         = 0x46,   /* set tuner IF mode - or gain */
    SET_SIDEBAND              = 0x47,   /* Mixer Sideband for R820T */
    REPORT_I2C_REGS           = 0x48,   /* perodically report I2C registers
                                         * - if reverse channel is enabled */

    GPIO_SET_OUTPUT_MODE      = 0x49,   /* rtlsdr_set_gpio_output() */
    GPIO_SET_INPUT_MODE       = 0x50,   /* rtlsdr_set_gpio_input() */
    GPIO_GET_IO_STATUS        = 0x51,   /* rtlsdr_set_gpio_status() */
    GPIO_WRITE_PIN            = 0x52,   /* rtlsdr_set_gpio_output() and rtlsdr_set_gpio_bit() */
    GPIO_READ_PIN             = 0x53,   /* rtlsdr_get_gpio_bit() */
    GPIO_GET_BYTE             = 0x54,   /* rtlsdr_get_gpio_byte() */
    
    IS_TUNER_PLL_LOCKED       = 0x55,   /* rtlsdr_is_tuner_PLL_locked() */

    /* SET_FREQ_HI32          = 0x56,    * rtlsdr_set_center_freq64() */
};

#ifdef __cplusplus
}
#endif

#endif
