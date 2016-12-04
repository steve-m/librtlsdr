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
 */
enum RTL_TCP_COMMANDS {
    SET_FREQUENCY             = 0x01,
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
    SET_TUNER_BANDWIDTH       = 0x0E,
    SET_BIAS_TEE              = 0x0F,
    UDP_ESTABLISH             = 0x10
};

#ifdef __cplusplus
}
#endif

#endif
