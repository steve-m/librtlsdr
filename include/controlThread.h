/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2019 <>
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

#ifndef __RTL_CONTROL_THREAD_H
#define __RTL_CONTROL_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	rtlsdr_dev_t *dev;
	int port;
	int wait;
	int report_i2c;
	char *addr;
	int* pDoExit;
}
ctrl_thread_data_t;
void *ctrl_thread_fn(void *arg);

#ifdef __cplusplus
}
#endif

#endif

