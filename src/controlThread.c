/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012-2013 by Hoernchen <la@tfc-server.de>
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

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <fcntl.h>
#else
#include <winsock2.h>
#include "getopt/getopt.h"
#define usleep(x) Sleep(x/1000)
#endif

#ifdef NEED_PTHREADS_WORKARROUND
#define HAVE_STRUCT_TIMESPEC
#endif
#include <pthread.h>

#include "rtl-sdr.h"
#include "rtl_tcp.h"
#include "controlThread.h"
#include "convenience/convenience.h"
#include "convenience/rtl_convenience.h"

#include "tuner_r82xx.h"

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")

typedef int socklen_t;

#else
#define closesocket close
#define SOCKADDR struct sockaddr
#define SOCKET int
#define SOCKET_ERROR -1
#endif

/* we need a message id in the protocol: 1st 2 byte (little endian) == message id */
#define USE_MSGID_IN_PROTOCOL		1

#define NUM_I2C_REGISTERS  32
#define TX_BUF_LEN (NUM_I2C_REGISTERS +4) //2 len, 1 head, 1 tail


ctrl_thread_data_t ctrl_thread_data;

void *ctrl_thread_fn(void *arg)
{
	unsigned char reg_values [NUM_I2C_REGISTERS];
#if USE_MSGID_IN_PROTOCOL
	unsigned char txbuf [2+2 +1+NUM_I2C_REGISTERS+1]; //2 type, 2 length, 1 head, 1 tail
#else
	unsigned char txbuf [NUM_I2C_REGISTERS+4]; //2 length, 1 head, 1 tail
#endif
	int r = 1;
	struct timeval tv = { 1,0 };
	struct linger ling = { 1,0 };
	SOCKET listensocket;
	SOCKET controlSocket;
	int haveControlSocket = 0;
	struct sockaddr_in local, remote;
	socklen_t rlen;

	int error = 0;
	int ret = 0, len, result;
	fd_set connfds;
	fd_set writefds;
	int bytesleft, bytessent, index;

	ctrl_thread_data_t *data = (ctrl_thread_data_t *)arg;

	rtlsdr_dev_t *dev = data->dev;
	int port = data->port;
	int wait = data->wait;
	int report_i2c = data->report_i2c;
	char *addr = data->addr;
	int* do_exit = data->pDoExit;
	u_long blockmode = 1;
	int retval;


	memset(reg_values, 0, NUM_I2C_REGISTERS);

	memset(&local, 0, sizeof(local));
	local.sin_family = AF_INET;
	local.sin_port = htons(port);
	local.sin_addr.s_addr = inet_addr(addr);

	listensocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	setsockopt(listensocket, SOL_SOCKET, SO_REUSEADDR, (char *)&r, sizeof(int));
	setsockopt(listensocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
	retval = bind(listensocket, (struct sockaddr *)&local, sizeof(local));
	if (retval == SOCKET_ERROR)
		error = 1;
#ifdef _WIN32
	ioctlsocket(listensocket, FIONBIO, &blockmode);
#else
	r = fcntl(listensocket, F_GETFL, 0);
	r = fcntl(listensocket, F_SETFL, r | O_NONBLOCK);
#endif

	while (1) {
		printf("listening on Control port %d...\n", port);
		retval = listen(listensocket, 1);
		if (retval == SOCKET_ERROR)
			error = 1;
		while (1) {
			FD_ZERO(&connfds);
			FD_SET(listensocket, &connfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(listensocket + 1, &connfds, NULL, NULL, &tv);
			if (*do_exit) {
				goto close;
			}
			else if (r) {
				rlen = sizeof(remote);
				controlSocket = accept(listensocket, (struct sockaddr *)&remote, &rlen);
				haveControlSocket = 1;
				break;
			}
		}

		setsockopt(controlSocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));

		printf("Control client accepted!\n");
		usleep(5000000);

		while (1) {

			/* check if i2c reporting is to be (de)activated */
			if ( report_i2c && !data->report_i2c )
				report_i2c = 0;
			else if ( !report_i2c && data->report_i2c )
				report_i2c = 1;

			/* @TODO: check if something else has to be transmitted */

			if ( !report_i2c )
				goto sleep;

			result = rtlsdr_get_tuner_i2c_register(dev, reg_values, NUM_I2C_REGISTERS);
			/* printf("rtlsdr_get_tuner_i2c_register\n"); */
			memset(txbuf, 0, TX_BUF_LEN);
			if (result)
				goto sleep;

			/* Little Endian */
			len = 0;
			/* we need some message id: use enum RTL_TCP_COMMANDS */
#if USE_MSGID_IN_PROTOCOL
			txbuf[len++] = REPORT_I2C_REGS & 0x0FF;
			txbuf[len++] = (REPORT_I2C_REGS >> 8) & 0x0FF;
			/* following message length in Little Endian */
			txbuf[len++] = TX_BUF_LEN - 2 - 2;	/* sub message id and length field */
#else
			txbuf[len++] = TX_BUF_LEN - 2;	/* sub message id and length field */
#endif
			txbuf[len++] = 0;
			
			/* now the message contents */
			txbuf[len++] = 0x55;			/* @CS: do we need this? */
			memcpy(&txbuf[len], reg_values, NUM_I2C_REGISTERS);
			txbuf[TX_BUF_LEN - 1] = 0xaa;	/* @CS: do we need this? */
			len = sizeof(txbuf);

			/* now start (possibly blocking) transmission */
			bytessent = 0;
			bytesleft = len;
			index = 0;

			while (bytesleft > 0) {
				FD_ZERO(&writefds);
				FD_SET(controlSocket, &writefds);
				tv.tv_sec = 1;
				tv.tv_usec = 0;
				r = select(controlSocket + 1, NULL, &writefds, NULL, &tv);
				if (r) {
					bytessent = send(controlSocket, &txbuf[index], bytesleft, 0);
					bytesleft -= bytessent;
					index += bytessent;
				}
				if (bytessent == SOCKET_ERROR || *do_exit) {
					goto close;
				}
			}
sleep:
			usleep(wait);
		}
close:
		if (haveControlSocket)
			closesocket(controlSocket);
		if (*do_exit)
		{
			closesocket(listensocket);
			printf("Control Thread terminates\n");
			break;
		}
	}
	return 0;
}

