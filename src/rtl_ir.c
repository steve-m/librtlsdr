/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2009 Antti Palosaari <crope@iki.fi>
 * Copyright (C) 2011 Antti Palosaari <crope@iki.fi>
 * Copyright (C) 2012 Thomas Mair <thomas.mair86@googlemail.com>
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 * Copyright (C) 2016 by Robert X. Seger <rseger@gmx.co.uk>
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
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include "getopt/getopt.h"
#define usleep(x) Sleep(x/1000)
#ifdef _MSC_VER
#define round(x) (x > 0.0 ? floor(x + 0.5): ceil(x - 0.5))
#endif
#define _USE_MATH_DEFINES
#endif

#include <math.h>
#include <pthread.h>
#include <libusb.h>

#include "rtl-sdr.h"
#include "convenience/convenience.h"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

// Registers from Linux drivers/media/usb/dvb-usb-v2/rtl28xxu.h TODO: merge into librtlsdr?
struct rtl28xxu_req {
	uint16_t value;
	uint16_t index;
	uint16_t size;
	uint8_t *data;
};

struct rtl28xxu_reg_val {
	uint16_t reg;
	uint8_t val;
};

struct rtl28xxu_reg_val_mask {
	uint16_t reg;
	uint8_t val;
	uint8_t mask;
};

/*
 * memory map
 *
 * 0x0000 DEMOD : demodulator
 * 0x2000 USB   : SIE, USB endpoint, debug, DMA
 * 0x3000 SYS   : system
 * 0xfc00 RC    : remote controller (not RTL2831U)
 */

/*
 * USB registers
 */
/* SIE Control Registers */
#define USB_SYSCTL         0x2000 /* USB system control */
#define USB_SYSCTL_0       0x2000 /* USB system control */
#define USB_SYSCTL_1       0x2001 /* USB system control */
#define USB_SYSCTL_2       0x2002 /* USB system control */
#define USB_SYSCTL_3       0x2003 /* USB system control */
#define USB_IRQSTAT        0x2008 /* SIE interrupt status */
#define USB_IRQEN          0x200C /* SIE interrupt enable */
#define USB_CTRL           0x2010 /* USB control */
#define USB_STAT           0x2014 /* USB status */
#define USB_DEVADDR        0x2018 /* USB device address */
#define USB_TEST           0x201C /* USB test mode */
#define USB_FRAME_NUMBER   0x2020 /* frame number */
#define USB_FIFO_ADDR      0x2028 /* address of SIE FIFO RAM */
#define USB_FIFO_CMD       0x202A /* SIE FIFO RAM access command */
#define USB_FIFO_DATA      0x2030 /* SIE FIFO RAM data */
/* Endpoint Registers */
#define EP0_SETUPA         0x20F8 /* EP 0 setup packet lower byte */
#define EP0_SETUPB         0x20FC /* EP 0 setup packet higher byte */
#define USB_EP0_CFG        0x2104 /* EP 0 configure */
#define USB_EP0_CTL        0x2108 /* EP 0 control */
#define USB_EP0_STAT       0x210C /* EP 0 status */
#define USB_EP0_IRQSTAT    0x2110 /* EP 0 interrupt status */
#define USB_EP0_IRQEN      0x2114 /* EP 0 interrupt enable */
#define USB_EP0_MAXPKT     0x2118 /* EP 0 max packet size */
#define USB_EP0_BC         0x2120 /* EP 0 FIFO byte counter */
#define USB_EPA_CFG        0x2144 /* EP A configure */
#define USB_EPA_CFG_0      0x2144 /* EP A configure */
#define USB_EPA_CFG_1      0x2145 /* EP A configure */
#define USB_EPA_CFG_2      0x2146 /* EP A configure */
#define USB_EPA_CFG_3      0x2147 /* EP A configure */
#define USB_EPA_CTL        0x2148 /* EP A control */
#define USB_EPA_CTL_0      0x2148 /* EP A control */
#define USB_EPA_CTL_1      0x2149 /* EP A control */
#define USB_EPA_CTL_2      0x214A /* EP A control */
#define USB_EPA_CTL_3      0x214B /* EP A control */
#define USB_EPA_STAT       0x214C /* EP A status */
#define USB_EPA_IRQSTAT    0x2150 /* EP A interrupt status */
#define USB_EPA_IRQEN      0x2154 /* EP A interrupt enable */
#define USB_EPA_MAXPKT     0x2158 /* EP A max packet size */
#define USB_EPA_MAXPKT_0   0x2158 /* EP A max packet size */
#define USB_EPA_MAXPKT_1   0x2159 /* EP A max packet size */
#define USB_EPA_MAXPKT_2   0x215A /* EP A max packet size */
#define USB_EPA_MAXPKT_3   0x215B /* EP A max packet size */
#define USB_EPA_FIFO_CFG   0x2160 /* EP A FIFO configure */
#define USB_EPA_FIFO_CFG_0 0x2160 /* EP A FIFO configure */
#define USB_EPA_FIFO_CFG_1 0x2161 /* EP A FIFO configure */
#define USB_EPA_FIFO_CFG_2 0x2162 /* EP A FIFO configure */
#define USB_EPA_FIFO_CFG_3 0x2163 /* EP A FIFO configure */
/* Debug Registers */
#define USB_PHYTSTDIS      0x2F04 /* PHY test disable */
#define USB_TOUT_VAL       0x2F08 /* USB time-out time */
#define USB_VDRCTRL        0x2F10 /* UTMI vendor signal control */
#define USB_VSTAIN         0x2F14 /* UTMI vendor signal status in */
#define USB_VLOADM         0x2F18 /* UTMI load vendor signal status in */
#define USB_VSTAOUT        0x2F1C /* UTMI vendor signal status out */
#define USB_UTMI_TST       0x2F80 /* UTMI test */
#define USB_UTMI_STATUS    0x2F84 /* UTMI status */
#define USB_TSTCTL         0x2F88 /* test control */
#define USB_TSTCTL2        0x2F8C /* test control 2 */
#define USB_PID_FORCE      0x2F90 /* force PID */
#define USB_PKTERR_CNT     0x2F94 /* packet error counter */
#define USB_RXERR_CNT      0x2F98 /* RX error counter */
#define USB_MEM_BIST       0x2F9C /* MEM BIST test */
#define USB_SLBBIST        0x2FA0 /* self-loop-back BIST */
#define USB_CNTTEST        0x2FA4 /* counter test */
#define USB_PHYTST         0x2FC0 /* USB PHY test */
#define USB_DBGIDX         0x2FF0 /* select individual block debug signal */
#define USB_DBGMUX         0x2FF4 /* debug signal module mux */

/*
 * SYS registers
 */
/* demod control registers */
#define SYS_SYS0           0x3000 /* include DEMOD_CTL, GPO, GPI, GPOE */
#define SYS_DEMOD_CTL      0x3000 /* control register for DVB-T demodulator */
/* GPIO registers */
#define SYS_GPIO_OUT_VAL   0x3001 /* output value of GPIO */
#define SYS_GPIO_IN_VAL    0x3002 /* input value of GPIO */
#define SYS_GPIO_OUT_EN    0x3003 /* output enable of GPIO */
#define SYS_SYS1           0x3004 /* include GPD, SYSINTE, SYSINTS, GP_CFG0 */
#define SYS_GPIO_DIR       0x3004 /* direction control for GPIO */
#define SYS_SYSINTE        0x3005 /* system interrupt enable */
#define SYS_SYSINTS        0x3006 /* system interrupt status */
#define SYS_GPIO_CFG0      0x3007 /* PAD configuration for GPIO0-GPIO3 */
#define SYS_SYS2           0x3008 /* include GP_CFG1 and 3 reserved bytes */
#define SYS_GPIO_CFG1      0x3008 /* PAD configuration for GPIO4 */
#define SYS_DEMOD_CTL1     0x300B

/* IrDA registers */
#define SYS_IRRC_PSR       0x3020 /* IR protocol selection */
#define SYS_IRRC_PER       0x3024 /* IR protocol extension */
#define SYS_IRRC_SF        0x3028 /* IR sampling frequency */
#define SYS_IRRC_DPIR      0x302C /* IR data package interval */
#define SYS_IRRC_CR        0x3030 /* IR control */
#define SYS_IRRC_RP        0x3034 /* IR read port */
#define SYS_IRRC_SR        0x3038 /* IR status */
/* I2C master registers */
#define SYS_I2CCR          0x3040 /* I2C clock */
#define SYS_I2CMCR         0x3044 /* I2C master control */
#define SYS_I2CMSTR        0x3048 /* I2C master SCL timing */
#define SYS_I2CMSR         0x304C /* I2C master status */
#define SYS_I2CMFR         0x3050 /* I2C master FIFO */

/*
 * IR registers
 */
#define IR_RX_BUF          0xFC00
#define IR_RX_IE           0xFD00
#define IR_RX_IF           0xFD01
#define IR_RX_CTRL         0xFD02
#define IR_RX_CFG          0xFD03
#define IR_MAX_DURATION0   0xFD04
#define IR_MAX_DURATION1   0xFD05
#define IR_IDLE_LEN0       0xFD06
#define IR_IDLE_LEN1       0xFD07
#define IR_GLITCH_LEN      0xFD08
#define IR_RX_BUF_CTRL     0xFD09
#define IR_RX_BUF_DATA     0xFD0A
#define IR_RX_BC           0xFD0B
#define IR_RX_CLK          0xFD0C
#define IR_RX_C_COUNT_L    0xFD0D
#define IR_RX_C_COUNT_H    0xFD0E
#define IR_SUSPEND_CTRL    0xFD10
#define IR_ERR_TOL_CTRL    0xFD11
#define IR_UNIT_LEN        0xFD12
#define IR_ERR_TOL_LEN     0xFD13
#define IR_MAX_H_TOL_LEN   0xFD14
#define IR_MAX_L_TOL_LEN   0xFD15
#define IR_MASK_CTRL       0xFD16
#define IR_MASK_DATA       0xFD17
#define IR_RES_MASK_ADDR   0xFD18
#define IR_RES_MASK_T_LEN  0xFD19

static volatile int do_exit = 0;

struct dongle_state
{
	int      exit_flag;
    int      rc_active;
	pthread_t thread;
	rtlsdr_dev_t *dev;
	int      dev_index;
};

// Register I/O
static int rtl28xxu_wr_regs(struct dongle_state *d, uint16_t reg, uint8_t *val, int len)
{
    /* TODO
	struct rtl28xxu_req req;

	if (reg < 0x3000)
		req.index = CMD_USB_WR;
	else if (reg < 0x4000)
		req.index = CMD_SYS_WR;
	else
		req.index = CMD_IR_WR;

	req.value = reg;
	req.size = len;
	req.data = val;

	return rtl28xxu_ctrl_msg(d, &req);
    */
}

static int rtl28xxu_rd_regs(struct dongle_state *d, uint16_t reg, uint8_t *val, int len)
{
    /* TODO
	struct rtl28xxu_req req;

	if (reg < 0x3000)
		req.index = CMD_USB_RD;
	else if (reg < 0x4000)
		req.index = CMD_SYS_RD;
	else
		req.index = CMD_IR_RD;

	req.value = reg;
	req.size = len;
	req.data = val;

	return rtl28xxu_ctrl_msg(d, &req);
    */
}

static int rtl28xxu_wr_reg(struct dongle_state *d, uint16_t reg, uint8_t val)
{
	return rtl28xxu_wr_regs(d, reg, &val, 1);
}

static int rtl28xxu_rd_reg(struct dongle_state *d, uint16_t reg, uint8_t *val)
{
	return rtl28xxu_rd_regs(d, reg, val, 1);
}

static int rtl28xxu_wr_reg_mask(struct dongle_state *d, uint16_t reg, uint8_t val,
		uint8_t mask)
{
	int ret;
	uint8_t tmp;

	/* no need for read if whole reg is written */
	if (mask != 0xff) {
		ret = rtl28xxu_rd_reg(d, reg, &tmp);
		if (ret)
			return ret;

		val &= mask;
		tmp &= ~mask;
		val |= tmp;
	}

	return rtl28xxu_wr_reg(d, reg, val);
}

struct rtlsdr_dev2 {
	libusb_context *ctx;
	struct libusb_device_handle *devh;
};

void dongle_init(struct dongle_state *s)
{
    bzero(s, sizeof(struct dongle_state));
}

struct dongle_state dongle;

void usage(void)
{
	fprintf(stderr,
		"rtl_ir\n\n"
		"Use:\trtl_ir [-options]\n"
		"\t[-d device_index (default: 0)]\n");
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

static void *dongle_thread_fn(void *arg)
{
	struct dongle_state *s = arg;
    printf("TODO\n");
	//rtlsdr_read_async(s->dev, rtlsdr_callback, s, 0, s->buf_len);
	return 0;
}

static int rtl2832u_rc_query(struct dongle_state *d)
{
	int ret, i, len;
    rtlsdr_dev_t *dev = d->dev;
	uint8_t buf[128];
	static const struct rtl28xxu_reg_val_mask refresh_tab[] = {
		{IR_RX_IF,               0x03, 0xff},
		{IR_RX_BUF_CTRL,         0x80, 0xff},
		{IR_RX_CTRL,             0x80, 0xff},
	};

	/* init remote controller */
	if (!d->rc_active) {
		static const struct rtl28xxu_reg_val_mask init_tab[] = {
			{SYS_DEMOD_CTL1,         0x00, 0x04},
			{SYS_DEMOD_CTL1,         0x00, 0x08},
			{USB_CTRL,               0x20, 0x20},
			{SYS_GPIO_DIR,           0x00, 0x08},
			{SYS_GPIO_OUT_EN,        0x08, 0x08},
			{SYS_GPIO_OUT_VAL,       0x08, 0x08},
			{IR_MAX_DURATION0,       0xd0, 0xff},
			{IR_MAX_DURATION1,       0x07, 0xff},
			{IR_IDLE_LEN0,           0xc0, 0xff},
			{IR_IDLE_LEN1,           0x00, 0xff},
			{IR_GLITCH_LEN,          0x03, 0xff},
			{IR_RX_CLK,              0x09, 0xff},
			{IR_RX_CFG,              0x1c, 0xff},
			{IR_MAX_H_TOL_LEN,       0x1e, 0xff},
			{IR_MAX_L_TOL_LEN,       0x1e, 0xff},
			{IR_RX_CTRL,             0x80, 0xff},
		};

		for (i = 0; i < ARRAY_SIZE(init_tab); i++) {
			ret = rtl28xxu_wr_reg_mask(d, init_tab[i].reg,
					init_tab[i].val, init_tab[i].mask);
			if (ret)
				goto err;
		}

		d->rc_active = 1;
	}

	ret = rtl28xxu_rd_reg(d, IR_RX_IF, &buf[0]);
	if (ret)
		goto err;

	if (buf[0] != 0x83)
		goto exit;

	ret = rtl28xxu_rd_reg(d, IR_RX_BC, &buf[0]);
	if (ret)
		goto err;

	len = buf[0];

	/* read raw code from hw */
	ret = rtl28xxu_rd_regs(d, IR_RX_BUF, buf, len);
	if (ret)
		goto err;

	/* let hw receive new code */
	for (i = 0; i < ARRAY_SIZE(refresh_tab); i++) {
		ret = rtl28xxu_wr_reg_mask(d, refresh_tab[i].reg,
				refresh_tab[i].val, refresh_tab[i].mask);
		if (ret)
			goto err;
	}

	/* pass data to Kernel IR decoder */
	//TODO init_ir_raw_event(&ev);

    printf("IR: \n");
	for (i = 0; i < len; i++) {
		//ev.pulse = buf[i] >> 7;
		//ev.duration = 50800 * (buf[i] & 0x7f);

        printf("pulse %d, duration %d\n", buf[i] >> 7, 50800 * (buf[i] & 0x7f));
        //TODO
		//ir_raw_event_store_with_filter(d->rc_dev, &ev);
	}

	/* 'flush' ir_raw_event_store_with_filter() */
    /*TODO
	ir_raw_event_set_idle(d->rc_dev, true);
	ir_raw_event_handle(d->rc_dev);
    */
exit:
	return ret;
err:
	//dev_dbg(&d->intf->dev, "failed=%d\n", ret);
	return ret;
}

int main(int argc, char **argv) {
#ifndef _WIN32
	struct sigaction sigact;
#endif
	int r, opt;
	int dev_given = 0;
	dongle_init(&dongle);

	while ((opt = getopt(argc, argv, "d:h")) != -1) {
		switch (opt) {
		case 'd':
			dongle.dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'h':
		default:
			usage();
			break;
		}
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

	verbose_reset_buffer(dongle.dev);

	usleep(100000);
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

	rtlsdr_close(dongle.dev);
	return r >= 0 ? r : -r;

    return 0;
}
