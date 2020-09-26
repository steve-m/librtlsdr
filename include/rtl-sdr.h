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

#ifndef __RTL_SDR_H
#define __RTL_SDR_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stddef.h>
#include <rtl-sdr_export.h>


typedef struct rtlsdr_dev rtlsdr_dev_t;

RTLSDR_API uint32_t rtlsdr_get_device_count(void);

RTLSDR_API const char* rtlsdr_get_device_name(uint32_t index);

/*!
 * Get USB device strings.
 *
 * NOTE: The string arguments must provide space for up to 256 bytes.
 *
 * \param index the device index
 * \param manufact manufacturer name, may be NULL
 * \param product product name, may be NULL
 * \param serial serial number, may be NULL
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_get_device_usb_strings(uint32_t index,
					     char *manufact,
					     char *product,
					     char *serial);

/*!
 * Get device index by USB serial string descriptor.
 *
 * \param serial serial string of the device
 * \return device index of first device where the name matched
 * \return -1 if name is NULL
 * \return -2 if no devices were found at all
 * \return -3 if devices were found, but none with matching name
 */
RTLSDR_API int rtlsdr_get_index_by_serial(const char *serial);

/*!
 * Open device index by index.
 *
 * \param pointer where to save the device handle, which again is a pointer
 * \param serial serial string of the device
 * \return device index to be opened
 * \return -1 if device or libusb is inaccessible
 * \return -3 if device permissions don't fit:
 *              check, if udev rules file rtl-sdr.rules needs to be installed
 * \return negative, for a libusb error code or some other initialization error
 */
RTLSDR_API int rtlsdr_open(rtlsdr_dev_t **dev, uint32_t index);

/*!
 * Close device.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \return -1 if device handle was already close - or never opened
 */
RTLSDR_API int rtlsdr_close(rtlsdr_dev_t *dev);

/* configuration functions */

/*!
 * Set crystal oscillator frequencies used for the RTL2832 and the tuner IC.
 *
 * Usually both ICs use the same clock. Changing the clock may make sense if
 * you are applying an external clock to the tuner or to compensate the
 * frequency (and samplerate) error caused by the original (cheap) crystal.
 *
 * NOTE: Call this function only if you fully understand the implications.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param rtl_freq frequency value used to clock the RTL2832 in Hz
 * \param tuner_freq frequency value used to clock the tuner IC in Hz
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_xtal_freq(rtlsdr_dev_t *dev, uint32_t rtl_freq,
				    uint32_t tuner_freq);

/*!
 * Get crystal oscillator frequencies used for the RTL2832 and the tuner IC.
 *
 * Usually both ICs use the same clock.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param rtl_freq frequency value used to clock the RTL2832 in Hz
 * \param tuner_freq frequency value used to clock the tuner IC in Hz
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_get_xtal_freq(rtlsdr_dev_t *dev, uint32_t *rtl_freq,
				    uint32_t *tuner_freq);

/*!
 * Get USB device strings.
 *
 * NOTE: The string arguments must provide space for up to 256 bytes.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param manufact manufacturer name, may be NULL
 * \param product product name, may be NULL
 * \param serial serial number, may be NULL
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_get_usb_strings(rtlsdr_dev_t *dev, char *manufact,
				      char *product, char *serial);

/*!
 * Write the device EEPROM
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param data buffer of data to be written
 * \param offset address where the data should be written
 * \param len length of the data
 * \return 0 on success
 * \return -1 if device handle is invalid
 * \return -2 if EEPROM size is exceeded
 * \return -3 if no EEPROM was found
 */

RTLSDR_API int rtlsdr_write_eeprom(rtlsdr_dev_t *dev, uint8_t *data,
				  uint8_t offset, uint16_t len);

/*!
 * Read the device EEPROM
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param data buffer where the data should be written
 * \param offset address where the data should be read from
 * \param len length of the data
 * \return 0 on success
 * \return -1 if device handle is invalid
 * \return -2 if EEPROM size is exceeded
 * \return -3 if no EEPROM was found
 */

RTLSDR_API int rtlsdr_read_eeprom(rtlsdr_dev_t *dev, uint8_t *data,
				  uint8_t offset, uint16_t len);

/*!
 * Set the frequency the device is tuned to.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param frequency in Hz
 * \return 0 on success
 * \return < 0 if device handle is invalid or some other error
 */
RTLSDR_API int rtlsdr_set_center_freq(rtlsdr_dev_t *dev, uint32_t freq);

/*!
 * Set the frequency the device is tuned to: allow frequency > 32 bit ~= 4.29 GHz
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param frequency in Hz in 64 bit
 * \return 0 on success
 * \return < 0 if device handle is invalid or some other error
 */
RTLSDR_API int rtlsdr_set_center_freq64(rtlsdr_dev_t *dev, uint64_t freq);

/*!
 * Set harmonic reception - for R820T/2 tuner
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param harmonic - receive n'th harmonic. 0 = default for disabling this
 * \return 0 on success
 * \return < 0 if device handle is invalid or some other error
 */
RTLSDR_API int rtlsdr_set_harmonic_rx(rtlsdr_dev_t *dev, int harmonic);

/*!
 * Check, if tuner PLL (frequency) is still locked.
 * Tuner/PLL might loose lock (at high frequencies),
 * e.g. for temperature reasons
 *
 * \param dev the device handle given by rtlsdr_open()
 * \return   1: PLL is NOT locked
 * \return   0: PLL HAS lock
 * \return < 0: if device handle is invalid or some other error
 * \return -2: not supported for devices' tuner
 */
RTLSDR_API int rtlsdr_is_tuner_PLL_locked(rtlsdr_dev_t *dev);


/*!
 * Get actual frequency the device is tuned to.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \return 0 on error, frequency in Hz otherwise
 */
RTLSDR_API uint32_t rtlsdr_get_center_freq(rtlsdr_dev_t *dev);

/*!
 * Get actual frequency the device is tuned to.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \return 0 on error, frequency in Hz otherwise
 */
RTLSDR_API uint64_t rtlsdr_get_center_freq64(rtlsdr_dev_t *dev);


/*!
 * Set the frequency correction value for the device.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param ppm correction value in parts per million (ppm)
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_freq_correction(rtlsdr_dev_t *dev, int ppm);

/*!
 * Get actual frequency correction value of the device.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \return correction value in parts per million (ppm)
 *         if dev is valid, no error can occur
 */
RTLSDR_API int rtlsdr_get_freq_correction(rtlsdr_dev_t *dev);

enum rtlsdr_tuner {
	RTLSDR_TUNER_UNKNOWN = 0,
	RTLSDR_TUNER_E4000,
	RTLSDR_TUNER_FC0012,
	RTLSDR_TUNER_FC0013,
	RTLSDR_TUNER_FC2580,
	RTLSDR_TUNER_R820T,		/* or R820T2 - R820T and R820T2 is not disdinguishable */
	RTLSDR_TUNER_R828D
};

/*!
 * Get the tuner type.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \return RTLSDR_TUNER_UNKNOWN on error, tuner type otherwise
 */
RTLSDR_API enum rtlsdr_tuner rtlsdr_get_tuner_type(rtlsdr_dev_t *dev);

/*!
 * Get a list of gains supported by the tuner.
 *
 * NOTE: The gains argument must be preallocated by the caller. If NULL is
 * being given instead, the number of available gain values will be returned.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param gains array of gain values. In tenths of a dB, 115 means 11.5 dB.
 * \return <= 0 on error, number of available (returned) gain values otherwise
 */
RTLSDR_API int rtlsdr_get_tuner_gains(rtlsdr_dev_t *dev, int *gains);

/*!
 * Set the gain for the device.
 * Manual gain mode must be enabled for this to work.
 *
 * Valid gain values (in tenths of a dB) for the E4000 tuner:
 * -10, 15, 40, 65, 90, 115, 140, 165, 190,
 * 215, 240, 290, 340, 420, 430, 450, 470, 490
 *
 * Valid gain values may be queried with \ref rtlsdr_get_tuner_gains function.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param gain in tenths of a dB, 115 means 11.5 dB.
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_tuner_gain(rtlsdr_dev_t *dev, int gain);

/*!
 * Set (and retrieve) the bandwidth for the device.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param bw bandwidth in Hz. Zero means automatic BW selection.
 * \param applied_bw is applied bandwidth in Hz, or 0 if unknown
 * \param apply_bw: 1 to really apply configure the tuner chip; 0 for just returning applied_bw
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_and_get_tuner_bandwidth(rtlsdr_dev_t *dev, uint32_t bw, uint32_t *applied_bw, int apply_bw );

/*!
 * Set the bandwidth for the device.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param bw bandwidth in Hz. Zero means automatic BW selection.
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_tuner_bandwidth(rtlsdr_dev_t *dev, uint32_t bw );

/*!
 * Sets the center of the filtered tuner band(width)
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param if_band_center_freq in Hz. Zero means, that band center shall be at zero (=default).
 *    set if_band_center_freq = +samplerate/4 to have the filtered band centered at output's right half.
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_tuner_band_center(rtlsdr_dev_t *dev, int32_t if_band_center_freq );

/*!
 * Set the mixer sideband for the device.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param sideband mixer sideband 0 means lower sideband, 1 means upper sideband.
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_tuner_sideband(rtlsdr_dev_t *dev, int sideband);

/*!
 * Get actual (RF / HF) gain the device is configured to - excluding the IF gain.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \return 0 on error, gain in tenths of a dB, 115 means 11.5 dB.
 *         unfortunately it's impossible to distinguish error against 0 dB
 */
RTLSDR_API int rtlsdr_get_tuner_gain(rtlsdr_dev_t *dev);

/*!
 * Set LNA / Mixer / VGA Device Gain for R820T/2 device is configured to.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param lna_gain index in 0 .. 15: 0 == min;   see tuner_r82xx.c table r82xx_lna_gain_steps[]
 * \param mixer_gain index in 0 .. 15: 0 == min; see tuner_r82xx.c table r82xx_mixer_gain_steps[]
 * \param vga_gain index in 0 .. 15: 0 == -12 dB; 15 == 40.5 dB; => 3.5 dB/step;
 *        vga_gain index 16 activates AGC for VGA controlled from RTL2832
 *     see tuner_r82xx.c table r82xx_vga_gain_steps[]
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_tuner_gain_ext(rtlsdr_dev_t *dev, int lna_gain, int mixer_gain, int vga_gain);

/*!
 * Set the intermediate frequency gain for the device.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param stage intermediate frequency gain stage number (1 to 6 for E4000)
 * \param gain in tenths of a dB, -30 means -3.0 dB.
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_tuner_if_gain(rtlsdr_dev_t *dev, int stage, int gain);

/*!
 * Set the gain mode (automatic/manual) for the device.
 * Manual gain mode must be enabled for the gain setter function to work.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param manual gain mode, 1 means manual gain mode shall be enabled.
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_tuner_gain_mode(rtlsdr_dev_t *dev, int manual);

/*!
 * Set the agc_variant for automatic gain mode for the device (only R820T/2).
 * Automatic gain mode must be enabled for the gain setter function to work.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param if_mode:
 *         0           set automatic VGA, which is controlled from RTL2832
 *     -2500 .. +2500: set fixed IF gain in tenths of a dB, 115 means 11.5 dB.
 *                     use -1 or +1 in case you neither want attenuation nor gain.
 *                     this equals the VGA gain for R820T/2 tuner.
 *                     exact values (R820T/2) are in range -47 .. 408 in tenth of a dB,
 *                       giving -4.7 .. +40.8 dB. these exact values may slightly change
 *                       with better measurements.
 *     10000 .. 10015: IF gain == VGA index from parameter if_mode
 *                     set if_mode by index: index := VGA_idx +10000
 *     10016 .. 10031: same as 10000 .. 10015, but additionally set automatic VGA
 *     10011:          for fixed VGA (=default) of -12 dB + 11 * 3.5 dB = 26.5 dB
 * 
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_tuner_if_mode(rtlsdr_dev_t *dev, int if_mode);

/*!
 * Set the sample rate for the device, also selects the baseband filters
 * according to the requested sample rate for tuners where this is possible.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param samp_rate the sample rate to be set, possible values are:
 * 		    225001 - 300000 Hz
 * 		    900001 - 3200000 Hz
 * 		    sample loss is to be expected for rates > 2400000
 * \return 0 on success, -EINVAL on invalid rate
 */
RTLSDR_API int rtlsdr_set_sample_rate(rtlsdr_dev_t *dev, uint32_t rate);

/*!
 * Get actual sample rate the device is configured to.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \return 0 on error, sample rate in Hz otherwise
 */
RTLSDR_API uint32_t rtlsdr_get_sample_rate(rtlsdr_dev_t *dev);

/*!
 * Enable test mode that returns an 8 bit counter instead of the samples.
 * The counter is generated inside the RTL2832.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param test mode, 1 means enabled, 0 disabled
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_testmode(rtlsdr_dev_t *dev, int on);

/*!
 * Enable or disable the internal digital AGC of the RTL2832.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param digital AGC mode, 1 means enabled, 0 disabled
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_agc_mode(rtlsdr_dev_t *dev, int on);

/*!
 * Enable or disable the direct sampling mode. When enabled, the IF mode
 * of the RTL2832 is activated, and rtlsdr_set_center_freq() will control
 * the IF-frequency of the DDC, which can be used to tune from 0 to 28.8 MHz
 * (xtal frequency of the RTL2832).
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param on 0 means disabled, 1 I-ADC input enabled, 2 Q-ADC input enabled
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_direct_sampling(rtlsdr_dev_t *dev, int on);

/*!
 * Get state of the direct sampling mode
 *
 * \param dev the device handle given by rtlsdr_open()
 * \return -1 on error, 0 means disabled, 1 I-ADC input enabled
 *	    2 Q-ADC input enabled
 */
RTLSDR_API int rtlsdr_get_direct_sampling(rtlsdr_dev_t *dev);

enum rtlsdr_ds_mode {
	RTLSDR_DS_IQ = 0,	/* I/Q quadrature sampling of tuner output */
	RTLSDR_DS_I,		/* 1: direct sampling on I branch: usually not connected */
	RTLSDR_DS_Q,		/* 2: direct sampling on Q branch: HF on rtl-sdr v3 dongle */
	RTLSDR_DS_I_BELOW,	/* 3: direct sampling on I branch when frequency below 'DS threshold frequency' */
	RTLSDR_DS_Q_BELOW	/* 4: direct sampling on Q branch when frequency below 'DS threshold frequency' */
};

/*!
 * Set direct sampling mode with threshold
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param mode static modes 0 .. 2 as in rtlsdr_set_direct_sampling(). other modes do automatic switching
 * \param freq_threshold direct sampling is used below this frequency, else quadrature mode through tuner
 *   set 0 for using default setting per tuner - not fully implemented yet!
 * \return negative on error, 0 on success
 */
RTLSDR_API int rtlsdr_set_ds_mode(rtlsdr_dev_t *dev, enum rtlsdr_ds_mode mode, uint32_t freq_threshold);

/*!
 * Enable or disable offset tuning for zero-IF tuners, which allows to avoid
 * problems caused by the DC offset of the ADCs and 1/f noise.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param on 0 means disabled, 1 enabled
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_offset_tuning(rtlsdr_dev_t *dev, int on);

/*!
 * Get state of the offset tuning mode
 *
 * \param dev the device handle given by rtlsdr_open()
 * \return -1 on error, 0 means disabled, 1 enabled
 */
RTLSDR_API int rtlsdr_get_offset_tuning(rtlsdr_dev_t *dev);

/*!
 * Enable or disable frequency dithering for r820t tuners.
 * Must be performed before freq_set().
 * Fails for other tuners.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param on 0 means disabled, 1 enabled
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_set_dithering(rtlsdr_dev_t *dev, int dither);

/* streaming functions */


/*!
 * Reset buffer in RTL2832
 *
 * \param dev the device handle given by rtlsdr_open()
 * \return 0 on success
 * \return -1 on error
 */
RTLSDR_API int rtlsdr_reset_buffer(rtlsdr_dev_t *dev);

/*!
 * Read data synchronously
 *
 * \param dev the device handle given by rtlsdr_open()
 * \return 0 on success
 * \return -1 on error or error code from libusb
 */
RTLSDR_API int rtlsdr_read_sync(rtlsdr_dev_t *dev, void *buf, int len, int *n_read);

typedef void(*rtlsdr_read_async_cb_t)(unsigned char *buf, uint32_t len, void *ctx);

/*!
 * Read samples from the device asynchronously. This function will block until
 * it is being canceled using rtlsdr_cancel_async()
 *
 * NOTE: This function is deprecated and is subject for removal.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param cb callback function to return received samples
 * \param ctx user specific context to pass via the callback function
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_wait_async(rtlsdr_dev_t *dev, rtlsdr_read_async_cb_t cb, void *ctx);

/*!
 * Read samples from the device asynchronously. This function will block until
 * it is being canceled using rtlsdr_cancel_async()
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param cb callback function to return received samples
 * \param ctx user specific context to pass via the callback function
 * \param buf_num optional buffer count, buf_num * buf_len = overall buffer size
 *		  set to 0 for default buffer count (15)
 * \param buf_len optional buffer length, must be multiple of 512,
 *		  should be a multiple of 16384 (URB size), set to 0
 *		  for default buffer length (16 * 32 * 512)
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_read_async(rtlsdr_dev_t *dev,
				 rtlsdr_read_async_cb_t cb,
				 void *ctx,
				 uint32_t buf_num,
				 uint32_t buf_len);

/*!
 * Cancel all pending asynchronous operations on the device.
 * Due to incomplete concurrency implementation, this should
 * only be called from within the callback function, so it is
 * in the correct thread.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \return 0 on success
 */
RTLSDR_API int rtlsdr_cancel_async(rtlsdr_dev_t *dev);

/*!
 * Read from the remote control (RC) infrared (IR) sensor
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param buf buffer to write IR signal (MSB=pulse/space, 7LSB=duration*20usec), recommended 128-bytes
 * \param buf_len size of buf
 * \return 0 if no signal, >0 number of bytes written into buf, <0 for error
 */
RTLSDR_API int rtlsdr_ir_query(rtlsdr_dev_t *dev, uint8_t *buf, size_t buf_len);


/*!
 * Enable or disable (the bias tee on) GPIO PIN 0 - if not reconfigured.
 * See rtlsdr_set_opt_string() option 'T'.
 * This works for rtl-sdr.com v3 dongles, see
 *   http://www.rtl-sdr.com/rtl-sdr-blog-v-3-dongles-user-guide/
 * Note: rtlsdr_close() does not clear GPIO lines,
 * so it leaves the (bias tee) line enabled if a client program
 * doesn't explictly disable it.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param on  1 for Bias T on. 0 for Bias T off.
 * \return -1 if device is not initialized. 0 otherwise.
 */
RTLSDR_API int rtlsdr_set_bias_tee(rtlsdr_dev_t *dev, int on);

/*!
 * Enable or disable (the bias tee on) the given GPIO pin.
 * Note: rtlsdr_close() does not clear GPIO lines,
 * so it leaves the (bias tee) lines enabled if a client program
 * doesn't explictly disable it.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param gpio the gpio pin -- assuming this line is connected to Bias T.
 *        gpio needs to be in 0 .. 7. BUT pin 4 is connected to Tuner RESET.
 *        and for FC0012 is already connected/reserved pin 6 for switching V/U-HF.
 * \param on  1 for Bias T on. 0 for Bias T off.
 * \return -1 if device is not initialized. 0 otherwise.
 */
RTLSDR_API int rtlsdr_set_bias_tee_gpio(rtlsdr_dev_t *dev, int gpio, int on);


/*
 * GPIO 0 .. 7 correspond to RTL2832U's pins as follows:
 * Pin information from http://lea.hamradio.si/~s57uuu/mischam/rtlsdr/ports.html
 *
 *   GPIO0 (=pin 37): BiasT for RTL-SDR.com V3
 *   GPIO1 (=pin 32): usually free - on RTL-SDR.com V3's Expansion Ports
 *   GPIO2 (=pin 31): usually free - on RTL-SDR.com V3's Expansion Ports
 *   GPIO3 (=pin 36): usually free
 *   GPIO4 (=pin 30): RESET for Tuners FC2580 and FC0012 /  on RTL-SDR.com V3's Expansion Ports
 *   GPIO5 (=pin 29): usually free - on RTL-SDR.com V3's Expansion Ports
 *   GPIO6 (=pin 22): select V-band/U-band filter for FC0012-Tuner
 *   GPIO7 (=pin 21): usually free
 *
 * CAUTION: The port pins use 3.3V logic levels and are static sensitive!
 *
 * most of following functions are also copied from Marko Cebokli's site
 * http://lea.hamradio.si/~s57uuu/mischam/rtlsdr/ports.html
 * but added return of error/status.
 * GPIO numbers 'gpio' in range 0 .. 7 for the following gpio functions
 * all \return negative values, on error
 */
RTLSDR_API int rtlsdr_set_gpio_output(rtlsdr_dev_t *dev, uint8_t gpio);
RTLSDR_API int rtlsdr_set_gpio_input(rtlsdr_dev_t *dev, uint8_t gpio);

RTLSDR_API int rtlsdr_set_gpio_bit(rtlsdr_dev_t *dev, uint8_t gpio, int val);
RTLSDR_API int rtlsdr_get_gpio_bit(rtlsdr_dev_t *dev, uint8_t gpio, int *val);

RTLSDR_API int rtlsdr_set_gpio_byte(rtlsdr_dev_t *dev, int val);
RTLSDR_API int rtlsdr_get_gpio_byte(rtlsdr_dev_t *dev, int *val);

RTLSDR_API int rtlsdr_set_gpio_status(rtlsdr_dev_t *dev, int *status );


/*!
 * Sets multiple options from a string encoded like "bw=300:agc=0:gain=27.3:dagc=0:T=1".
 * this is a helper function, that programs don't need to implement every single option
 *   at the command line interface.
 * Options are seperated by colon ':'.
 * There mustn't be extra spaces between option name and '='.
 * option 'f' set center frequency as in rtlsdr_set_center_freq()
 * option 'bw' sets tuner bandwidth as in rtlsdr_set_tuner_bandwidth()
 *   - but value is in kHz.
 * option 'agc' sets tuner gain mode as with rtlsdr_set_tuner_gain_mode():
 *   '1' means manual gain mode shall be enabled.
 * option 'gain' sets tuner gain as with rtlsdr_set_tuner_gain():
 *   values in tenth dB.
 * option 'dagc' or 'dgc' de/activates digital agc as with rtlsdr_set_agc_mode().
 *   value 1 to enable. 0 to disable.
 * option 'ds' set direct sampling as with rtlsdr_set_direct_sampling():
 *   '0' to deactivate, '1' or 'i' for I-ADC input, '2' or 'q' for Q-ADC input
 * option 't' or 'T' for enabling bias tee on GPIO PIN 0 as with rtlsdr_set_bias_tee():
 *   '1' for Bias T on. '0' for Bias T off.
 *
 * \param dev the device handle given by rtlsdr_open()
 * \param opts described option string
 * \param verbose print parsed options to stderr
 */
RTLSDR_API int rtlsdr_set_opt_string(rtlsdr_dev_t *dev, const char *opts, int verbose);

RTLSDR_API const char * rtlsdr_get_opt_help(int longInfo);


/*!
 * Exposes/permits hacking of Tuner-specific I2C registers: set register once
 *
 * \param dev           the device handle given by rtlsdr_open()
 * \param i2c_register  register address
 * \param mask          8-bit bitmask, indicating which bits shall be set
 * \param data          8-bit data, which shall be set
 * \return -1 if device is not initialized. 0 otherwise.
 */
RTLSDR_API int rtlsdr_set_tuner_i2c_register(rtlsdr_dev_t *dev, unsigned i2c_register, unsigned mask, unsigned data);

/* TODO: uint8_t */
RTLSDR_API int rtlsdr_get_tuner_i2c_register(rtlsdr_dev_t *dev, unsigned char* data, int len);


/*!
 * Exposes/permits hacking of Tuner-specific I2C registers: set and keep register for future
 *
 * \param dev           the device handle given by rtlsdr_open()
 * \param i2c_register  register address
 * \param mask          8-bit bitmask, indicating which bits shall be set
 * \param data          8-bit data, which shall be set; data in 0 .. 255 sets override; data > 255 clears override
 * \return -1 if device is not initialized. 0 otherwise.
 */
RTLSDR_API int rtlsdr_set_tuner_i2c_override(rtlsdr_dev_t *dev, unsigned i2c_register, unsigned mask, unsigned data);


/*!
 * request version id string to identify source and date of library
 *
 * \return pointer to C string, e.g. "github.com/librtlsdr" or "github.com/hayguen" or .. with build date (in parantheses)
 *   string keeps owned by library
 */
RTLSDR_API const char * rtlsdr_get_ver_id();

/*!
 * request version numbers of library
 *
 * \return major version in upper 16 bit, minor revision in lower 16 bit
 */
RTLSDR_API uint32_t rtlsdr_get_version();


#ifdef __cplusplus
}
#endif

#endif /* __RTL_SDR_H */
