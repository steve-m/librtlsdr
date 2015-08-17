#ifndef RTLSDR_RPC_H_INCLUDED
#define RTLSDR_RPC_H_INCLUDED


#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef void (*rtlsdr_rpc_read_async_cb_t)
(unsigned char*, uint32_t, void*);

uint32_t rtlsdr_rpc_get_device_count(void);

const char* rtlsdr_rpc_get_device_name
(uint32_t nidex);

int rtlsdr_rpc_get_device_usb_strings
(uint32_t index, char* manufact, char* product, char* serial);

int rtlsdr_rpc_get_index_by_serial
(const char* serial);

int rtlsdr_rpc_open
(void** dev, uint32_t index);

int rtlsdr_rpc_close
(void* dev);

int rtlsdr_rpc_set_xtal_freq
(void* dev, uint32_t rtl_freq, uint32_t tuner_freq);

int rtlsdr_rpc_get_xtal_freq
(void* dev, uint32_t* rtl_freq, uint32_t* tuner_freq);

int rtlsdr_rpc_get_usb_strings
(void* dev, char* manufact, char* product, char* serial);

int rtlsdr_rpc_write_eeprom
(void* dev, uint8_t* data, uint8_t offset, uint16_t len);

int rtlsdr_rpc_read_eeprom
(void* dev, uint8_t* data, uint8_t offset, uint16_t len);

int rtlsdr_rpc_set_center_freq
(void* dev, uint32_t freq);

uint32_t rtlsdr_rpc_get_center_freq
(void* dev);

int rtlsdr_rpc_set_freq_correction
(void* dev, int ppm);

int rtlsdr_rpc_get_freq_correction
(void *dev);

int rtlsdr_rpc_get_tuner_type
(void* dev);

int rtlsdr_rpc_get_tuner_gains
(void* dev, int* gainsp);

int rtlsdr_rpc_set_tuner_gain
(void *dev, int gain);

int rtlsdr_rpc_get_tuner_gain
(void* dev);

int rtlsdr_rpc_set_tuner_if_gain
(void* dev, int stage, int gain);

int rtlsdr_rpc_set_tuner_gain_mode
(void* dev, int manual);

int rtlsdr_rpc_set_sample_rate
(void* dev, uint32_t rate);

uint32_t rtlsdr_rpc_get_sample_rate
(void* dev);

int rtlsdr_rpc_set_testmode
(void* dev, int on);

int rtlsdr_rpc_set_agc_mode
(void* dev, int on);

int rtlsdr_rpc_set_direct_sampling
(void* dev, int on);

int rtlsdr_rpc_get_direct_sampling
(void* dev);

int rtlsdr_rpc_set_offset_tuning
(void* dev, int on);

int rtlsdr_rpc_get_offset_tuning
(void* dev);

int rtlsdr_rpc_reset_buffer
(void* dev);

int rtlsdr_rpc_read_sync
(void* dev, void* buf, int len, int* n_read);

int rtlsdr_rpc_wait_async
(void* dev, rtlsdr_rpc_read_async_cb_t cb, void* ctx);

int rtlsdr_rpc_read_async
(void* dev, rtlsdr_rpc_read_async_cb_t cb, void* ctx, uint32_t buf_num, uint32_t buf_len);

int rtlsdr_rpc_cancel_async
(void* dev);

unsigned int rtlsdr_rpc_is_enabled(void);

#ifdef __cplusplus
}
#endif


#endif /* RTLSDR_RPC_H_INCLUDED */
