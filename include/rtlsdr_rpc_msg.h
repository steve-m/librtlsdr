#ifndef RTLSDR_RPC_MSG_H_INCLUDED
#define RTLSDR_RPC_MSG_H_INCLUDED


#include <stdint.h>
#include <sys/types.h>

typedef enum
{
  RTLSDR_RPC_OP_GET_DEVICE_COUNT = 0,
  RTLSDR_RPC_OP_GET_DEVICE_NAME,
  RTLSDR_RPC_OP_GET_DEVICE_USB_STRINGS,
  RTLSDR_RPC_OP_GET_INDEX_BY_SERIAL,
  RTLSDR_RPC_OP_OPEN,
  RTLSDR_RPC_OP_CLOSE,
  RTLSDR_RPC_OP_SET_XTAL_FREQ,
  RTLSDR_RPC_OP_GET_XTAL_FREQ,
  RTLSDR_RPC_OP_GET_USB_STRINGS,
  RTLSDR_RPC_OP_WRITE_EEPROM,
  RTLSDR_RPC_OP_READ_EEPROM,
  RTLSDR_RPC_OP_SET_CENTER_FREQ,
  RTLSDR_RPC_OP_GET_CENTER_FREQ,
  RTLSDR_RPC_OP_SET_FREQ_CORRECTION,
  RTLSDR_RPC_OP_GET_FREQ_CORRECTION,
  RTLSDR_RPC_OP_GET_TUNER_TYPE,
  RTLSDR_RPC_OP_GET_TUNER_GAINS,
  RTLSDR_RPC_OP_SET_TUNER_GAIN,
  RTLSDR_RPC_OP_GET_TUNER_GAIN,
  RTLSDR_RPC_OP_SET_TUNER_IF_GAIN,
  RTLSDR_RPC_OP_SET_TUNER_GAIN_MODE,
  RTLSDR_RPC_OP_SET_SAMPLE_RATE,
  RTLSDR_RPC_OP_GET_SAMPLE_RATE,
  RTLSDR_RPC_OP_SET_TESTMODE,
  RTLSDR_RPC_OP_SET_AGC_MODE,
  RTLSDR_RPC_OP_SET_DIRECT_SAMPLING,
  RTLSDR_RPC_OP_GET_DIRECT_SAMPLING,
  RTLSDR_RPC_OP_SET_OFFSET_TUNING,
  RTLSDR_RPC_OP_GET_OFFSET_TUNING,
  RTLSDR_RPC_OP_RESET_BUFFER,
  RTLSDR_RPC_OP_READ_SYNC,
  RTLSDR_RPC_OP_WAIT_ASYNC,
  RTLSDR_RPC_OP_READ_ASYNC,
  RTLSDR_RPC_OP_CANCEL_ASYNC,

  /* non api operations */
  RTLSDR_RPC_OP_EVENT_STATE,
 
  RTLSDR_RPC_OP_INVALID
} rtlsdr_rpc_op_t;

typedef struct
{
  /* raw network format */
  uint32_t size;
  uint8_t op;
  uint8_t id;
  uint32_t err;
  uint8_t data[1];
} __attribute__((packed)) rtlsdr_rpc_fmt_t;

typedef struct
{
  size_t off;
  size_t size;
  uint8_t* fmt;
} rtlsdr_rpc_msg_t;

int rtlsdr_rpc_msg_init(rtlsdr_rpc_msg_t*, size_t);
int rtlsdr_rpc_msg_fini(rtlsdr_rpc_msg_t*);
void rtlsdr_rpc_msg_reset(rtlsdr_rpc_msg_t*);
int rtlsdr_rpc_msg_realloc(rtlsdr_rpc_msg_t*, size_t);

void rtlsdr_rpc_msg_set_size(rtlsdr_rpc_msg_t*, size_t);
size_t rtlsdr_rpc_msg_get_size(const rtlsdr_rpc_msg_t*);
void rtlsdr_rpc_msg_set_op(rtlsdr_rpc_msg_t*, rtlsdr_rpc_op_t);
rtlsdr_rpc_op_t rtlsdr_rpc_msg_get_op(const rtlsdr_rpc_msg_t*);
void rtlsdr_rpc_msg_set_id(rtlsdr_rpc_msg_t*, uint8_t);
uint8_t rtlsdr_rpc_msg_get_id(const rtlsdr_rpc_msg_t*);
void rtlsdr_rpc_msg_set_err(rtlsdr_rpc_msg_t*, int);
int rtlsdr_rpc_msg_get_err(const rtlsdr_rpc_msg_t*);

int rtlsdr_rpc_msg_push_int32(rtlsdr_rpc_msg_t*, int32_t);
int rtlsdr_rpc_msg_push_uint32(rtlsdr_rpc_msg_t*, uint32_t);
void rtlsdr_rpc_msg_push_uint32_safe(rtlsdr_rpc_msg_t*, uint32_t);
int rtlsdr_rpc_msg_push_str(rtlsdr_rpc_msg_t*, const char*);
int rtlsdr_rpc_msg_push_buf(rtlsdr_rpc_msg_t*, const uint8_t*, size_t);
void rtlsdr_rpc_msg_skip_safe(rtlsdr_rpc_msg_t*, size_t);
int rtlsdr_rpc_msg_pop_int32(rtlsdr_rpc_msg_t*, int32_t*);
int rtlsdr_rpc_msg_pop_uint32(rtlsdr_rpc_msg_t*, uint32_t*);
int rtlsdr_rpc_msg_pop_str(rtlsdr_rpc_msg_t*, const char**);
int rtlsdr_rpc_msg_pop_buf(rtlsdr_rpc_msg_t*, const uint8_t**, size_t*);


#endif /* RTLSDR_RPC_MSG_H_INCLUDED */
