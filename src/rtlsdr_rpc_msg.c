#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "rtlsdr_rpc_msg.h"

#if 1
#include <stdio.h>
#define PRINTF(__s, ...) fprintf(stderr, __s, ##__VA_ARGS__)
#define TRACE() PRINTF("[t] %s,%u\n", __FILE__, __LINE__)
#define ERROR() PRINTF("[e] %s,%u\n", __FILE__, __LINE__)
#else
#define TRACE()
#define ERROR()
#define PRINTF(...)
#endif


int rtlsdr_rpc_msg_init(rtlsdr_rpc_msg_t* msg, size_t data_size)
{
  size_t fmt_size;

  if (data_size == 0) data_size = 64;

  fmt_size = offsetof(rtlsdr_rpc_fmt_t, data) + data_size;
  msg->fmt = malloc(fmt_size);
  if (msg->fmt == NULL) return -1;

  msg->off = offsetof(rtlsdr_rpc_fmt_t, data);
  msg->size = fmt_size;

  return 0;
}

int rtlsdr_rpc_msg_fini(rtlsdr_rpc_msg_t* msg)
{
  free(msg->fmt);
  return 0;
}

void rtlsdr_rpc_msg_reset(rtlsdr_rpc_msg_t* msg)
{
  msg->off = offsetof(rtlsdr_rpc_fmt_t, data);
}

int rtlsdr_rpc_msg_realloc(rtlsdr_rpc_msg_t* msg, size_t size)
{
  uint8_t* new_fmt;

  if (msg->size >= size) return 0;

  new_fmt = malloc(size);
  if (new_fmt == NULL) return -1;

  memcpy(new_fmt, msg->fmt, msg->off);
  free(msg->fmt);
  msg->fmt = new_fmt;
  msg->size = size;

  return 0;
}

static int check_size(const rtlsdr_rpc_msg_t* msg, size_t size)
{
  if ((msg->off + size) > msg->size) return -1;
  return 0;
}

static int check_size_or_realloc(rtlsdr_rpc_msg_t* msg, size_t size)
{
  uint8_t* new_fmt;
  size_t new_size;

  if (check_size(msg, size) == 0) return 0;

  new_size = (msg->off + size + 256) & ~(256 - 1);
  new_fmt = malloc(new_size);
  if (new_fmt == NULL) return -1;

  memcpy(new_fmt, msg->fmt, msg->off);
  free(msg->fmt);

  msg->fmt = new_fmt;
  msg->size = new_size;

  return 0;
}

static int pop_uint32(rtlsdr_rpc_msg_t* msg, uint32_t* x)
{
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#error "unsupported endianness"
#endif

  if (check_size(msg, sizeof(uint32_t))) return -1;
  *x = *(const uint32_t*)(msg->fmt + msg->off);
  msg->off += sizeof(uint32_t);
  return 0;
}

static void push_mem_safe(rtlsdr_rpc_msg_t* msg, const uint8_t* x, size_t n)
{
  memcpy(msg->fmt + msg->off, x, n);
  msg->off += n;
}

static void push_uint32_safe(rtlsdr_rpc_msg_t* msg, uint32_t x)
{
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#error "unsupported endianness"
#endif

  push_mem_safe(msg, (const uint8_t*)&x, sizeof(uint32_t));
}

int rtlsdr_rpc_msg_push_int32(rtlsdr_rpc_msg_t* msg, int x)
{
  if (check_size_or_realloc(msg, sizeof(x))) return -1;
  push_uint32_safe(msg, (uint32_t)x);
  return 0;
}

int rtlsdr_rpc_msg_push_uint32(rtlsdr_rpc_msg_t* msg, uint32_t x)
{
  if (check_size_or_realloc(msg, sizeof(x))) return -1;
  push_uint32_safe(msg, x);
  return 0;
}

void rtlsdr_rpc_msg_push_uint32_safe(rtlsdr_rpc_msg_t* msg, uint32_t x)
{
  push_uint32_safe(msg, x);
}

int rtlsdr_rpc_msg_push_str(rtlsdr_rpc_msg_t* msg, const char* s)
{
  if (check_size_or_realloc(msg, strlen(s) + 1)) return -1;
  push_mem_safe(msg, (const uint8_t*)s, strlen(s) + 1);
  return 0;
}

int rtlsdr_rpc_msg_push_buf(rtlsdr_rpc_msg_t* msg, const uint8_t* buf, size_t size)
{
  size_t total_size = sizeof(uint32_t) + size;
  if (check_size_or_realloc(msg, total_size)) return -1;
  push_uint32_safe(msg, (uint32_t)size);
  push_mem_safe(msg, buf, size);
  return 0;
}

void rtlsdr_rpc_msg_skip_safe(rtlsdr_rpc_msg_t* msg, size_t size)
{
  msg->off += size;
}

int rtlsdr_rpc_msg_pop_int32(rtlsdr_rpc_msg_t* msg, int32_t* x)
{
  return pop_uint32(msg, (uint32_t*)x);
}

int rtlsdr_rpc_msg_pop_uint32(rtlsdr_rpc_msg_t* msg, uint32_t* x)
{
  return pop_uint32(msg, x);
}

int rtlsdr_rpc_msg_pop_str(rtlsdr_rpc_msg_t* msg, const char** s)
{
  size_t i;

  *s = (const char*)(msg->fmt + msg->off);

  for (i = msg->off; i != msg->size; ++i)
  {
    if (msg->fmt[i] == 0)
    {
      msg->off = i + 1;
      return 0;
    }
  }

  return -1;
}

int rtlsdr_rpc_msg_pop_buf
(rtlsdr_rpc_msg_t* msg, const uint8_t** buf, size_t* size)
{
  uint32_t x;

  if (pop_uint32(msg, &x)) return -1;
  if ((msg->off + x) > msg->size) return -1;

  *buf = (const uint8_t*)(msg->fmt + msg->off);
  msg->off += x;

  *size = (size_t)x;

  return 0;
}

static void put_uint8(void* p, uint8_t x)
{
  memcpy(p, (const void*)&x, sizeof(x));
}

static void put_uint16(void* p, uint16_t x)
{
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#error "unsupported endianness"
#endif

  memcpy(p, (const void*)&x, sizeof(x));
}

static void put_uint32(void* p, uint32_t x)
{
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#error "unsupported endianness"
#endif

  memcpy(p, (const void*)&x, sizeof(x));
}

static uint8_t get_uint8(const void* p)
{
  uint8_t x;
  memcpy((void*)&x, p, sizeof(x));
  return x;
}

static uint16_t get_uint16(const void* p)
{
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#error "unsupported endianness"
#endif

  uint16_t x;
  memcpy((void*)&x, p, sizeof(x));
  return x;
}

static uint32_t get_uint32(const void* p)
{
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#error "unsupported endianness"
#endif

  uint32_t x;
  memcpy((void*)&x, p, sizeof(x));
  return x;
}

void rtlsdr_rpc_msg_set_size(rtlsdr_rpc_msg_t* msg, size_t size)
{
  rtlsdr_rpc_fmt_t* const fmt = (rtlsdr_rpc_fmt_t*)msg->fmt;
  put_uint32(&fmt->size, (uint32_t)size);
}

size_t rtlsdr_rpc_msg_get_size(const rtlsdr_rpc_msg_t* msg)
{
  const rtlsdr_rpc_fmt_t* const fmt = (const rtlsdr_rpc_fmt_t*)msg->fmt;
  return (size_t)get_uint32(&fmt->size);
}

void rtlsdr_rpc_msg_set_op(rtlsdr_rpc_msg_t* msg, rtlsdr_rpc_op_t op)
{
  rtlsdr_rpc_fmt_t* const fmt = (rtlsdr_rpc_fmt_t*)msg->fmt;
  put_uint8(&fmt->op, (uint8_t)op);
}

rtlsdr_rpc_op_t rtlsdr_rpc_msg_get_op(const rtlsdr_rpc_msg_t* msg)
{
  const rtlsdr_rpc_fmt_t* const fmt = (const rtlsdr_rpc_fmt_t*)msg->fmt;
  return (rtlsdr_rpc_op_t)get_uint8(&fmt->op);
}

void rtlsdr_rpc_msg_set_id(rtlsdr_rpc_msg_t* msg, uint8_t id)
{
  rtlsdr_rpc_fmt_t* const fmt = (rtlsdr_rpc_fmt_t*)msg->fmt;
  put_uint16(&fmt->id, id);
}

uint8_t rtlsdr_rpc_msg_get_id(const rtlsdr_rpc_msg_t* msg)
{
  const rtlsdr_rpc_fmt_t* const fmt = (const rtlsdr_rpc_fmt_t*)msg->fmt;
  return get_uint8(&fmt->id);
}

void rtlsdr_rpc_msg_set_err(rtlsdr_rpc_msg_t* msg, int err)
{
  rtlsdr_rpc_fmt_t* const fmt = (rtlsdr_rpc_fmt_t*)msg->fmt;
  put_uint32(&fmt->err, (uint32_t)err);
}

int rtlsdr_rpc_msg_get_err(const rtlsdr_rpc_msg_t* msg)
{
  const rtlsdr_rpc_fmt_t* const fmt = (const rtlsdr_rpc_fmt_t*)msg->fmt;
  return (int)get_uint32(&fmt->err);
}
