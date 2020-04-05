#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/fcntl.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <pthread.h>
#include <rtl-sdr.h>
#include "rtlsdr_rpc_msg.h"


#if 1
#include <stdio.h>
#define PRINTF(__s, ...)			\
do {						\
  fprintf(stderr, __s, ##__VA_ARGS__);		\
  fflush(stderr);				\
} while (0)
#define TRACE() PRINTF("[t] %s,%u\n", __FILE__, __LINE__)
#define ERROR() PRINTF("[e] %s,%u\n", __FILE__, __LINE__)
#else
#define TRACE()
#define ERROR()
#define PRINTF(...)
#endif


typedef struct
{
  int listen_sock;
  int cli_sock;

  rtlsdr_dev_t* dev;
  uint32_t did;

  rtlsdr_rpc_msg_t query_msg;
  rtlsdr_rpc_msg_t reply_msg;
  rtlsdr_rpc_msg_t event_msg;

  unsigned int async_replied;
  uint8_t async_id;

  int port_ir;
  const char *addr_ir;
  int wait_ir;
} rpcd_t;

static int resolve_ip_addr
(
 struct sockaddr_storage saddr_both[2], size_t size_both[2],
 const char* addr, const char* port
)
{
  struct addrinfo ai;
  struct addrinfo* aip = NULL;
  int err = -1;
  size_t i;

  memset(&ai, 0, sizeof(ai));
  ai.ai_family = AF_UNSPEC;
  ai.ai_socktype = SOCK_STREAM;
  ai.ai_flags = AI_PASSIVE;

  if (getaddrinfo(addr, port, &ai, &aip)) goto on_error;

  size_both[0] = 0;
  size_both[1] = 0;
  i = 0;
  for (; (i != 2) && (aip != NULL); aip = aip->ai_next)
  {
    if ((aip->ai_family != AF_INET) && (aip->ai_family != AF_INET6)) continue ;
    if (aip->ai_addrlen == 0) continue ;
    memcpy(&saddr_both[i], aip->ai_addr, aip->ai_addrlen);
    size_both[i] = aip->ai_addrlen;
    ++i;
  }

  if (i == 0) goto on_error;

  err = 0;
 on_error:
  if (aip != NULL) freeaddrinfo(aip);
  return err;
}

static int open_nonblock_socket
(
 struct sockaddr_storage saddr_both[2], size_t size_both[2],
 int type, int proto,
 struct sockaddr_storage** saddr_used, size_t* size_used
)
{
  size_t i;
  int fd = -1;

  for (i = 0; (i != 2) && (size_both[i]); ++i)
  {
    const struct sockaddr* const sa = (const struct sockaddr*)&saddr_both[i];
    fd = socket(sa->sa_family, type, proto);
    if (fd != -1) break ;
  }

  if ((i == 2) || (size_both[i] == 0)) return -1;

  *saddr_used = &saddr_both[i];
  *size_used = size_both[i];

  if (fcntl(fd, F_SETFL, O_NONBLOCK))
  {
    close(fd);
    return -1;
  }

  return fd;
}

static int init_rpcd(rpcd_t* rpcd, const char* addr, const char* port)
{
  struct sockaddr_storage saddrs[2];
  struct sockaddr_storage* saddr;
  size_t sizes[2];
  size_t size;
  int err;
  int enable = 1;

  /* TODO: handle errors */
  rtlsdr_rpc_msg_init(&rpcd->query_msg, 0);
  rtlsdr_rpc_msg_init(&rpcd->reply_msg, 0);
  rtlsdr_rpc_msg_init(&rpcd->event_msg, 0);

  if (resolve_ip_addr(saddrs, sizes, addr, port))
  {
    ERROR();
    goto on_error_0;
  }

  rpcd->listen_sock = open_nonblock_socket
    (saddrs, sizes, SOCK_STREAM, IPPROTO_TCP, &saddr, &size);
  if (rpcd->listen_sock == -1)
  {
    ERROR();
    goto on_error_0;
  }

  err = setsockopt
    (rpcd->listen_sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
  if (err)
  {
    ERROR();
    goto on_error_1;
  }

  err = bind
    (rpcd->listen_sock, (const struct sockaddr*)saddr, (socklen_t)size);
  if (err)
  {
    ERROR();
    goto on_error_1;
  }

  if (listen(rpcd->listen_sock, 5))
  {
    ERROR();
    goto on_error_1;
  }

  rpcd->cli_sock = -1;
  rpcd->dev = NULL;

  return 0;

 on_error_1:
  close(rpcd->listen_sock);
 on_error_0:
  return -1;
}

static int fini_rpcd(rpcd_t* rpcd)
{
  if (rpcd->cli_sock != -1)
  {
    shutdown(rpcd->cli_sock, SHUT_RDWR);
    close(rpcd->cli_sock);
  }

  shutdown(rpcd->listen_sock, SHUT_RDWR);
  close(rpcd->listen_sock);

  rtlsdr_rpc_msg_fini(&rpcd->query_msg);
  rtlsdr_rpc_msg_fini(&rpcd->reply_msg);
  rtlsdr_rpc_msg_fini(&rpcd->event_msg);

  return 0;
}

static int recv_all(int fd, uint8_t* buf, size_t size)
{
  ssize_t n;
  fd_set rset;

  while (1)
  {
    errno = 0;
    n = recv(fd, buf, size, 0);
    if (n <= 0)
    {
      if ((errno != EWOULDBLOCK) || (errno != EAGAIN))
	return -1;
    }
    else
    {
      size -= (size_t)n;
      buf += (size_t)n;
      if (size == 0) break ;
    }

    FD_ZERO(&rset);
    FD_SET(fd, &rset);
    if (select(fd + 1, &rset, NULL, NULL, NULL) <= 0)
    {
      return -1;
    }
  }

  return 0;
}

static int send_all_check_recv
(int fd, const uint8_t* buf, size_t size, unsigned int* is_recv)
{
  ssize_t n;
  fd_set wset;
  fd_set rset;
  fd_set* rsetp;

  rsetp = NULL;
  if (is_recv != NULL) *is_recv = 0;

  while (1)
  {
    FD_ZERO(&wset);
    FD_SET(fd, &wset);

    rsetp = NULL;
    if ((is_recv != NULL) && (*is_recv == 0))
    {
      FD_ZERO(&rset);
      FD_SET(fd, &rset);
      rsetp = &rset;
    }

    if (select(fd + 1, rsetp, &wset, NULL, NULL) <= 0)
    {
      return -1;
    }

    if ((rsetp != NULL) && FD_ISSET(fd, rsetp))
    {
      *is_recv = 1;
    }

    if (FD_ISSET(fd, &wset))
    {
      errno = 0;
      n = send(fd, buf, size, 0);
      if (n <= 0)
      {
	if ((errno != EWOULDBLOCK) || (errno != EAGAIN))
	  return -1;
      }
      else
      {
	size -= (size_t)n;
	buf += (size_t)n;
	if (size == 0) break ;
      }
    }
  }

  return 0;
}

static int send_all(int fd, const uint8_t* buf, size_t size)
{
  return send_all_check_recv(fd, buf, size, NULL);
}

static int recv_msg(int fd, rtlsdr_rpc_msg_t* m)
{
  uint32_t size;

  if (recv_all(fd, (uint8_t*)&size, sizeof(uint32_t)))
  {
    ERROR();
    return -1;
  }

  rtlsdr_rpc_msg_set_size(m, size);
  size = rtlsdr_rpc_msg_get_size(m);

  if (rtlsdr_rpc_msg_realloc(m, size))
  {
    ERROR();
    return -1;
  }

  if (recv_all(fd, m->fmt + sizeof(uint32_t), size - sizeof(uint32_t)))
  {
    ERROR();
    return -1;
  }

  return 0;
}

static int send_msg(int fd, rtlsdr_rpc_msg_t* m)
{
  return send_all(fd, m->fmt, m->off);
}

static int recv_query(rpcd_t* rpcd, rtlsdr_rpc_msg_t** q)
{
  *q = NULL;
  rtlsdr_rpc_msg_reset(&rpcd->query_msg);
  if (recv_msg(rpcd->cli_sock, &rpcd->query_msg)) return -1;
  *q = &rpcd->query_msg;
  return 0;
}

static int send_reply(rpcd_t* rpcd, rtlsdr_rpc_msg_t* r)
{
  return send_msg(rpcd->cli_sock, r);
}

static void read_async_cb
(unsigned char* buf, uint32_t len, void* ctx)
{
  const size_t off = offsetof(rtlsdr_rpc_fmt_t, data);

  rpcd_t* const rpcd = ctx;
  uint8_t fmt[offsetof(rtlsdr_rpc_fmt_t, data)];
  rtlsdr_rpc_msg_t msg;
  unsigned int is_recv;

  if (rpcd->async_replied == 0)
  {
    send_reply(rpcd, &rpcd->reply_msg);
    rpcd->async_replied = 1;
  }

  msg.off = off;
  msg.size = off + len;
  msg.fmt = fmt;
  rtlsdr_rpc_msg_set_size(&msg, msg.size);
  rtlsdr_rpc_msg_set_op(&msg, RTLSDR_RPC_OP_READ_ASYNC);
  rtlsdr_rpc_msg_set_id(&msg, rpcd->async_id);
  rtlsdr_rpc_msg_set_err(&msg, 0);

  send_all(rpcd->cli_sock, fmt, off);
  send_all_check_recv(rpcd->cli_sock, buf, len, &is_recv);

  if (is_recv) rtlsdr_cancel_async(rpcd->dev);
}

__attribute__((unused))
static const char* op_to_string(rtlsdr_rpc_op_t op)
{
  const char* const s[] =
  {
    "RTLSDR_RPC_OP_GET_DEVICE_COUNT",
    "RTLSDR_RPC_OP_GET_DEVICE_NAME",
    "RTLSDR_RPC_OP_GET_DEVICE_USB_STRINGS",
    "RTLSDR_RPC_OP_GET_INDEX_BY_SERIAL",
    "RTLSDR_RPC_OP_OPEN",
    "RTLSDR_RPC_OP_CLOSE",
    "RTLSDR_RPC_OP_SET_XTAL_FREQ",
    "RTLSDR_RPC_OP_GET_XTAL_FREQ",
    "RTLSDR_RPC_OP_GET_USB_STRINGS",
    "RTLSDR_RPC_OP_WRITE_EEPROM",
    "RTLSDR_RPC_OP_READ_EEPROM",
    "RTLSDR_RPC_OP_SET_CENTER_FREQ",
    "RTLSDR_RPC_OP_GET_CENTER_FREQ",
    "RTLSDR_RPC_OP_SET_FREQ_CORRECTION",
    "RTLSDR_RPC_OP_GET_FREQ_CORRECTION",
    "RTLSDR_RPC_OP_GET_TUNER_TYPE",
    "RTLSDR_RPC_OP_GET_TUNER_GAINS",
    "RTLSDR_RPC_OP_SET_TUNER_GAIN",
    "RTLSDR_RPC_OP_GET_TUNER_GAIN",
    "RTLSDR_RPC_OP_SET_TUNER_IF_GAIN",
    "RTLSDR_RPC_OP_SET_TUNER_GAIN_MODE",
    "RTLSDR_RPC_OP_SET_GET_TUNER_BW",
    "RTLSDR_RPC_OP_SET_SAMPLE_RATE",
    "RTLSDR_RPC_OP_GET_SAMPLE_RATE",
    "RTLSDR_RPC_OP_SET_TESTMODE",
    "RTLSDR_RPC_OP_SET_AGC_MODE",
    "RTLSDR_RPC_OP_SET_DIRECT_SAMPLING",
    "RTLSDR_RPC_OP_GET_DIRECT_SAMPLING",
    "RTLSDR_RPC_OP_SET_OFFSET_TUNING",
    "RTLSDR_RPC_OP_GET_OFFSET_TUNING",
    "RTLSDR_RPC_OP_RESET_BUFFER",
    "RTLSDR_RPC_OP_READ_SYNC",
    "RTLSDR_RPC_OP_WAIT_ASYNC",
    "RTLSDR_RPC_OP_READ_ASYNC",
    "RTLSDR_RPC_OP_CANCEL_ASYNC",
    "RTLSDR_RPC_OP_EVENT_STATE",
    "RTLSDR_RPC_OP_INVALID"
  };
  if (op >= RTLSDR_RPC_OP_INVALID) op = RTLSDR_RPC_OP_INVALID;
  return s[op];
}

static int handle_query
(rpcd_t* rpcd, rtlsdr_rpc_msg_t* q, rtlsdr_rpc_msg_t** rr)
{
  rtlsdr_rpc_msg_t* const r = &rpcd->reply_msg;
  rtlsdr_rpc_op_t op;
  int err = -1;

  *rr = NULL;

  rtlsdr_rpc_msg_reset(r);

  op = rtlsdr_rpc_msg_get_op(q);
  switch (op)
  {
  case RTLSDR_RPC_OP_GET_DEVICE_COUNT:
    {
      uint32_t n;

      n = rtlsdr_get_device_count();
      if (rtlsdr_rpc_msg_push_uint32(r, n)) goto on_error;
      err = 0;
      break ;
    }

  case RTLSDR_RPC_OP_GET_DEVICE_NAME:
    {
      const char* s;
      uint32_t i;

      if (rtlsdr_rpc_msg_pop_uint32(q, &i)) goto on_error;

      s = rtlsdr_get_device_name(i);
      if (s == NULL) s = "";

      if (rtlsdr_rpc_msg_push_str(r, s)) goto on_error;

      err = 0;

      break ;
    }

  case RTLSDR_RPC_OP_GET_DEVICE_USB_STRINGS:
    {
      char manuf[256];
      char product[256];
      char serial[256];
      uint32_t i;

      if (rtlsdr_rpc_msg_pop_uint32(q, &i)) goto on_error;

      err = rtlsdr_get_device_usb_strings(i, manuf, product, serial);
      if (err) goto on_error;

      if (rtlsdr_rpc_msg_push_str(r, manuf))
      {
	err = -1;
	goto on_error;
      }

      if (rtlsdr_rpc_msg_push_str(r, product))
      {
	err = -1;
	goto on_error;
      }

      if (rtlsdr_rpc_msg_push_str(r, serial))
      {
	err = -1;
	goto on_error;
      }

      break ;
    }

  case RTLSDR_RPC_OP_GET_INDEX_BY_SERIAL:
    {
      const char* serial;

      if (rtlsdr_rpc_msg_pop_str(q, &serial)) goto on_error;
      err = rtlsdr_get_index_by_serial(serial);
      if (err < 0) goto on_error;

      break ;
    }

  case RTLSDR_RPC_OP_OPEN:
    {
      if (rpcd->dev != NULL)
      {
	/* already opened */
	err = 0;
	goto on_error;
      }

      if (rtlsdr_rpc_msg_pop_uint32(q, &rpcd->did)) goto on_error;
      err = rtlsdr_open(&rpcd->dev, rpcd->did);
      if (err)
      {
	rpcd->dev = NULL;
	goto on_error;
      }

      break ;
    }

  case RTLSDR_RPC_OP_CLOSE:
    {
      uint32_t did;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;
      err = rtlsdr_close(rpcd->dev);
      rpcd->dev = NULL;

      break ;
    }

  case RTLSDR_RPC_OP_SET_XTAL_FREQ:
    {
      uint32_t did;
      uint32_t rtl_freq;
      uint32_t tuner_freq;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &rtl_freq)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &tuner_freq)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_set_xtal_freq(rpcd->dev, rtl_freq, tuner_freq);
      if (err) goto on_error;

      break ;
    }

  case RTLSDR_RPC_OP_GET_XTAL_FREQ:
    {
      uint32_t did;
      uint32_t rtl_freq;
      uint32_t tuner_freq;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_get_xtal_freq(rpcd->dev, &rtl_freq, &tuner_freq);
      if (err) goto on_error;

      if (rtlsdr_rpc_msg_push_uint32(r, rtl_freq))
      {
	err = -1;
	goto on_error;
      }

      if (rtlsdr_rpc_msg_push_uint32(r, tuner_freq))
      {
	err = -1;
	goto on_error;
      }

      break ;
    }

  case RTLSDR_RPC_OP_GET_USB_STRINGS:
    {
      uint32_t did;
      char manuf[256];
      char product[256];
      char serial[256];

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_get_usb_strings(rpcd->dev, manuf, product, serial);
      if (err) goto on_error;

      if (rtlsdr_rpc_msg_push_str(r, manuf))
      {
	err = -1;
	goto on_error;
      }

      if (rtlsdr_rpc_msg_push_str(r, product))
      {
	err = -1;
	goto on_error;
      }

      if (rtlsdr_rpc_msg_push_str(r, serial))
      {
	err = -1;
	goto on_error;
      }

      break ;
    }

  case RTLSDR_RPC_OP_SET_CENTER_FREQ:
    {
      uint32_t did;
      uint32_t freq;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &freq)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_set_center_freq(rpcd->dev, freq);
      if (err) goto on_error;

      break ;
    }

  case RTLSDR_RPC_OP_GET_CENTER_FREQ:
    {
      uint32_t did;
      uint32_t freq;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      freq = rtlsdr_get_center_freq(rpcd->dev);
      if (freq == 0) goto on_error;
      if (rtlsdr_rpc_msg_push_uint32(r, freq)) goto on_error;
      err = 0;

      break ;
    }

  case RTLSDR_RPC_OP_SET_FREQ_CORRECTION:
    {
      uint32_t did;
      uint32_t ppm;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &ppm)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_set_freq_correction(rpcd->dev, (int)ppm);
      if (err) goto on_error;

      break ;
    }

  case RTLSDR_RPC_OP_GET_FREQ_CORRECTION:
    {
      uint32_t did;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_get_freq_correction(rpcd->dev);

      break ;
    }

  case RTLSDR_RPC_OP_GET_TUNER_TYPE:
    {
      uint32_t did;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_get_tuner_type(rpcd->dev);

      break ;
    }

  case RTLSDR_RPC_OP_GET_TUNER_GAINS:
    {
      uint32_t did;
      uint32_t is_null;
      uint32_t gain_size;
      size_t new_size;
      uint8_t* buf;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &is_null)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &gain_size)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      if (is_null)
      {
	err = rtlsdr_get_tuner_gains(rpcd->dev, NULL);
	if (err <= 0) goto on_error;
      }
      else
      {
	new_size = r->off + sizeof(uint32_t) + gain_size;
	if (rtlsdr_rpc_msg_realloc(r, new_size)) goto on_error;
	buf = r->fmt + r->off + sizeof(uint32_t);

	err = rtlsdr_get_tuner_gains(rpcd->dev, (int*)buf);
	if (err <= 0) goto on_error;

	rtlsdr_rpc_msg_push_uint32_safe(r, gain_size);
	rtlsdr_rpc_msg_skip_safe(r, gain_size);
      }

      break ;
    }

  case RTLSDR_RPC_OP_SET_TUNER_GAIN:
    {
      uint32_t did;
      uint32_t gain;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &gain)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_set_tuner_gain(rpcd->dev, (int)gain);
      if (err) goto on_error;

      break ;
    }

  case RTLSDR_RPC_OP_GET_TUNER_GAIN:
    {
      uint32_t did;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_get_tuner_gain(rpcd->dev);

      break ;
    }

  case RTLSDR_RPC_OP_SET_TUNER_IF_GAIN:
    {
      uint32_t did;
      uint32_t stage;
      uint32_t gain;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &stage)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &gain)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_set_tuner_if_gain(rpcd->dev, (int)stage, (int)gain);
      if (err) goto on_error;

      break ;
    }

  case RTLSDR_RPC_OP_SET_TUNER_GAIN_MODE:
    {
      uint32_t did;
      uint32_t manual;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &manual)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_set_tuner_gain_mode(rpcd->dev, (int)manual);
      if (err) goto on_error;

      break ;
    }

  case RTLSDR_RPC_OP_SET_GET_TUNER_BW:
    {
      uint32_t did;
      uint32_t bw;
      uint32_t applied_bw;
      int apply_bw;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &bw)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &apply_bw)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_set_and_get_tuner_bandwidth(rpcd->dev, bw, &applied_bw, apply_bw);
      if (err) goto on_error;
      if (rtlsdr_rpc_msg_push_uint32(r, applied_bw)) goto on_error;

      break ;
    }

  case RTLSDR_RPC_OP_SET_SAMPLE_RATE:
    {
      uint32_t did;
      uint32_t rate;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &rate)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_set_sample_rate(rpcd->dev, rate);
      if (err) goto on_error;

      break ;
    }

  case RTLSDR_RPC_OP_GET_SAMPLE_RATE:
    {
      uint32_t did;
      uint32_t rate;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      rate = rtlsdr_get_sample_rate(rpcd->dev);
      if (rate == 0) goto on_error;
      if (rtlsdr_rpc_msg_push_uint32(r, rate)) goto on_error;
      err = 0;

      break ;
    }

  case RTLSDR_RPC_OP_SET_TESTMODE:
    {
      uint32_t did;
      uint32_t on;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &on)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_set_testmode(rpcd->dev, (int)on);
      if (err) goto on_error;

      break ;
    }

  case RTLSDR_RPC_OP_SET_AGC_MODE:
    {
      uint32_t did;
      uint32_t on;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &on)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_set_agc_mode(rpcd->dev, (int)on);
      if (err) goto on_error;

      break ;
    }

  case RTLSDR_RPC_OP_SET_DIRECT_SAMPLING:
    {
      uint32_t did;
      uint32_t on;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &on)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_set_direct_sampling(rpcd->dev, (int)on);
      if (err) goto on_error;

      break ;
    }

  case RTLSDR_RPC_OP_GET_DIRECT_SAMPLING:
    {
      uint32_t did;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_get_direct_sampling(rpcd->dev);

      break ;
    }

  case RTLSDR_RPC_OP_SET_OFFSET_TUNING:
    {
      uint32_t did;
      uint32_t on;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &on)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_set_offset_tuning(rpcd->dev, (int)on);
      if (err) goto on_error;

      break ;
    }

  case RTLSDR_RPC_OP_GET_OFFSET_TUNING:
    {
      uint32_t did;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_get_offset_tuning(rpcd->dev);

      break ;
    }

  case RTLSDR_RPC_OP_RESET_BUFFER:
    {
      uint32_t did;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      err = rtlsdr_reset_buffer(rpcd->dev);
      if (err) goto on_error;

      break ;
    }

  case RTLSDR_RPC_OP_READ_ASYNC:
    {
      uint32_t did;
      uint32_t buf_num;
      uint32_t buf_len;
      rtlsdr_rpc_op_t new_op;
      rtlsdr_rpc_msg_t* new_q;
      rtlsdr_rpc_msg_t* new_r;
      uint8_t id;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &buf_num)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &buf_len)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      /* prepare the reply here */
      id = rtlsdr_rpc_msg_get_id(q);
      rtlsdr_rpc_msg_set_size(r, r->off);
      rtlsdr_rpc_msg_set_op(r, op);
      rtlsdr_rpc_msg_set_id(r, id);
      rtlsdr_rpc_msg_set_err(r, 0);
      
      rpcd->async_id = id;
      rpcd->async_replied = 0;
      while (1)
      {
	rtlsdr_read_async
	  (rpcd->dev, read_async_cb, rpcd, buf_num, buf_len);

	if (rpcd->async_replied == 0) goto on_error;

	if (recv_query(rpcd, &new_q)) goto on_error;

	new_op = rtlsdr_rpc_msg_get_op(new_q);

	/* do not reply is cancel_async */
	if (new_op == RTLSDR_RPC_OP_CANCEL_ASYNC) return 0;
	handle_query(rpcd, new_q, &new_r);

	if (new_r != NULL) send_reply(rpcd, new_r);
      }

      /* do not resend reply */
      if (rpcd->async_replied) return 0;
      goto on_error;
      break ;
    }

  case RTLSDR_RPC_OP_CANCEL_ASYNC:
    {
      uint32_t did;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      /* already cancelled if here */
      err = 0;

      break ;
    }

  case RTLSDR_RPC_OP_READ_SYNC:
    {
      uint32_t did;
      uint32_t len;
      int n_read;
      uint8_t* buf;
      size_t new_size;

      if (rtlsdr_rpc_msg_pop_uint32(q, &did)) goto on_error;
      if (rtlsdr_rpc_msg_pop_uint32(q, &len)) goto on_error;

      if ((rpcd->dev == NULL) || (rpcd->did != did)) goto on_error;

      new_size = r->off + sizeof(uint32_t) + len;
      if (rtlsdr_rpc_msg_realloc(r, new_size)) goto on_error;
      buf = r->fmt + r->off + sizeof(uint32_t);

      err = rtlsdr_read_sync(rpcd->dev, buf, (int)len, &n_read);
      if (err) goto on_error;

      rtlsdr_rpc_msg_push_uint32_safe(r, (uint32_t)n_read);
      rtlsdr_rpc_msg_skip_safe(r, (size_t)n_read);

      break ;
    }

  default:
    {
      PRINTF("invalid op: %u\n", op);
      break ;
    }
  }

 on_error:
  rtlsdr_rpc_msg_set_size(r, r->off);
  rtlsdr_rpc_msg_set_op(r, op);
  rtlsdr_rpc_msg_set_id(r, rtlsdr_rpc_msg_get_id(q));
  rtlsdr_rpc_msg_set_err(r, err);
  *rr = r;
  return 0;
}

static int do_rpcd(rpcd_t* rpcd)
{
  fd_set rset;
  int max_fd;

  while (1)
  {
    FD_ZERO(&rset);

    if (rpcd->cli_sock != -1)
    {
      FD_SET(rpcd->cli_sock, &rset);
      max_fd = rpcd->cli_sock;
    }
    else
    {
      FD_SET(rpcd->listen_sock, &rset);
      max_fd = rpcd->listen_sock;
    }

    if (select(max_fd + 1, &rset, NULL, NULL, NULL) <= 0)
    {
      ERROR();
      return -1;
    }

    if (FD_ISSET(rpcd->listen_sock, &rset))
    {
      PRINTF("new client\n");

      rpcd->cli_sock = accept(rpcd->listen_sock, NULL, NULL);
      if (rpcd->cli_sock == -1)
      {
	if ((errno == EWOULDBLOCK) || (errno == EAGAIN))
	  continue ;
	ERROR();
	break ;
      }

      if (fcntl(rpcd->cli_sock, F_SETFL, O_NONBLOCK))
      {
	ERROR();
	break ;
      }
    }
    else if (FD_ISSET(rpcd->cli_sock, &rset))
    {
      rtlsdr_rpc_msg_t* q;
      rtlsdr_rpc_msg_t* r;

      if (recv_query(rpcd, &q))
      {
	PRINTF("cli closed\n");
	shutdown(rpcd->cli_sock, SHUT_RDWR);
	close(rpcd->cli_sock);
	rpcd->cli_sock = -1;
      }
      else if (q != NULL)
      {
	handle_query(rpcd, q, &r);
	if (r != NULL) send_reply(rpcd, r);
      }
    }
  }

  return 0;
}

void usage(void)
{
  fprintf(stderr,
    "rtl_rpcd, a Remote Procedure Call server for RTL-SDR dongles\n\n"
    "Use:\trtl_rpcd [-options]\n"
    "\t[-a address]\tAddress to listen on (default: 0.0.0.0 for all), or RTLSDR_RPC_SERV_ADDR\n"
    "\t[-p port]\tPort number to listen on (default: 40000), or RTLSDR_RPC_SERV_PORT\n"
    "\t[-I port_ir]\tinfrared sensor listen port (default: 0=none)\n"
    "\t[-w wait_ir]\tinfrared sensor query wait interval usec (default: 10000)\n"
    "\t[-h]\t\tHelp\n"
    "\n"
    "On the remote client, set RTLSDR_RPC_IS_ENABLED and matching address/port\n"
  );
  exit(1);
}

void *ir_thread_fn(void *arg)
{
  int r = 1;
  struct linger ling = {1,0};
  int listensocket;
  int irsocket;
  struct sockaddr_in local, remote;
  socklen_t rlen;
  uint8_t buf[128];
  int ret = 0, len;
  struct timeval tv;
  fd_set writefds, errorfds;

  rpcd_t *rpcd = (rpcd_t *)arg;

  rtlsdr_dev_t *dev = rpcd->dev;
  const char *addr = rpcd->addr_ir;
  int port = rpcd->port_ir;
  int wait = rpcd->wait_ir;


  memset(&local,0,sizeof(local));
  local.sin_family = AF_INET;
  local.sin_port = htons(port);
  local.sin_addr.s_addr = inet_addr(addr);

  listensocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  r = 1;
  setsockopt(listensocket, SOL_SOCKET, SO_REUSEADDR, (char *)&r, sizeof(int));
  setsockopt(listensocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
  bind(listensocket,(struct sockaddr *)&local,sizeof(local));


  while(1) {
    printf("listening on IR port %d...\n", port);
    listen(listensocket,1);

    irsocket = accept(listensocket,(struct sockaddr *)&remote, &rlen);
    setsockopt(irsocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));

    printf("IR client accepted!\n");

    while(1) {
      dev = rpcd->dev;
      if (!dev) {
        printf("no device available yet\n");
        // Keep waiting until RPC connects and opens device
        usleep(1000000);
        continue;
      }

      ret = rtlsdr_ir_query(dev, buf, sizeof(buf));
      if (ret < 0) {
        printf("rtlsdr_ir_query error %d\n", ret);
        break;
      }

      len = ret;

      FD_ZERO(&writefds);
      FD_SET(irsocket, &writefds);

      FD_ZERO(&errorfds);
      FD_SET(irsocket, &errorfds);

      tv.tv_sec = 1;
      tv.tv_usec = 0;
      select(irsocket+1, NULL, &writefds, &errorfds, &tv);

      if (FD_ISSET(irsocket, &errorfds)) {
        printf("error condition from irsocket, breaking\n");
        break;
      }

      if (FD_ISSET(irsocket, &writefds)) {
        ret = send(irsocket, buf, len, 0);
        if (ret != len){
          printf("incomplete write to ir client: %d != %d\n", ret,len);
          break;
        }
      } else {
        printf("irsocket not ready to write, breaking\n");
        break;
      }

      usleep(wait);
    }

    close(irsocket);
  }

  return 0;
}


int main(int ac, char** av)
{
  rpcd_t rpcd;
  const char* addr = NULL;
  const char* port = NULL;

  int opt;
  int port_ir = 0, wait_ir = 10000;
  pthread_t thread_ir;

  while ((opt = getopt(ac, av, "a:p:I:w:h")) != -1) {
    switch (opt) {
    case 'a':
        addr = optarg;
        break;
    case 'p':
        port = optarg;
        break;
    case 'I':
        port_ir = atoi(optarg);
        break;
    case 'w':
        wait_ir = atoi(optarg);
        break;
    case 'h':
    default:
        usage();
    }
  }

  if (addr == NULL) addr = getenv("RTLSDR_RPC_SERV_ADDR");
  if (addr == NULL) addr = "0.0.0.0";

  if (port == NULL) port = getenv("RTLSDR_RPC_SERV_PORT");
  if (port == NULL) port = "40000";

  if (init_rpcd(&rpcd, addr, port)) return -1;
  if (port_ir) {
    rpcd.port_ir = port_ir;
    rpcd.addr_ir = addr;
    rpcd.wait_ir = wait_ir;
    pthread_create(&thread_ir, NULL, &ir_thread_fn, (void *)(&rpcd));
  }

  do_rpcd(&rpcd);
  fini_rpcd(&rpcd);

  return 0;
}
