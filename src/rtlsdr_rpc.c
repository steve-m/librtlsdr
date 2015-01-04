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
#include <netinet/in.h>
#include <netdb.h>
#include "rtlsdr_rpc_msg.h"


#if 1
#include <stdio.h>
#define PRINTF(__s, ...) fprintf(stderr, __s, ##__VA_ARGS__)
#define TRACE() PRINTF("[t] %s,%u\n", __FILE__, __LINE__)
#define ERROR() PRINTF("[e] %s,%u\n", __FILE__, __LINE__)
#define UNIMPL() PRINTF("[u] %s,%u\n", __FILE__, __LINE__)
#else
#define PRINTF(...)
#define TRACE()
#define ERROR()
#define UNIMPL()
#endif


typedef struct
{
  unsigned int is_init;
  uint16_t mid;
  int sock;
  rtlsdr_rpc_msg_t query;
  rtlsdr_rpc_msg_t reply;
  volatile unsigned int is_async_cancel;
} rtlsdr_rpc_cli_t;

static rtlsdr_rpc_cli_t rtlsdr_rpc_cli = { 0, };

typedef void (*rtlsdr_rpc_read_async_cb_t)
(unsigned char*, uint32_t, void*);

typedef struct rtlsdr_rpc_dev
{
  uint32_t index;
  rtlsdr_rpc_cli_t* cli;
} rtlsdr_rpc_dev_t;

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

static int open_socket
(
 struct sockaddr_storage saddr_both[2], size_t size_both[2],
 int type, int proto,
 struct sockaddr_storage** saddr_used, size_t* size_used
)
{
  size_t i;
  int fd;

  for (i = 0; (i != 2) && (size_both[i]); ++i)
  {
    const struct sockaddr* const sa = (const struct sockaddr*)&saddr_both[i];
    fd = socket(sa->sa_family, type, proto);
    if (fd != -1) break ;
  }

  if ((i == 2) || (size_both[i] == 0)) return -1;

  *saddr_used = &saddr_both[i];
  *size_used = size_both[i];

  return fd;
}

static int init_cli(rtlsdr_rpc_cli_t* cli)
{
  struct sockaddr_storage saddrs[2];
  struct sockaddr_storage* saddr;
  size_t sizes[2];
  size_t size;
  const char* addr;
  const char* port;

  if (cli->is_init) return 0;

  addr = getenv("RTLSDR_RPC_SERV_ADDR");
  if (addr == NULL) addr = "127.0.0.1";

  port = getenv("RTLSDR_RPC_SERV_PORT");
  if (port == NULL) port = "40000";

  if (resolve_ip_addr(saddrs, sizes, addr, port))
  {
    ERROR();
    goto on_error_0;
  }

  cli->sock = open_socket
    (saddrs, sizes, SOCK_STREAM, IPPROTO_TCP, &saddr, &size);
  if (cli->sock == -1)
  {
    ERROR();
    goto on_error_0;
  }

  if (connect(cli->sock, (const struct sockaddr*)saddr, (socklen_t)size))
  {
    ERROR();
    goto on_error_1;
  }

  if (fcntl(cli->sock, F_SETFL, O_NONBLOCK))
  {
    ERROR();
    goto on_error_1;
  }

  cli->mid = 0;
  if (rtlsdr_rpc_msg_init(&cli->query, 0)) goto on_error_1;
  if (rtlsdr_rpc_msg_init(&cli->reply, 0)) goto on_error_2;
  cli->is_init = 1;

  return 0;

 on_error_2:
  rtlsdr_rpc_msg_fini(&cli->query);
 on_error_1:
  shutdown(cli->sock, SHUT_RDWR);
  close(cli->sock);
 on_error_0:
  return -1;
}

__attribute__((unused))
static int fini_cli(rtlsdr_rpc_cli_t* cli)
{
  rtlsdr_rpc_msg_fini(&cli->query);
  rtlsdr_rpc_msg_fini(&cli->reply);
  shutdown(cli->sock, SHUT_RDWR);
  close(cli->sock);
  return 0;
}

static uint16_t get_mid(rtlsdr_rpc_cli_t* cli)
{
  return cli->mid++;
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

static int send_all(int fd, const uint8_t* buf, size_t size)
{
  ssize_t n;
  fd_set wset;

  while (1)
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

    FD_ZERO(&wset);
    FD_SET(fd, &wset);
    if (select(fd + 1, NULL, &wset, NULL, NULL) <= 0)
    {
      return -1;
    }
  }

  return 0;
}

static int recv_msg(int fd, rtlsdr_rpc_msg_t* m)
{
  uint32_t size;

  if (recv_all(fd, (uint8_t*)&size, sizeof(uint32_t))) return -1;
  rtlsdr_rpc_msg_set_size(m, size);
  size = rtlsdr_rpc_msg_get_size(m);
  if (rtlsdr_rpc_msg_realloc(m, size)) return -1;
  size -= sizeof(uint32_t);
  if (recv_all(fd, m->fmt + sizeof(uint32_t), size)) return -1;
  return 0;
}

static int send_msg(int fd, rtlsdr_rpc_msg_t* m)
{
  return send_all(fd, m->fmt, m->off);
}

static int send_recv_msg
(rtlsdr_rpc_cli_t* cli, rtlsdr_rpc_msg_t* q, rtlsdr_rpc_msg_t* r)
{
  rtlsdr_rpc_msg_set_size(q, (uint32_t)q->off);
  rtlsdr_rpc_msg_set_mid(q, get_mid(cli));

  if (send_msg(cli->sock, q)) return -1;

  if (recv_msg(cli->sock, r)) return -1;
  rtlsdr_rpc_msg_reset(r);

  return 0;
}

uint32_t rtlsdr_rpc_get_device_count(void)
{
  rtlsdr_rpc_cli_t* const cli = &rtlsdr_rpc_cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  uint32_t n = 0;

  if (init_cli(cli)) goto on_error;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_DEVICE_COUNT);

  if (send_recv_msg(cli, q, r)) goto on_error;

  if (rtlsdr_rpc_msg_pop_uint32(r, &n)) goto on_error;

 on_error:
  return n;
}

const char* rtlsdr_rpc_get_device_name
(
 uint32_t index
)
{
  rtlsdr_rpc_cli_t* const cli = &rtlsdr_rpc_cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  const char* s = NULL;

  if (init_cli(cli)) goto on_error;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_DEVICE_NAME);
  if (rtlsdr_rpc_msg_push_uint32(q, index)) goto on_error;

  if (send_recv_msg(cli, q, r)) goto on_error;

  if (rtlsdr_rpc_msg_pop_str(r, &s)) goto on_error;
  /* TODO: memory leak here */
  s = strdup(s);

 on_error:
  return s;
}

int rtlsdr_rpc_get_device_usb_strings
(uint32_t index, char* manufact, char* product, char* serial)
{
  rtlsdr_rpc_cli_t* const cli = &rtlsdr_rpc_cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  const char* s;
  int err = -1;

  if (init_cli(cli)) goto on_error;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_DEVICE_USB_STRINGS);
  if (rtlsdr_rpc_msg_push_uint32(q, index)) goto on_error;

  if (send_recv_msg(cli, q, r)) goto on_error;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error;

  if (rtlsdr_rpc_msg_pop_str(r, &s)) goto on_error;
  strcpy(manufact, s);
  if (rtlsdr_rpc_msg_pop_str(r, &s)) goto on_error;
  strcpy(product, s);
  if (rtlsdr_rpc_msg_pop_str(r, &s)) goto on_error;
  strcpy(serial, s);

 on_error:
  return err;
}

int rtlsdr_rpc_get_index_by_serial(const char* serial)
{
  rtlsdr_rpc_cli_t* const cli = &rtlsdr_rpc_cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  int err = -1;

  if (init_cli(cli)) goto on_error;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_INDEX_BY_SERIAL);
  if (rtlsdr_rpc_msg_push_str(q, serial)) goto on_error;

  if (send_recv_msg(cli, q, r)) goto on_error;

  err = rtlsdr_rpc_msg_get_err(r);

 on_error:
  return err;
}

int rtlsdr_rpc_open(void** devp, uint32_t index)
{
  rtlsdr_rpc_cli_t* const cli = &rtlsdr_rpc_cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  rtlsdr_rpc_dev_t* dev;
  int err = -1;

  *devp = NULL;

  if (init_cli(cli)) goto on_error_0;

  dev = malloc(sizeof(rtlsdr_rpc_dev_t));
  if (dev == NULL) goto on_error_0;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_OPEN);
  if (rtlsdr_rpc_msg_push_uint32(q, index)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

  dev->index = index;
  dev->cli = cli;
  *devp = dev;

 on_error_1:
  if (err) free(dev);
 on_error_0:
  return err;
}

int rtlsdr_rpc_close(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  int err = -1;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_CLOSE);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_0;

  if (send_recv_msg(cli, q, r)) goto on_error_0;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_0;

 on_error_0:
  free(dev);
  return err;
}

int rtlsdr_rpc_set_xtal_freq
(void* devp, uint32_t rtl_freq, uint32_t tuner_freq)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  int err = -1;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_XTAL_FREQ);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_0;
  if (rtlsdr_rpc_msg_push_uint32(q, rtl_freq)) goto on_error_0;
  if (rtlsdr_rpc_msg_push_uint32(q, tuner_freq)) goto on_error_0;

  if (send_recv_msg(cli, q, r)) goto on_error_0;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_0;

 on_error_0:
  return err;
}

int rtlsdr_rpc_get_xtal_freq
(void* devp, uint32_t* rtl_freq, uint32_t* tuner_freq)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  int err = -1;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_XTAL_FREQ);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_0;

  if (send_recv_msg(cli, q, r)) goto on_error_0;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_0;

  if (rtlsdr_rpc_msg_pop_uint32(r, rtl_freq))
  {
    err = -1;
    goto on_error_0;
  }

  if (rtlsdr_rpc_msg_pop_uint32(r, tuner_freq))
  {
    err = -1;
    goto on_error_0;
  }

 on_error_0:
  return err;
}

int rtlsdr_rpc_get_usb_strings
(void* dev, char* manufact, char* product, char* serial)
{
  UNIMPL();
  return -1;
}

int rtlsdr_rpc_write_eeprom
(void* dev, uint8_t* data, uint8_t offset, uint16_t len)
{
  UNIMPL();
  return -1;
}

int rtlsdr_rpc_read_eeprom
(void* dev, uint8_t* data, uint8_t offset, uint16_t len)
{
  UNIMPL();
  return -1;
}

int rtlsdr_rpc_set_center_freq(void* devp, uint32_t freq)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  int err = -1;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_CENTER_FREQ);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_0;
  if (rtlsdr_rpc_msg_push_uint32(q, freq)) goto on_error_0;

  if (send_recv_msg(cli, q, r)) goto on_error_0;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_0;

 on_error_0:
  return err;
}

uint32_t rtlsdr_rpc_get_center_freq(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  uint32_t freq = 0;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_CENTER_FREQ);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_0;

  if (send_recv_msg(cli, q, r)) goto on_error_0;

  if (rtlsdr_rpc_msg_get_err(r)) goto on_error_0;
  if (rtlsdr_rpc_msg_pop_uint32(r, &freq)) goto on_error_0;

 on_error_0:
  return freq;
}

int rtlsdr_rpc_set_freq_correction(void* devp, int ppm)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  int err = -1;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_FREQ_CORRECTION);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error;
  if (rtlsdr_rpc_msg_push_uint32(q, (uint32_t)ppm)) goto on_error;

  if (send_recv_msg(cli, q, r)) goto on_error;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error;

 on_error:
  return err;
}

int rtlsdr_rpc_get_freq_correction(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  int err = -1;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_FREQ_CORRECTION);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error;

  if (send_recv_msg(cli, q, r)) goto on_error;

  err = rtlsdr_rpc_msg_get_err(r);

 on_error:
  return err;
}

int rtlsdr_rpc_get_tuner_type(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  int err = -1;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_TUNER_TYPE);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error;

  if (send_recv_msg(cli, q, r)) goto on_error;

  err = rtlsdr_rpc_msg_get_err(r);

 on_error:
  return err;
}

int rtlsdr_rpc_get_tuner_gains(void* dev, int* gainsp)
{
  UNIMPL();
  return -1;
}

int rtlsdr_rpc_set_tuner_gain(void* dev, int gain)
{
  UNIMPL();
  return -1;
}

int rtlsdr_rpc_get_tuner_gain(void* dev)
{
  UNIMPL();
  return -1;
}

int rtlsdr_rpc_set_tuner_if_gain(void* dev, int stage, int gain)
{
  UNIMPL();
  return -1;
}

int rtlsdr_rpc_set_tuner_gain_mode(void* devp, int manual)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  int err = -1;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_TUNER_GAIN_MODE);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error;
  if (rtlsdr_rpc_msg_push_uint32(q, manual)) goto on_error;

  if (send_recv_msg(cli, q, r)) goto on_error;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error;

 on_error:
  return err;
}

int rtlsdr_rpc_set_sample_rate(void* devp, uint32_t rate)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  int err = -1;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_SAMPLE_RATE);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_0;
  if (rtlsdr_rpc_msg_push_uint32(q, rate)) goto on_error_0;

  if (send_recv_msg(cli, q, r)) goto on_error_0;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_0;

 on_error_0:
  return err;
}

uint32_t rtlsdr_rpc_get_sample_rate(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  uint32_t rate = 0;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_SAMPLE_RATE);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_0;

  if (send_recv_msg(cli, q, r)) goto on_error_0;

  if (rtlsdr_rpc_msg_get_err(r)) goto on_error_0;
  if (rtlsdr_rpc_msg_pop_uint32(r, &rate)) goto on_error_0;

 on_error_0:
  return rate;
}

int rtlsdr_rpc_set_testmode(void* dev, int on)
{
  UNIMPL();
  return -1;
}

int rtlsdr_rpc_set_agc_mode(void* dev, int on)
{
  UNIMPL();
  return -1;
}

int rtlsdr_rpc_set_direct_sampling(void* dev, int on)
{
  UNIMPL();
  return -1;
}

int rtlsdr_rpc_get_direct_sampling(void* dev)
{
  UNIMPL();
  return -1;
}

int rtlsdr_rpc_set_offset_tuning(void* dev, int on)
{
  UNIMPL();
  return -1;
}

int rtlsdr_rpc_get_offset_tuning(void* dev)
{
  UNIMPL();
  return -1;
}

int rtlsdr_rpc_reset_buffer(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  int err = -1;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_RESET_BUFFER);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_0;

  if (send_recv_msg(cli, q, r)) goto on_error_0;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_0;

 on_error_0:
  return err;
}

int rtlsdr_rpc_read_sync
(void* devp, void* buf, int len, int* n_read)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  const uint8_t* tmp;
  size_t size;
  int err = -1;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_READ_SYNC);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error;
  if (rtlsdr_rpc_msg_push_uint32(q, (uint32_t)len)) goto on_error;

  if (send_recv_msg(cli, q, r)) goto on_error;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error;

  if (rtlsdr_rpc_msg_pop_buf(r, &tmp, &size))
  {
    err = -1;
    goto on_error;
  }

  if (size > (size_t)len) size = len;

  memcpy(buf, tmp, size);

  *n_read = (int)size;

 on_error:
  return err;
}

static volatile unsigned int is_cancel;
int rtlsdr_rpc_read_async
(
 void* devp,
 rtlsdr_rpc_read_async_cb_t cb, void* ctx,
 uint32_t buf_num, uint32_t buf_len
)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* const q = &cli->query;
  rtlsdr_rpc_msg_t* const r = &cli->reply;
  size_t size;
  int err = -1;

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_READ_ASYNC);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_0;
  if (rtlsdr_rpc_msg_push_uint32(q, buf_num)) goto on_error_0;
  if (rtlsdr_rpc_msg_push_uint32(q, buf_len)) goto on_error_0;

  if (send_recv_msg(cli, q, r)) goto on_error_0;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_0;

  cli->is_async_cancel = 0;
  while (cli->is_async_cancel == 0)
  {
    static const size_t off = offsetof(rtlsdr_rpc_fmt_t, data);

    if (recv_msg(cli->sock, r))
    {
      err = -1;
      goto on_error_0;
    }

    size = rtlsdr_rpc_msg_get_size(r);
    cb(r->fmt + off, size - off, ctx);
  }

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_CANCEL_ASYNC);
  rtlsdr_rpc_msg_push_uint32(q, dev->index);
  send_recv_msg(cli, q, r);

 on_error_0:
  return err;
}

int rtlsdr_rpc_wait_async
(
 void* dev,
 rtlsdr_rpc_read_async_cb_t cb, void* ctx
)
{
  return rtlsdr_rpc_read_async(dev, cb, ctx, 0, 0);
}

int rtlsdr_rpc_cancel_async(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  cli->is_async_cancel = 1;
  return 0;
}

unsigned int rtlsdr_rpc_is_enabled(void)
{
  static unsigned int is_enabled = (unsigned int)-1;
  if (is_enabled == (unsigned int)-1)
    is_enabled = (getenv("RTLSDR_RPC_IS_ENABLED") != NULL);
  return is_enabled;
}
