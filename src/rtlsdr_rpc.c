#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
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
  volatile unsigned int is_locked;
  volatile unsigned int is_init;
  int sock;

#define QR_COUNT 32
  pthread_mutex_t qr_lock;
  volatile uint32_t qr_mask;
  volatile uint32_t qr_box;
  rtlsdr_rpc_msg_t query[QR_COUNT];
  rtlsdr_rpc_msg_t reply[QR_COUNT];

  pthread_mutex_t send_lock;
  pthread_mutex_t recv_lock;

  volatile unsigned int is_async_cancel;

} rtlsdr_rpc_cli_t;

static rtlsdr_rpc_cli_t rtlsdr_rpc_cli = { 0, };

typedef void (*rtlsdr_rpc_read_async_cb_t)
(unsigned char*, uint32_t, void*);

typedef struct rtlsdr_rpc_dev
{
  uint32_t index;
  size_t gain_count;
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
  size_t i;
  size_t j;
  int err = -1;

  /* no better way in this case ... */
  while (cli->is_locked) usleep(10000);
  cli->is_locked = 1;

  if (cli->is_init) goto on_success;

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

  for (i = 0; i != QR_COUNT; ++i)
  {
    rtlsdr_rpc_msg_t* const q = &cli->query[i];
    rtlsdr_rpc_msg_t* const r = &cli->reply[i];

    if (rtlsdr_rpc_msg_init(q, 0))
      goto on_error_2;

    if (rtlsdr_rpc_msg_init(r, 0))
    {
      rtlsdr_rpc_msg_fini(q);
      goto on_error_2;
    }
  }
  pthread_mutex_init(&cli->qr_lock, NULL);
  cli->qr_mask = 0;
  cli->qr_box = 0;

  pthread_mutex_init(&cli->send_lock, NULL);
  pthread_mutex_init(&cli->recv_lock, NULL);

  cli->is_init = 1;

 on_success:
  err = 0;
  goto on_error_0;

 on_error_2:
  for (j = 0; j != i; ++j)
  {
    rtlsdr_rpc_msg_fini(&cli->query[j]);
    rtlsdr_rpc_msg_fini(&cli->reply[j]);
  }

 on_error_1:
  shutdown(cli->sock, SHUT_RDWR);
  close(cli->sock);

 on_error_0:
  cli->is_locked = 0;
  return err;
}

__attribute__((unused))
static int fini_cli(rtlsdr_rpc_cli_t* cli)
{
  size_t i;

  for (i = 0; i != QR_COUNT; ++i)
  {
    rtlsdr_rpc_msg_fini(&cli->query[i]);
    rtlsdr_rpc_msg_fini(&cli->reply[i]);
  }
  pthread_mutex_destroy(&cli->qr_lock);

  pthread_mutex_destroy(&cli->send_lock);
  pthread_mutex_destroy(&cli->recv_lock);

  shutdown(cli->sock, SHUT_RDWR);
  close(cli->sock);

  return 0;
}

static int alloc_qr
(rtlsdr_rpc_cli_t* cli, rtlsdr_rpc_msg_t** q, rtlsdr_rpc_msg_t** r)
{
  size_t i;

  pthread_mutex_lock(&cli->qr_lock);
  for (i = 0; i != QR_COUNT; ++i)
  {
    const uint32_t m = 1 << i;
    if ((cli->qr_mask & m) == 0)
    {
      cli->qr_mask |= m;
      break ;
    }
  }
  pthread_mutex_unlock(&cli->qr_lock);

  if (i == QR_COUNT) return -1;

  *q = &cli->query[i];
  *r = &cli->reply[i];

  /* set the query id */
  rtlsdr_rpc_msg_reset(*q);
  rtlsdr_rpc_msg_set_id(*q, (uint8_t)i);

  return 0;
}

static void free_qr
(rtlsdr_rpc_cli_t* cli, rtlsdr_rpc_msg_t* q, rtlsdr_rpc_msg_t* r)
{
  const uint32_t m = 1 << (uint32_t)rtlsdr_rpc_msg_get_id(q);
  pthread_mutex_lock(&cli->qr_lock);
  cli->qr_mask &= ~m;
  pthread_mutex_unlock(&cli->qr_lock);
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

static int recv_msg
(rtlsdr_rpc_cli_t* cli, uint8_t id, rtlsdr_rpc_msg_t* m)
{
  static const size_t fmt_size = offsetof(rtlsdr_rpc_fmt_t, data);
  const uint32_t mask = 1 << (uint32_t)id;
  const int fd = cli->sock;
  uint32_t size;
  uint8_t to_id;
  int err = -1;
  rtlsdr_rpc_msg_t* to_m;

  pthread_mutex_lock(&cli->recv_lock);

  if (cli->qr_box & mask)
  {
    cli->qr_box &= ~mask;
    goto on_success;
  }

  while (1)
  {
    /* receive next message */
    if (recv_all(fd, m->fmt, fmt_size)) goto on_error;

    /* get destination message by id */
    to_id = rtlsdr_rpc_msg_get_id(m);

    if (to_id >= QR_COUNT) goto on_error;
    to_m = &cli->reply[to_id];
    if (to_id != id) memcpy(to_m->fmt, m->fmt, fmt_size);

    size = rtlsdr_rpc_msg_get_size(to_m);
    if (size < fmt_size) goto on_error;

    if (rtlsdr_rpc_msg_realloc(to_m, size)) goto on_error;

    size -= fmt_size;
    if (size)
    {
      if (recv_all(fd, to_m->fmt + fmt_size, size)) goto on_error;
    }

    if (to_id == id) goto on_success;

    /* message not for this query, forward */
    cli->qr_box |= 1 << (uint32_t)to_id;
  }

 on_success:
  err = 0;
 on_error:
  pthread_mutex_unlock(&cli->recv_lock);
  return err;
}

static int send_msg(rtlsdr_rpc_cli_t* cli, rtlsdr_rpc_msg_t* m)
{
  int err;

  rtlsdr_rpc_msg_set_size(m, (uint32_t)m->off);

  pthread_mutex_lock(&cli->send_lock);
  err = send_all(cli->sock, m->fmt, m->off);
  pthread_mutex_unlock(&cli->send_lock);

  return err;
}

static int send_recv_msg
(rtlsdr_rpc_cli_t* cli, rtlsdr_rpc_msg_t* q, rtlsdr_rpc_msg_t* r)
{
  const uint8_t id = rtlsdr_rpc_msg_get_id(q);

  if (send_msg(cli, q)) return -1;
  if (recv_msg(cli, id, r)) return -1;
  rtlsdr_rpc_msg_reset(r);

  return 0;
}

static int send_flush_msgs
(rtlsdr_rpc_cli_t* cli, rtlsdr_rpc_msg_t* q)
{
  struct timeval tm;
  ssize_t n;
  fd_set rset;
  uint8_t buf[256];

  if (send_msg(cli, q)) return -1;

  pthread_mutex_lock(&cli->recv_lock);

  while (1)
  {
    FD_ZERO(&rset);
    FD_SET(cli->sock, &rset);
    tm.tv_sec = 0;
    tm.tv_usec = 200000;
    if (select(cli->sock + 1, &rset, NULL, NULL, &tm) < 0) break ;
    if (recv(cli->sock, buf, sizeof(buf), 0) <= 0) break ;
  }

  pthread_mutex_unlock(&cli->recv_lock);

  return 0;
}

uint32_t rtlsdr_rpc_get_device_count(void)
{
  rtlsdr_rpc_cli_t* const cli = &rtlsdr_rpc_cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  uint32_t n = 0;

  if (init_cli(cli)) goto on_error_0;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_DEVICE_COUNT);

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  if (rtlsdr_rpc_msg_pop_uint32(r, &n)) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return n;
}

const char* rtlsdr_rpc_get_device_name
(
 uint32_t index
)
{
  rtlsdr_rpc_cli_t* const cli = &rtlsdr_rpc_cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  const char* s = NULL;

  if (init_cli(cli)) goto on_error_0;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_DEVICE_NAME);
  if (rtlsdr_rpc_msg_push_uint32(q, index)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  if (rtlsdr_rpc_msg_pop_str(r, &s)) goto on_error_1;
  /* TODO: memory leak here */
  s = strdup(s);

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return s;
}

int rtlsdr_rpc_get_device_usb_strings
(uint32_t index, char* manufact, char* product, char* serial)
{
  rtlsdr_rpc_cli_t* const cli = &rtlsdr_rpc_cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  const char* s;
  int err = -1;

  if (init_cli(cli)) goto on_error_0;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_DEVICE_USB_STRINGS);
  if (rtlsdr_rpc_msg_push_uint32(q, index)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

  if (rtlsdr_rpc_msg_pop_str(r, &s)) goto on_error_1;
  strcpy(manufact, s);
  if (rtlsdr_rpc_msg_pop_str(r, &s)) goto on_error_1;
  strcpy(product, s);
  if (rtlsdr_rpc_msg_pop_str(r, &s)) goto on_error_1;
  strcpy(serial, s);

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_get_index_by_serial(const char* serial)
{
  rtlsdr_rpc_cli_t* const cli = &rtlsdr_rpc_cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (init_cli(cli)) goto on_error_0;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_INDEX_BY_SERIAL);
  if (rtlsdr_rpc_msg_push_str(q, serial)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_open(void** devp, uint32_t index)
{
  rtlsdr_rpc_cli_t* const cli = &rtlsdr_rpc_cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  rtlsdr_rpc_dev_t* dev;
  int err = -1;

  *devp = NULL;

  if (init_cli(cli)) goto on_error_0;

  dev = malloc(sizeof(rtlsdr_rpc_dev_t));
  if (dev == NULL) goto on_error_0;

  if (alloc_qr(cli, &q, &r)) goto on_error_1;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_OPEN);
  if (rtlsdr_rpc_msg_push_uint32(q, index)) goto on_error_2;

  if (send_recv_msg(cli, q, r)) goto on_error_2;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_2;

  dev->index = index;
  dev->gain_count = 32;
  dev->cli = cli;
  *devp = dev;

 on_error_2:
  free_qr(cli, q, r);
 on_error_1:
  if (err) free(dev);
 on_error_0:
  return err;
}

int rtlsdr_rpc_close(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_CLOSE);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  free(dev);
  return err;
}

int rtlsdr_rpc_set_xtal_freq
(void* devp, uint32_t rtl_freq, uint32_t tuner_freq)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_XTAL_FREQ);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, rtl_freq)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, tuner_freq)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_get_xtal_freq
(void* devp, uint32_t* rtl_freq, uint32_t* tuner_freq)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_XTAL_FREQ);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

  if (rtlsdr_rpc_msg_pop_uint32(r, rtl_freq))
  {
    err = -1;
    goto on_error_1;
  }

  if (rtlsdr_rpc_msg_pop_uint32(r, tuner_freq))
  {
    err = -1;
    goto on_error_1;
  }

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_get_usb_strings
(void* devp, char* manufact, char* product, char* serial)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = &rtlsdr_rpc_cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  const char* s;
  int err = -1;

  if (init_cli(cli)) goto on_error_0;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_USB_STRINGS);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

  if (rtlsdr_rpc_msg_pop_str(r, &s)) goto on_error_1;
  strcpy(manufact, s);
  if (rtlsdr_rpc_msg_pop_str(r, &s)) goto on_error_1;
  strcpy(product, s);
  if (rtlsdr_rpc_msg_pop_str(r, &s)) goto on_error_1;
  strcpy(serial, s);

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
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
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_CENTER_FREQ);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, freq)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

uint32_t rtlsdr_rpc_get_center_freq(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  uint32_t freq = 0;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_CENTER_FREQ);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  if (rtlsdr_rpc_msg_get_err(r)) goto on_error_1;
  if (rtlsdr_rpc_msg_pop_uint32(r, &freq)) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return freq;
}

int rtlsdr_rpc_set_freq_correction(void* devp, int ppm)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_FREQ_CORRECTION);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, (uint32_t)ppm)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_get_freq_correction(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_FREQ_CORRECTION);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_get_tuner_type(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_TUNER_TYPE);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_get_tuner_gains(void* devp, int* gainsp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  const uint32_t is_null = (gainsp == NULL);
  const uint32_t gain_size = (uint32_t)dev->gain_count * sizeof(int);
  const uint8_t* tmp;
  size_t size;
  int err = 0;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_TUNER_GAINS);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, is_null)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, gain_size)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);

  if (err <= 0) goto on_error_1;

  dev->gain_count = (size_t)err;

  if (is_null == 0)
  {
    if (rtlsdr_rpc_msg_pop_buf(r, &tmp, &size))
    {
      err = 0;
      goto on_error_1;
    }

    /* TODO: endianess */
    memcpy(gainsp, tmp, size);
  }

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_set_tuner_gain(void* devp, int gain)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_TUNER_GAIN);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, (uint32_t)gain)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_get_tuner_gain(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_TUNER_GAIN);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_set_tuner_if_gain(void* devp, int stage, int gain)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_TUNER_IF_GAIN);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, (uint32_t)stage)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, (uint32_t)gain)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_set_tuner_gain_mode(void* devp, int manual)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_TUNER_GAIN_MODE);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, manual)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_set_sample_rate(void* devp, uint32_t rate)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_SAMPLE_RATE);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, rate)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

uint32_t rtlsdr_rpc_get_sample_rate(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  uint32_t rate = 0;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_SAMPLE_RATE);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  if (rtlsdr_rpc_msg_get_err(r)) goto on_error_1;
  if (rtlsdr_rpc_msg_pop_uint32(r, &rate)) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return rate;
}

int rtlsdr_rpc_set_testmode(void* devp, int on)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_TESTMODE);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, (uint32_t)on)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_set_agc_mode(void* devp, int on)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_AGC_MODE);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, (uint32_t)on)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_set_direct_sampling(void* devp, int on)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_DIRECT_SAMPLING);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, (uint32_t)on)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_get_direct_sampling(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_DIRECT_SAMPLING);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_set_offset_tuning(void* devp, int on)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_SET_OFFSET_TUNING);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, (uint32_t)on)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_get_offset_tuning(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_GET_OFFSET_TUNING);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_reset_buffer(void* devp)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_RESET_BUFFER);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
  return err;
}

int rtlsdr_rpc_read_sync
(void* devp, void* buf, int len, int* n_read)
{
  rtlsdr_rpc_dev_t* const dev = devp;
  rtlsdr_rpc_cli_t* const cli = dev->cli;
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  const uint8_t* tmp;
  size_t size;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_READ_SYNC);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, (uint32_t)len)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

  if (rtlsdr_rpc_msg_pop_buf(r, &tmp, &size))
  {
    err = -1;
    goto on_error_1;
  }

  if (size > (size_t)len) size = len;

  memcpy(buf, tmp, size);

  *n_read = (int)size;

 on_error_1:
  free_qr(cli, q, r);
 on_error_0:
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
  rtlsdr_rpc_msg_t* q;
  rtlsdr_rpc_msg_t* r;
  uint8_t id;
  size_t size;
  int err = -1;

  if (alloc_qr(cli, &q, &r)) goto on_error_0;

  id = rtlsdr_rpc_msg_get_id(q);

  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_READ_ASYNC);
  if (rtlsdr_rpc_msg_push_uint32(q, dev->index)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, buf_num)) goto on_error_1;
  if (rtlsdr_rpc_msg_push_uint32(q, buf_len)) goto on_error_1;

  if (send_recv_msg(cli, q, r)) goto on_error_1;

  err = rtlsdr_rpc_msg_get_err(r);
  if (err) goto on_error_1;

  cli->is_async_cancel = 0;
  while (cli->is_async_cancel == 0)
  {
    static const size_t off = offsetof(rtlsdr_rpc_fmt_t, data);

    if (recv_msg(cli, id, r))
    {
      err = -1;
      goto on_error_1;
    }

    size = rtlsdr_rpc_msg_get_size(r);
    cb(r->fmt + off, size - off, ctx);
  }

  rtlsdr_rpc_msg_reset(q);
  rtlsdr_rpc_msg_set_id(q, id);
  rtlsdr_rpc_msg_set_op(q, RTLSDR_RPC_OP_CANCEL_ASYNC);
  rtlsdr_rpc_msg_push_uint32(q, dev->index);
  send_flush_msgs(cli, q);

 on_error_1:
  free_qr(cli, q, r);
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
