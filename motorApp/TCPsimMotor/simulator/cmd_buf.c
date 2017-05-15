#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "cmd_buf.h"
#include "sock-util.h"

static size_t buf_len = 0;
static size_t used_len = 0;
static char *buf = NULL;


/*****************************************************************************/
static int cmd_buf_vprintf_crlf (int flags, const char* format, va_list arg)
{
  const static size_t len = 4096;
  int add_cr = flags & PRINT_ADD_CR;

  char *buf = calloc(len,1);
  int res = vsnprintf(buf, len-1, format, arg);
  if (res >= 0 && !add_cr) {
    add_to_buf(buf, res);
  }
  else if (res >= 0)
  {
    unsigned src_idx = 0;
    unsigned dst_idx = 0;
    char *buf2 = calloc(2*res, 1);
    char oldc = 0;
    for (src_idx=0; src_idx < res; src_idx++)
    {
      char c = buf[src_idx];
      buf2[dst_idx] = c;
#if 1
      if ((c == '\n') && (oldc != '\r')) {
        buf2[dst_idx++] = '\r';
        buf2[dst_idx] = '\n';
      }
#else
      (void)oldc;
#endif
      oldc = c;
      dst_idx++;
    }
    add_to_buf(buf2, dst_idx);

    free(buf2);
  }
  free(buf);
  return res;
}

/*****************************************************************************/
void cmd_buf_printf(const char *format, ...)
{
  va_list ap;

  va_start(ap, format);
  (void)cmd_buf_vprintf_crlf(0, format, ap);
  va_end(ap);
}

/*****************************************************************************/
void cmd_buf_printf_crlf(int add_cr, const char *format, ...)
{
  va_list ap;

  va_start(ap, format);
  (void)cmd_buf_vprintf_crlf(add_cr, format, ap);
  va_end(ap);
}

/*****************************************************************************/
void add_to_buf(const char *add_txt, size_t add_len)
{
  if (add_len + used_len >= buf_len) {
    buf_len = 1 + add_len + used_len; /* 1 for '\0' */
    buf = realloc(buf, buf_len);
  }
  memcpy(&buf[used_len], add_txt, add_len);
  used_len += add_len;
  buf[used_len] = '\0';
}

/*****************************************************************************/
char *get_buf(void)
{
  return buf ? buf : "";
}

/*****************************************************************************/
void clear_buf(void)
{
  used_len = 0;
}
