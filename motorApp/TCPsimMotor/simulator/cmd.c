#include <string.h>
#include <ctype.h> /* isprint() */
#include <stdlib.h>
#include <stdarg.h>
#include "sock-util.h"
#include "cmd_TCPsim.h"
#include "logerr_info.h"
#include "cmd_buf.h"

void dump_to_std(const char *buf,
                 unsigned len,
                 const char *inout,
                 int had_cr, int had_lf)
{
  int i;
  if (!inout) {
    inout = "INOUT";
  }
  if (!stdlog) {
    return;
  }
  fprintf(stdlog, "------- len=%u cr=%d lf=%d----------%s =>\n", len, had_cr, had_lf, inout);
  for (i=0; i < len; i++) {
    int ch = buf[i];
    switch (ch) {
      case '\t':
        fprintf(stdlog,"\\t");
        break;
      case '\r':
        fprintf(stdlog,"\\r");
        break;
      case '\n':
        fprintf(stdlog,"\\n");
        break;
      default:
        if (isprint(ch))
          fprintf(stdlog,"%c", ch);
        else
          fprintf(stdlog,"\\%03o", ch);
    }
  }
  fprintf(stdlog,"%s%s\n=================End of %s\n",
         had_cr ? "\\r" : "",
         had_lf ? "\\n" : "",
         inout);
}

void dump_and_send(int fd, int flags, const char *buf, unsigned len)
{
  if ((flags & PRINT_OUT) || PRINT_STDOUT_BIT1()) {
    dump_to_std(buf, len, "OUT", 0, 0);
  }
  send_to_socket(fd, buf, len);
}
/*****************************************************************************/
static int fd_vprintf_crlf(int fd, int flags, const char* format, va_list arg)
{
  const static size_t len = 4096;
  int add_cr = flags & PRINT_ADD_CR;

  char *buf = calloc(len,1);
  int res = vsnprintf(buf, len-1, format, arg);
  if (res > 0 && !add_cr) {
    dump_and_send(fd, flags, buf, res);
  }
  else if (res > 0)
  {
    unsigned src_idx = 0;
    unsigned dst_idx = 0;
    char *buf2 = calloc(2*res, 1);
    char oldc = 0;
    for (src_idx=0; src_idx < res; src_idx++)
    {
      char c = buf[src_idx];
      buf2[dst_idx] = c;
      if ((c == '\n') && (oldc != '\r')) {
        buf2[dst_idx++] = '\r';
        buf2[dst_idx] = '\n';
      }
      oldc = c;
      dst_idx++;
    }
    dump_and_send(fd, flags, buf2, dst_idx);
    free(buf2);
  }
  free(buf);
  return res;
}

/*****************************************************************************/
void fd_printf_crlf(int fd, int add_cr, const char *format, ...)
{
  va_list ap;

  va_start(ap, format);
  (void)fd_vprintf_crlf(fd, add_cr, format, ap);
  va_end(ap);
}
/*****************************************************************************/



static int create_argv(const char *line, int had_cr, int had_lf, const char*** argv_p)
{
  char *input_line = strdup(line);
  size_t calloc_len = 2 + strlen(input_line);
  int argc = 0;
  /* Allocate an array big enough, could be max strlen/2
     space <-> non-space transitions */
  const char **argv; /* May be more */

  if (PRINT_STDOUT_BIT0()) {
    dump_to_std(line, (unsigned)strlen(line), "IN", had_cr, had_lf);
  }

  argv = (const char **) (void *)calloc(calloc_len, sizeof(char *));
  *argv_p = argv;
  if (argv  == NULL)
  {
    return 0;
  }
  /* argv[0] is the whole line */
  argv[argc++] = strdup(input_line);;
  if (!strlen(input_line)) {
    fprintf(stdlog, "%s/%s:%d argc=%d (Early return)\n", __FILE__, __FUNCTION__, __LINE__,
            argc);
    return argc;
  }
  if (strchr(input_line, ' ') != NULL) {
    /* Start the loop */
    char *arg = strtok(input_line, " ");
    while (arg) {
      argv[argc++] = strdup(arg);
      arg = strtok(NULL, " ");
    }
  } else if (strchr(input_line, ';') != NULL) {
    /* Start the loop */
    char *arg = strtok(input_line, ";");
    while (arg) {
      argv[argc++] = strdup(arg);
      arg = strtok(NULL, ";");
    }
  } else {
    argv[argc++] = strdup(input_line);
  }

  free(input_line);
  if (PRINT_STDOUT_BIT2()) {
    fprintf(stdlog, "%s/%s:%d argc=%d calloc_len=%u\n", __FILE__, __FUNCTION__, __LINE__,
            argc, (unsigned)calloc_len);
    {
      int i;
      for(i=0; i <= argc;i++) {
        fprintf(stdlog, "%s/%s:%d argv[%d]=\"%s\"\n", __FILE__, __FUNCTION__, __LINE__,
                i, argv[i] ? argv[i] : "NULL");
      }
    }
  }

  return argc;
}

/*****************************************************************************/
int handle_input_line(int socket_fd, const char *input_line, int had_cr, int had_lf)
{
  static const char *seperator_seperator = ";";
  static const char *terminator_terminator = "\n";
  const char *this_stSettings_iTimeOut_str_s = ".THIS.stSettings.iTimeOut=";

  static unsigned int counter;

  const char **my_argv = NULL;
  int argc = create_argv(input_line, had_cr, had_lf, (const char*** )&my_argv);
  const char *argv1 = (argc > 1) ? my_argv[1] : "";

  if (!strncmp(argv1, this_stSettings_iTimeOut_str_s, strlen(this_stSettings_iTimeOut_str_s))) {
    const char *myarg_1 = &argv1[strlen(this_stSettings_iTimeOut_str_s)];
    int timeout;
    int nvals;
    nvals = sscanf(myarg_1, "%d", &timeout);
    if (nvals == 1) {
      int res = socket_set_timeout(socket_fd, timeout);
      cmd_buf_printf("%s%s%s",
                     res ? "Error" : "OK",
                     seperator_seperator,
                     terminator_terminator);
    } else {
      cmd_buf_printf("Error nvals=%d %s%s",
                     nvals,
                     seperator_seperator,
                     terminator_terminator);
    }
  }
  else if ((argc > 1) && (0 == strcmp(argv1, "bye"))) {
    fprintf(stdlog, "%s/%s:%d bye\n", __FILE__, __FUNCTION__, __LINE__);
    return 1;
  }
  else if ((argc > 1) && (0 == strcmp(argv1, "kill"))) {
    exit(0);
  }
  else if (cmd_TCPsim(argc, my_argv)) {
    ; /* TCPSIM command */
  }
  else if (argv1[0] == 'h' ||
           argv1[0] == '?') {
    fd_printf_crlf(socket_fd, had_cr,
                   "Valid commands :\n");
    fd_printf_crlf(socket_fd, had_cr,
                   "bye            : Bye\n"
                   "kill           : exit(0)\n");
  }
  else if (argc > 2){
    fd_printf_crlf(socket_fd, had_cr,"error(%s:%d): invalid command (%s)\n",
                   __FILE__, __LINE__,  argv1);
  }
  else if (argc == 1) {
    /* Just a return, print a prompt */
  }
  {
    int i;
    for (i=0; i < argc; i++)
    {
      free((void *)my_argv[i]);
    }
    free(my_argv);
  }
  if (PRINT_STDOUT_BIT2()) {
    fprintf(stdlog, "%s/%s:%d (%u)\n",
            __FILE__, __FUNCTION__, __LINE__,
            counter++);
  }
  {
    int flags = had_cr ? PRINT_ADD_CR : 0;
    const char *buf = get_buf();

    fd_printf_crlf(socket_fd, flags, "%s", buf);
    clear_buf();
  }

  return 0;
}

