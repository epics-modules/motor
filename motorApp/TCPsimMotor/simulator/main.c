#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <libgen.h>
#include <stdint.h>
#include <stdbool.h>

#include <errno.h>
#include "sock-util.h"
#if (!defined _WIN32 && !defined __WIN32__ && !defined __CYGWIN__)
  #include <signal.h>
#endif

#include "sock-util.h"
#include "logerr_info.h"

/* defines */
/*****************************************************************************/

/*****************************************************************************/
/* Static variables */
/*****************************************************************************/

unsigned int debug_print_flags;
unsigned int die_on_error_flags = 1;

FILE *stdlog;

void help_and_exit(const char *msg)
{
  if (msg) {
    fprintf(stderr, "%s\n", msg);
  }

  fprintf(stderr,
          "Usage    telnet_motor\n"
          "Example: telnet_motor -v \n"
          "Example: telnet_motor -v   1 prints all data received\n"
          "Example: telnet_motor -v   2 prints all data send\n"
          "Example: telnet_motor -v   3 prints all data received or send\n"
          "Example: telnet_motor -v  64 prints the socket events\n"
          "Example: telnet_motor -v 128 prints all data received or send\n"
          "Example:\n");

  exit(1);
}

/*****************************************************************************/

/*****************************************************************************/
int main(int argc, char** argv)
{
#if (!defined _WIN32 && !defined __WIN32__ && !defined __CYGWIN__)
  (void)signal(SIGPIPE, SIG_IGN);
#endif

  if (argc == 3 &&
      !strcmp(argv[1], "-v")) {
    debug_print_flags = atoi(argv[2]);
    if (!debug_print_flags) {
      help_and_exit("debug_print_flags must not be 0");
    }
  } else if (argc == 1) {
    ;
  } else {
    fprintf(stderr, "argc=%d\n", argc);

    help_and_exit("wrong argc");
  }

  stdlog = stdout;
  socket_loop();

  LOGINFO("End %s\n", __FUNCTION__);
  fflush(stdlog);
  return 0;
}
