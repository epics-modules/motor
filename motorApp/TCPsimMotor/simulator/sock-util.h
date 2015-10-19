#include <stdio.h>

extern int handle_input_line(int socket_fd, const char *input_line, int had_cr, int had_lf);
extern int get_listen_socket(const char *listen_port_asc);
extern int handle_input_line(int socket_fd, const char *input_line, int had_cr, int had_lf);
extern void send_to_socket(int fd, const char *buf, unsigned len);
extern int socket_set_timeout(int fd, int seconds);
void socket_loop(void);


#define PRINT_ADD_CR (1<<0)
#define PRINT_OUT    (1<<1)
extern void fd_printf_crlf(int fd, int add_cr, const char *format, ...)__attribute__((format (printf, 3, 4)));

extern FILE *stdlog;
