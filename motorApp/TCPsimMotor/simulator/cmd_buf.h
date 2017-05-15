


__attribute__((format (printf,2,3)))
extern void cmd_buf_printf_crlf(int flags, const char *fmt, ...);

__attribute__((format (printf,1,2)))
extern void cmd_buf_printf(const char *fmt, ...);

void add_to_buf(const char *add_txt, size_t add_len);
char *get_buf(void);
void clear_buf(void);
