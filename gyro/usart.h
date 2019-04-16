
#ifdef DEBUG
#define debug(str, ...) usart_print(str, ##__VA_ARGS__)
#else
#define debug(str, ...)
#endif

void usart_init();

void usart_putc(unsigned char data);

void usart_putstr(const char *str);

void usart_print(const char* str, ...);