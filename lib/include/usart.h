#ifndef _USART_H
#define _USART_H
#include "sys.h"

void usart1_init(void);
void usart2_init(void);
void usart3_init(void);
void myputc(uint8_t c);
void myputbuf(uint8_t *buffer, uint16_t len);
void myputs(uint8_t *s);


void usart3_putc(uint8_t c);
void usart3_puts(uint8_t *s);
void usart3_putbuf(uint8_t *buffer, uint16_t len);
#endif
