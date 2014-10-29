#ifndef LCD_STDIO_H
#define LCD_STDIO_H

#include <stdarg.h>
#include "LCD/LCD_config.h"

int LCD_printf(LCD_InitTypeDef*, const char*, ...);
int LCD_putc(LCD_InitTypeDef*, int);
int LCD_puts(LCD_InitTypeDef*, const char*);
int LCD_vprintf(LCD_InitTypeDef*, const char*, va_list);

#endif
