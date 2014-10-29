#include <stdio.h>
#include <string.h>
#include "clib/__internal__.h"
#include "LCD/LCD_curses.h"
#include "LCD/LCD_stdio.h"

typedef struct {
	LCD_InitTypeDef *lcd;
	int count;
} LCD_printf_data;

static int _putc_LCD_printf(void *param, char c)
{
	LCD_printf_data *data = (LCD_printf_data*)param;
	int r = LCD_addch(data->lcd, c);

	if (r == LCD_OK)
		data->count++;

	return r;
}

static int _puts_LCD_printf(void *param, const char *s)
{
	LCD_printf_data *data = (LCD_printf_data*)param;
	int r = LCD_addstr(data->lcd, s);

	if (r == LCD_OK)
		data->count += strlen(s);

	return r;
}

int LCD_printf(LCD_InitTypeDef *lcd, const char *fmt, ...)
{
	int result;
	va_list arg_list;

	va_start(arg_list, fmt);
	result = LCD_vprintf(lcd, fmt, arg_list);
	va_end(arg_list);

	return result;
}

int LCD_putc(LCD_InitTypeDef *lcd, int c)
{
	if (LCD_addch(lcd, c) != LCD_OK)
		return EOF;
	return (unsigned char)c;
}

int LCD_puts(LCD_InitTypeDef *lcd, const char *s)
{
	int r = LCD_addstr(lcd, s);

	if (r == LCD_OK)
		r = LCD_addch(lcd, '\n');
	return r;
}

int LCD_vprintf(LCD_InitTypeDef *lcd, const char *fmt, va_list arg_list)
{
	LCD_printf_data data = {.lcd = lcd, .count = 0};

	vprintf_core(fmt, arg_list, _putc_LCD_printf, _puts_LCD_printf, &data);
	return data.count;
}
