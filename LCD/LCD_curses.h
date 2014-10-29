#ifndef LCD_CURSES_H
#define LCD_CURSES_H

#include "LCD/LCD_config.h"

#define LCD_ERR -1
#define LCD_OK   0

int LCD_addch(LCD_InitTypeDef*, uint16_t);
int LCD_addstr(LCD_InitTypeDef*, const char*);
int LCD_move(LCD_InitTypeDef*, int, int);

#endif
