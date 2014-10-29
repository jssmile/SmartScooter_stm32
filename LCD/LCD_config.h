#ifndef LCDCONFIG_H
#define LCDCONFIG_H

#include <stdint.h>
#include "main.h"
/*
	LCD Library for STM32Fx
*/

typedef struct {
	uint16_t RS_Pin;
	uint16_t RW_Pin;
	uint16_t E_Pin;

	/* DB 0 to 7 */
	uint16_t DB_Pins[8];
	GPIO_TypeDef *GPIO;

	/* window size */
	uint8_t max_col;
	uint8_t max_row;

	/* current cursor position */
	uint8_t col;
	uint8_t row;
} LCD_InitTypeDef;

void LCD_Init(LCD_InitTypeDef *);

/* commands */
enum LCD_CMD {
	LCD_CLEARDISPLAY	=0x01,
	LCD_RETURNHOME		=0x02,
	LCD_ENTRYMODESET	=0x04,
	LCD_DISPLAYCONTROL	=0x08,
	LCD_CURSORSHIFT		=0x10,
	LCD_FUNCTIONSET		=0x20,
	LCD_SETCGRAMADDR	=0x40,
	LCD_SETDDRAMADDR	=0x80
};

/* flags for display entry mode */
enum LCD_ENTRY_FLAGS {
	LCD_ENTRYRIGHT		=0x00,
	LCD_ENTRYLEFT		=0x02,
	LCD_ENTRYSHIFTINCREMENT	=0x01,
	LCD_ENTRYSHIFTDECREMENT	=0x00
};

/* flags for display on/off control */
enum LCD_DISPLAY {
	LCD_DISPLAYON		=0x04,
	LCD_DISPLAYOFF		=0x00,
	LCD_CURSORON		=0x02,
	LCD_CURSOROFF		=0x00,
	LCD_BLINKON		=0x01,
	LCD_BLINKOFF		=0x00
};

/* flags for display/cursor shift */
enum LCD_MOVE {
	LCD_DISPLAYMOVE		=0x08,
	LCD_CURSORMOVE		=0x00,
	LCD_MOVERIGHT		=0x04,
	LCD_MOVELEFT		=0x00
};

/* flags for function set */
enum LCD_FUNSET {
	LCD_8BITMODE		=0x10,
	LCD_4BITMODE		=0x00,
	LCD_2LINE		=0x08,
	LCD_1LINE		=0x00,
	LCD_5x10DOTS		=0x04,
	LCD_5x8DOTS		=0x00
};
#endif
