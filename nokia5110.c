/* Nokia 5110 LCD AVR Library
 * Copyright (C)  2017 Sergey Shelepin.
 *
 * Based on lib
 * written by Sergey Denisov aka LittleBuster (DenisovS21@gmail.com)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public Licence
 * as published by the Free Software Foundation; either version 3
 * of the Licence, or (at your option) any later version.
 *
 * Original library written by SkewPL, http://skew.tk
 */

#include "nokia5110.h"


#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include "nokia5110_chars.h"
#include "nokia5110_pics.h"

extern uint8_t SPI_MasterTransmit(uint8_t data);

extern void UARTPrint( const char *str );
extern void UARTPrintln( const char *str );
extern void UARTPrintUint(uint8_t ui, uint8_t raddix);
extern void UARTPrintUint_M(uint8_t ui, uint8_t raddix, uint8_t min_cp);
extern void UARTPrintBit(uint8_t byte_val, const char *bit_name, uint8_t bit_num);

#define LCD_BUF_LEN 504


static struct {
    /* screen byte massive */
    uint8_t screen[LCD_BUF_LEN];

    /* cursor position */
    uint8_t cursor_x;
    uint8_t cursor_y;

} nokia_lcd = {
    .cursor_x = 0,
    .cursor_y = 0
};

/**
 * Sending data to LCD
 * @bytes: data
 * @is_data: transfer mode: 1 - data; 0 - command;
 */
static void write(uint8_t bytes, uint8_t is_data)
{
	/* Enable controller */
	PORT_LCD &= ~(1 << LCD_SCE);

	/* We are sending data */
	if (is_data)
		PORT_LCD |= (1 << LCD_DC);
	/* We are sending commands */
	else
		PORT_LCD &= ~(1 << LCD_DC);

	SPI_MasterTransmit(bytes);

	/* Disable controller */
	PORT_LCD |= (1 << LCD_SCE);
}

static void write_cmd(uint8_t cmd)
{
	write(cmd, 0);
}

static void write_data(uint8_t data)
{
	write(data, 1);
}

/*
 * Public functions
 */

void nokia_lcd_init(void)
{
	register unsigned i;
	/* Set pins as output */
	DDR_LCD |= (1 << LCD_SCE);
	DDR_LCD |= (1 << LCD_RST);
	DDR_LCD |= (1 << LCD_DC);
// 	DDR_LCD |= (1 << LCD_DIN);
// 	DDR_LCD |= (1 << LCD_CLK);

	/* Reset display */
	PORT_LCD |= (1 << LCD_RST);
	PORT_LCD |= (1 << LCD_SCE);
	_delay_ms(10);
	PORT_LCD &= ~(1 << LCD_RST);
	_delay_ms(70);
	PORT_LCD |= (1 << LCD_RST);

	/*
	 * Initialize display
	 */
	/* Enable controller */
	PORT_LCD &= ~(1 << LCD_SCE);
	/* -LCD Extended Commands mode- */
	write_cmd(0x21);
	/* LCD bias mode 1:48 */
	write_cmd(0x13);
	/* Set temperature coefficient */
	write_cmd(0x06);
	/* Default VOP (3.06 + 66 * 0.06 = 7V) */
	write_cmd(0xC2);
	/* Standard Commands mode, powered down */
	write_cmd(0x20);
	/* LCD in normal mode */
	write_cmd(0x09);

	/* Clear LCD RAM */
	write_cmd(0x80);
	write_cmd(LCD_CONTRAST);
	for (i = 0; i < 504; i++)
		write_data(0x00);

	/* Activate LCD */
	write_cmd(0x08);
	write_cmd(0x0C);
}

void nokia_lcd_clear(void)
{
	register unsigned i;
	/* Set column and row to 0 */
	write_cmd(0x80);
	write_cmd(0x40);
	/*Cursor too */
	nokia_lcd.cursor_x = 0;
	nokia_lcd.cursor_y = 0;
	/* Clear everything (504 bytes = 84cols * 48 rows / 8 bits) */
	for(i = 0;i < 504; i++)
		nokia_lcd.screen[i] = 0x00;
}

void nokia_lcd_power(uint8_t on)
{
	write_cmd(on ? 0x20 : 0x24);
}

void nokia_lcd_set_pixel(uint8_t x, uint8_t y, uint8_t value) {
	
	register uint16_t byte_num = y/8*84 + x;
	
	if (byte_num >= LCD_BUF_LEN) {
		return;
	}

	uint8_t *byte = &nokia_lcd.screen[byte_num];
	
	register uint8_t bit = (1 << (y % 8));
	
	if (value)
		*byte |= bit;
	else
		*byte &= ~bit;

}

void nokia_lcd_invert_pixel(uint8_t x, uint8_t y) {

	register uint16_t byte_num = y/8*84 + x;
	
	if (byte_num >= LCD_BUF_LEN) {
		return;
	}
	
	uint8_t *byte = &nokia_lcd.screen[byte_num];	
	
	register uint8_t bit = (1 << (y % 8));
	
	if(*byte & bit) {
		*byte &= ~bit;		
	} else {
		*byte |= bit;
	}
	
}

void nokia_lcd_set_byte(uint8_t b, uint8_t value) {
	nokia_lcd.screen[b] = value;
}

void nokia_lcd_inc_cursor(uint8_t scale) {
	
	nokia_lcd.cursor_x += 5*scale + 1;
	if (nokia_lcd.cursor_x >= 84) {
		nokia_lcd.cursor_x = 0;
		nokia_lcd.cursor_y += 7*scale + 1;
	}
	if (nokia_lcd.cursor_y >= 48) {
		nokia_lcd.cursor_x = 0;
		nokia_lcd.cursor_y = 0;
	}

}

void nokia_lcd_write_char(char code, uint8_t scale)
{
	register uint8_t x, y;

	for (x = 0; x < 5*scale; x++)
		for (y = 0; y < 7*scale; y++)
			if (pgm_read_byte(&CHARSET[code-32][x/scale]) & (1 << y/scale))
				nokia_lcd_set_pixel(nokia_lcd.cursor_x + x, nokia_lcd.cursor_y + y, 1);
//			else
//				nokia_lcd_set_pixel(nokia_lcd.cursor_x + x, nokia_lcd.cursor_y + y, 0);

	nokia_lcd_inc_cursor(scale);

}

void nokia_lcd_write_string_P(const char *str) {
	char ch;
	while (0 != (ch = pgm_read_byte(str++))) {
		nokia_lcd_write_char(ch, 1);
	}
}




void nokia_lcd_write_string(const char *str, uint8_t scale)
{
	while(*str)
		nokia_lcd_write_char(*str++, scale);
}

//---------------------------------------------------------------------------------------------
void nokia_lcd_put_char(char code) {
	register uint8_t x, y;

	for (x = 0; x < 5; x++) {
		register char byte = pgm_read_byte(&CHARSET[code-32][x]);
		for (y = 0; y < 7; y++) {
			if (byte & (1 << y)) {
				nokia_lcd_set_pixel(nokia_lcd.cursor_x + x, nokia_lcd.cursor_y + y, 1);
			}
		}
	}
	
}
//---------------------------------------------------------------------------------------------
void nokia_lcd_write_string_bold_P(const char *str) {
	char ch;
	while (0 != (ch = pgm_read_byte(str++))) {
		nokia_lcd_put_char(ch);
		++nokia_lcd.cursor_y;
		nokia_lcd_put_char(ch);
		nokia_lcd_inc_cursor(1);
	}
}
//---------------------------------------------------------------------------------------------


void nokia_lcd_put_string(const char *str, uint8_t string_style) {

	while(*str) {
		register char ch = *str++;
//		++str;
		
		nokia_lcd_put_char(ch);
	
		if(string_style == SS_BOLD)	{
			++nokia_lcd.cursor_x;
			nokia_lcd_put_char(ch);
		}
		nokia_lcd.cursor_x += 5 + 1;
		if (nokia_lcd.cursor_x >= 84) {
			nokia_lcd.cursor_x = 0;
			nokia_lcd.cursor_y += 7 + 1;
		}
		if (nokia_lcd.cursor_y >= 48) {
			nokia_lcd.cursor_x = 0;
			nokia_lcd.cursor_y = 0;
		}	
	}
	
}
//---------------------------------------------------------------------------------------------



void nokia_lcd_set_cursor(uint8_t x, uint8_t y)
{
	nokia_lcd.cursor_x = x;
	nokia_lcd.cursor_y = y;
}

void nokia_lcd_render(void) {
	register unsigned i;
	/* Set column and row to 0 */
	write_cmd(0x80);
	write_cmd(0x40);

	/* Write screen to display */
	for (i = 0; i < 504; i++)
		write_data(nokia_lcd.screen[i]);
}

//------------------------------------------------------------------------------
//  rectangle are invertion
//------------------------------------------------------------------------------
void nokia_lcd_invert_area(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
	while(x1 <= x2) {
		register uint8_t _y1 = y1;
		while (_y1 <= y2) {
			nokia_lcd_invert_pixel(x1, _y1++);
		}
		x1++;
	}
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void nokia_lcd_put_str(uint8_t x, uint8_t y, char* str) {
	
	#define char_width  3	
	
	while(*str) {
		nokia_lcd_put_char_3x7(x, y, *str);
		x += char_width + 1;
		++str;
	}
}
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void nokia_lcd_put_str_3x5(uint8_t x, uint8_t y, char* str) {                        //@todo: refactor this copy paste....
	
	#define char_width  3
	
	while(*str) {
		nokia_lcd_put_char_3x5(x, y, *str);
		x += char_width + 1;
		++str;
	}
}
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void nokia_lcd_put_str_3x5_P(uint8_t x, uint8_t y, const char* str) {                        //@todo: refactor this copy paste....
	
	#define char_width  3
	char ch;
	while (0 != (ch = pgm_read_byte(str++))) {
		nokia_lcd_put_char_3x5(x, y, ch);
		x += char_width + 1;
	}
}
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void nokia_lcd_put_char_3x7(uint8_t x, uint8_t y, char code) {

	const uint8_t cwi = 3;
	const uint8_t che = 7;
	
	for(uint8_t i = 0; i < cwi; i++) {
		char ch = pgm_read_byte(CHARS_3x7 + (code-65)*cwi + i);
		
		for (uint8_t j = 0 ; j < che; j++) {
			register uint8_t val = (uint8_t) ch & 1;
			nokia_lcd_set_pixel(x +i, y + j, val);
			ch >>= 1;
		}
	}
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void nokia_lcd_put_char_3x5(uint8_t x, uint8_t y, char code) {

	const uint8_t cwi = 3;
	const uint8_t che = 7;
	
	if(code > 90) {                         // to upper case
		code -= 32;
	}
	
	uint8_t cas;                            // character address shift
	switch(code){
		case 'A':
			cas = 0;
		break;
		case 'F':
			cas = 1;
		break;
		case 'I':
			cas = 2;
		break;
		case 'N':
			cas = 3;
		break;
		case 'O':
			cas = 4;
		break;
		case 'P':
			cas = 5;
		break;
		case 'R':
			cas = 6;
		break;		
		case 'S':
			cas = 7;
		break;		
		case 'T':
			cas = 8;
		break;
		case 'Z':
			cas = 9;
		break;
		case 'M':
			cas = 10;
		break;
		case 'U':
			cas = 11;
		break;		
		case 'E':
			cas = 12;
		break;		
		case 'W':
			cas = 13;
		break;		
		case 'D':
			cas = 14;
		break;		
		case 'H':
			cas = 15;
		break;		
		default:
			cas = 9;
		break;		

	}
	
	for(uint8_t i = 0; i < cwi; i++) {
		char ch = pgm_read_byte(CHARS_3x5 + cas*cwi + i);
		for (uint8_t j = 0 ; j < che; j++) {
			register uint8_t val = (uint8_t) ch & 1;
			nokia_lcd_set_pixel(x +i, y + j, val);
			ch >>= 1;
		}
	}
}

//-----------------------------------------------------------------------
//  put 1 byte height picture
//-----------------------------------------------------------------------
void nokia_lcd_put_small_pic(uint8_t x, uint8_t y, PGM_P pic_ptr, uint8_t len) { 
	
	for (uint8_t i = 0; i < len; i++ ) {

		uint8_t pic_byte = pgm_read_byte(pic_ptr + i);
		
		for(uint8_t j = 0; j < 8; j++ ) {
			nokia_lcd_set_pixel(x + i, y + j, pic_byte & 1 );
			pic_byte >>= 1;
		}
	}
}
//-----------------------------------------------------------------------
//
//-----------------------------------------------------------------------
void nokia_lcd_put_screen(uint8_t num) {

	PGM_P sc_ptr = WATER_DROP_SCREEN_1;	
	
	switch (num) {
		case 1:
			sc_ptr = WATER_DROP_SCREEN_2;
		break;
		case 2:
			sc_ptr = WATER_DROP_SCREEN_3;
		break;	
	}
	
	for(uint16_t i = 0; i < LCD_BUF_LEN; i++) {
		nokia_lcd.screen[i] = pgm_read_byte(sc_ptr + i);
	}
	
}

//-----------------------------------------------------------------------
//
//-----------------------------------------------------------------------
void nokia_lcd_put_pic(uint8_t x, uint8_t y, uint8_t pic_type) { 
	
	PGM_P pic = 0;
	uint8_t len = 0;
	
	switch(pic_type) {
		case PIC_NO_VALVE:
		default:
			pic = EMPTY_PIC_6x8;
			len = sizeof(EMPTY_PIC_6x8);
		break;

		case PIC_VALVE_CONNECTED:
			pic = PLUG_PIC_5x8;
			len = sizeof(PLUG_PIC_5x8);
		break;

		case PIC_VALVE_READY:
			pic = TAP_PIC_6x8;
			len = sizeof(TAP_PIC_6x8);
		break;
		
		case PIC_VALVE_WATERING:
			pic = FONTAIN_PIC_6x8;
			len = sizeof(FONTAIN_PIC_6x8);
		break;

		case PIC_VALVE_DELAYED:
			pic = SAND_WATCH_PIC_5x8;
			len = sizeof(SAND_WATCH_PIC_5x8);
		break;
		
		case PIC_CONFO_SLOT:
			pic = TASK_PIC_9x8;
			len = sizeof(TASK_PIC_9x8);
		break;

		case PIC_REPEAT_TYPE:
			pic = REPEAT_PIC_13x8;
			len = sizeof(REPEAT_PIC_13x8);
		break;

		case PIC_OK:
			pic = OK_PIC_10x8;
			len = sizeof(OK_PIC_10x8);
		break;

		case PIC_DISCARD:
			pic = DISCARD_PIC_9x8;
			len = sizeof(DISCARD_PIC_9x8);
		break;

		case PIC_SAND_WATCH:
			pic = SAND_WATCH_PIC_5x8;
			len = sizeof(SAND_WATCH_PIC_5x8);
		break;
		
		case PIC_SHIFTED:
			pic = ARROW_PIC_5x8;
			len = sizeof(ARROW_PIC_5x8);
		break;

		case PIC_CORRECTED:
			pic = RIGHT_LIMIT_PIC_5x8;
			len = sizeof(RIGHT_LIMIT_PIC_5x8);
		break;

		case PIC_ACTIVE:
			pic = FONTAIN_SMALL_PIC_5x8;
			len = sizeof(FONTAIN_SMALL_PIC_5x8);
		break;

		case PIC_CANCELED:
			pic = CROSS_PIC_5x8;
			len = sizeof(CROSS_PIC_5x8);
		break;
		
		case PIC_SHIFT_COR:
			pic = ARROW_RIGTH_LIMIT_PIC_5x8;
			len = sizeof(ARROW_RIGTH_LIMIT_PIC_5x8);
		break;

		case PIC_SCROLL_UP:
			pic = UP_SCROLL_ARROW_PIC_5x8;
			len = sizeof(UP_SCROLL_ARROW_PIC_5x8);
		break;

		case PIC_SCROLL_DOWN:
			pic = DOWN_SCROLL_ARROW_PIC_5x8;
			len = sizeof(DOWN_SCROLL_ARROW_PIC_5x8);
		break;		

	}
	nokia_lcd_put_small_pic(x, y, pic, len);
}


void nokia_lcd_draw_hline(uint8_t x1, uint8_t x2, uint8_t y) {
	do {
		nokia_lcd_set_pixel(x1, y, 1);
	} while(x1++ < x2);
}


void nokia_lcd_draw_vline(uint8_t y1, uint8_t y2, uint8_t x) {
	do {
		nokia_lcd_set_pixel(x, y1, 1);
	} while (y1++ < y2);
}

void nokia_lcd_draw_dot_hline(uint8_t x1, uint8_t x2, uint8_t y) {
	register uint8_t dot = 1;
	do {
		nokia_lcd_set_pixel(x1, y, 1 & dot++);
	} while(x1++ < x2);	
}

void nokia_lcd_draw_dot_vline(uint8_t y1, uint8_t y2, uint8_t x){
	register uint8_t dot = 1;	
	do {
		nokia_lcd_set_pixel(x, y1, 1 & dot++);
	} while (y1++ < y2);
}

void  nokia_lcd_draw_rect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
	nokia_lcd_draw_hline(x1, x2, y1);
	nokia_lcd_draw_hline(x1, x2, y2);	
	nokia_lcd_draw_vline(y1, y2, x1);
	nokia_lcd_draw_vline(y1, y2, x2);	
	
}