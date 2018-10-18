/* Nokia 5110 LCD AVR Library
 * Copyright (C)  2017 Sergey Shelepin.
 *
 * Based on lib: 
 * Written by Sergey Denisov aka LittleBuster (DenisovS21@gmail.com)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public Licence
 * as published by the Free Software Foundation; either version 3
 * of the Licence, or (at your option) any later version.
 *
 * Original library written by SkewPL, http://skew.tk
 */

#ifndef __NOKIA_5110_H__
#define __NOKIA_5110_H__

#include <avr/pgmspace.h>
#include <stdint.h>

#define PIC_NO_VALVE         0
#define PIC_VALVE_CONNECTED  1
#define PIC_VALVE_READY      2
#define PIC_VALVE_WATERING   3
#define PIC_VALVE_DELAYED    4
#define PIC_CONFO_SLOT       5
#define PIC_REPEAT_TYPE      6       
#define PIC_OK			     7
#define PIC_DISCARD          8
#define PIC_SAND_WATCH       9
#define PIC_SHIFTED         10
#define PIC_CANCELED        11
#define PIC_ACTIVE          12
#define PIC_CORRECTED       13
#define PIC_SHIFT_COR       14
#define PIC_SCROLL_UP       15
#define PIC_SCROLL_DOWN     16

#define SS_SINGLE			0    // string style
#define SS_BOLD				1

/*
 * LCD's port
 */
#define PORT_LCD PORTC
#define DDR_LCD DDRC

/*
 * LCD's pins
 */
#define LCD_RST PC0
#define LCD_SCE PC1
#define LCD_DC  PC2
// #define LCD_DIN PB3 
// #define LCD_CLK PB5

#define LCD_CONTRAST 0x40

/*
 * Must be called once before any other function, initializes display
 */
void nokia_lcd_init(void);

/*
 * Clear screen
 */
void nokia_lcd_clear(void);

/**
 * Power of display
 * @lcd: lcd nokia struct
 * @on: 1 - on; 0 - off;
 */
void nokia_lcd_power(uint8_t on);

/**
 * Set single pixel
 * @x: horizontal pozition
 * @y: vertical position
 * @value: show/hide pixel
 */
void nokia_lcd_set_pixel(uint8_t x, uint8_t y, uint8_t value);

/**
 * Draw single char with 1-6 scale
 * @code: char code
 * @scale: size of char
 */
void nokia_lcd_write_char(char code, uint8_t scale);

/**
 * Draw string. Example: writeString("abc",3);
 * @str: sending string
 * @scale: size of text
 */
void nokia_lcd_write_string(const char *str, uint8_t scale);
void nokia_lcd_write_string_P(const char *str);

/**
 * Set cursor position
 * @x: horizontal position
 * @y: vertical position
 */
void nokia_lcd_set_cursor(uint8_t x, uint8_t y);

/*
 * Render screen to display
 */
void nokia_lcd_render(void);
void nokia_lcd_set_byte(uint8_t b, uint8_t value);
void nokia_lcd_put_digit(uint8_t dig);

void nokia_lcd_put_pic(uint8_t x, uint8_t y, uint8_t pic_type);
void nokia_lcd_put_char_3x7(uint8_t x, uint8_t y, char code);
void nokia_lcd_put_char_3x5(uint8_t x, uint8_t y, char code);
void nokia_lcd_draw_hline(uint8_t x1, uint8_t x2, uint8_t y);
void nokia_lcd_draw_vline(uint8_t y1, uint8_t y2, uint8_t x);
void nokia_lcd_draw_dot_hline(uint8_t x1, uint8_t x2, uint8_t y);
void nokia_lcd_draw_dot_vline(uint8_t y1, uint8_t y2, uint8_t x);
void nokia_lcd_put_str(uint8_t x, uint8_t y, char* str);
void nokia_lcd_put_str_3x5(uint8_t x, uint8_t y, char* str);
void nokia_lcd_put_str_3x5_P(uint8_t x, uint8_t y, const char* str);
void nokia_lcd_set_pixel(uint8_t x, uint8_t y, uint8_t value);
void nokia_lcd_invert_area(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void nokia_lcd_put_screen(uint8_t num);
void  nokia_lcd_draw_rect(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void nokia_lcd_put_string(const char *str, uint8_t string_style);
void nokia_lcd_write_string_bold_P(const char *str);
#endif
