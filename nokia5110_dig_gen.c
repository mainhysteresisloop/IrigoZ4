/*
 * nokia5110_dig_gen.c
 *
 * Created: 06.06.2017 22:37:01
 *  Author: USER
 */ 

#define FALSE 0
#define TRUE 1

#define LS_SINGLE_LINE 0
#define LS_DOUBLE_LINE 1

#include <avr/io.h>
#include "nokia5110_dig_gen.h"

extern void nokia_lcd_set_pixel(uint8_t x, uint8_t y, uint8_t value);
extern void nokia_lcd_draw_hline(uint8_t x1, uint8_t x2, uint8_t y);
extern void nokia_lcd_draw_vline(uint8_t y1, uint8_t y2, uint8_t x);
extern void UARTPrint( const char *str );

//font size parameters for double line digits
uint8_t dig_height;
uint8_t dig_width;
uint8_t line_style;
uint8_t dig_interval;

uint8_t get_dig_line_bits(uint8_t digit) {
	
	//bit sequence: 0, hh, hm, hl, lh, ll, rh, rl	
	
	switch(digit) {
		case 0:
			return 0b01011111;
		break;
		case 1:
			return 0b00000011;
		break;
		case 2:
			return 0b01110110;
		break;
		case 3:
			return 0b01110011;
		break;
		case 4:
			return 0b00101011;
		break;
		case 5:
			return 0b01111001;
		break;
		case 6:
			return 0b01111101;
		break;
		case 7:
			return 0b01000011;
		break;
		case 8:
			return 0b01111111;
		break;
		case 9:
			return 0b01111011;
		break;
		default:
			return 0;
		break;
	}	
	
}
//-----------------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------------
void nokia_lcd_set_dig_font(uint8_t sl_font_type) {

	line_style = LS_SINGLE_LINE;
	dig_interval = 2;	
	
	switch(sl_font_type){
		case SL_FONT_3x5:
			dig_width = 2;
			dig_height = 4;
		break;
		
		case SL_FONT_4x7:
		default:
			dig_width = 3;
			dig_height = 6;
		break;

		case DL_FONT_8x14:
			dig_width = 8;
			dig_height = 14;
			line_style = LS_DOUBLE_LINE;
			dig_interval = 4;			
		break;		
		
		case DL_FONT_7x12:
			dig_width = 7;
			dig_height = 12;
			line_style = LS_DOUBLE_LINE;
			dig_interval = 3;
		break;
		
	}
	
}
//----------------------------------------------------------------------
//  complement digit to double line digit
//----------------------------------------------------------------------
void nokia_lcd_dl_compl(uint8_t x, uint8_t y, uint8_t lines_bits) {
	
	uint8_t h2 = dig_height/2;

	if (lines_bits & 1) {
		nokia_lcd_draw_vline(y + h2, y + dig_height, x + dig_width -1);  // right low
	}

	if ((lines_bits >>= 1 ) & 1) {
		nokia_lcd_draw_vline(y, y + h2, x + dig_width - 1);                 // right-high
	}
	
	if ((lines_bits >>= 1 ) & 1) {
		nokia_lcd_draw_vline(y + h2, y + dig_height, x + 1);                // left-low
	}

	if ((lines_bits >>= 1 ) & 1) {
		nokia_lcd_draw_vline(y, y + h2, x + 1);                             // left-high
	}
	
	if ((lines_bits >>= 1 ) & 1) {
		nokia_lcd_draw_hline(x, x + dig_width, y + dig_height - 1);         // hor-low
	}

	if ((lines_bits >>= 1 ) & 1) {
		nokia_lcd_draw_hline(x, x + dig_width, y + h2 - 1);                 // hor-mid
	}

	if ((lines_bits >>= 1 ) & 1) {
		nokia_lcd_draw_hline(x, x + dig_width, y + 1);                      // hor-hig
	}
}
//-----------------------------------------------------------------------------------------------
// generates single line digit
//-----------------------------------------------------------------------------------------------
void nokia_lcd_gen_dig(uint8_t x, uint8_t y, uint8_t digit) {
	
	uint8_t h2 = dig_height/2;
	
	uint8_t lines_bits = get_dig_line_bits(digit);
	
	if(line_style == LS_DOUBLE_LINE) {
		nokia_lcd_dl_compl(x, y, lines_bits);
	}
	
	if (lines_bits & 1) {
		nokia_lcd_draw_vline(y + h2, y + dig_height, x + dig_width);         // right low
	}

	if ((lines_bits >>= 1 ) & 1) {
		nokia_lcd_draw_vline(y, y + h2, x + dig_width);                     // right-high
	}
	
	if ((lines_bits >>= 1 ) & 1) {
		nokia_lcd_draw_vline(y + h2, y + dig_height, x);                    // left-low
	}

	if ((lines_bits >>= 1 ) & 1) {
		nokia_lcd_draw_vline(y, y + h2, x);                                    // left-high
	}
	
	if ((lines_bits >>= 1 ) & 1) {
		nokia_lcd_draw_hline(x, x + dig_width, y + dig_height);          // hor-low
	}

	if ((lines_bits >>= 1 ) & 1) {
		nokia_lcd_draw_hline(x, x + dig_width, y + h2);                     // hor-mid
	}

	if ((lines_bits >>= 1 ) & 1) {
		nokia_lcd_draw_hline(x, x + dig_width, y);                          // hor-hig
	}	

}

//-----------------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------------
uint8_t nokia_lcd_print_uint(uint8_t x, uint8_t y, uint8_t ui, uint8_t dig_qty ) {
		
	register uint8_t p = dig_width + dig_interval; // dig pos period 
	register uint8_t xpos = 0;
	
	if(dig_qty == 3) {
		nokia_lcd_gen_dig(x, y, ui/100);
		++xpos;
	}
	
	if (dig_qty >= 2) {
		nokia_lcd_gen_dig(x + xpos*p, y, ui/10);
		++xpos;		
	}
	
	nokia_lcd_gen_dig(x + xpos*p, y, ui%10);
	
	return ++xpos*p + x ;
}
//-----------------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------------
void nokia_lcd_gen_sl_slash(uint8_t x, uint8_t y) {
	#define X_SCALER_MAX 2
	uint8_t cur_x = x;
	uint8_t cur_y = y + dig_height;
	register uint8_t x_scaler = X_SCALER_MAX;
	
	do {
		nokia_lcd_set_pixel(cur_x, cur_y--, 1);
		--x_scaler;
		if (!x_scaler) {
			++cur_x;
			x_scaler = X_SCALER_MAX;
		}
		
	} while(cur_x <= x + dig_width && cur_y >= y);
}

//-----------------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------------
void nokia_lcd_gen_slash(uint8_t x, uint8_t y) {
	
	nokia_lcd_gen_sl_slash(x, y);
	
	if (line_style == LS_DOUBLE_LINE) {
		nokia_lcd_gen_sl_slash(x + 1, y);		
	}
	
}



