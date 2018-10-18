/*
 * nokia5110_dig_gen.h
 *
 * Created: 06.06.2017 22:36:51
 *  Author: USER
 */ 


#ifndef NOKIA5110_DIG_GEN_H_
#define NOKIA5110_DIG_GEN_H_

#define SL_FONT_4x7   0
#define SL_FONT_3x5   1
#define DL_FONT_8x14  2
#define DL_FONT_7x12  3

void    nokia_lcd_gen_dig(uint8_t x, uint8_t y, uint8_t digit);
uint8_t nokia_lcd_print_uint(uint8_t x, uint8_t y, uint8_t ui, uint8_t dig_qty );

void nokia_lcd_set_dig_font(uint8_t sl_font_type);
void nokia_lcd_gen_slash(uint8_t x, uint8_t y);



#endif /* NOKIA5110_DIG_GEN_H_ */