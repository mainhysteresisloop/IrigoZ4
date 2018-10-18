/*
 * valve_sr595.h
 *
 * Created: 12.06.2017 13:16:11
 *  Author: USER
 */ 


#ifndef VALVE_SR595_H_
#define VALVE_SR595_H_

#include <avr/io.h>

#define VALVE_1   1
#define VALVE_2   2
#define VALVE_3   3
#define VALVE_4   4

#define SR_LATCH_PORT  PORTB
#define SR_LATCH_DDR   DDRB
#define SR_LATCH_PIN   PB2

#define SR_OE_PORT  PORTB
#define SR_OE_DDR   DDRB
#define SR_OE_PIN   PB7


#define sr_set_latch_low()   SR_LATCH_PORT &= ~_BV(SR_LATCH_PIN)
#define sr_set_latch_high()  SR_LATCH_PORT |=  _BV(SR_LATCH_PIN)

#define sr_set_oe_low()   SR_OE_PORT &= ~_BV(SR_OE_PIN)
#define sr_set_oe_high()  SR_OE_PORT |=  _BV(SR_OE_PIN)


void valve_sr_init();
void valve_open(uint8_t valve_num);
void valve_close_all();
void valve_enable_sr(uint8_t enable);


#endif /* VALVE_SR595_H_ */