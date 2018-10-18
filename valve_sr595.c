/*
 * valve_sr595.c
 *
 * Created: 12.06.2017 13:16:37
 *  Author: USER
 */ 
#include "valve_sr595.h"
#include <util/delay.h>

extern uint8_t SPI_MasterTransmit(uint8_t data);

void sr_update(uint8_t val) {
	sr_set_latch_low();
	SPI_MasterTransmit(val);
	sr_set_latch_high();
}


void valve_sr_init() {
	SR_LATCH_DDR |= _BV(SR_LATCH_PIN);
	SR_OE_DDR    |= _BV(SR_OE_PIN);
	sr_set_latch_high();
	sr_set_oe_high();
	sr_update(0);
	_delay_ms(10);
	valve_enable_sr(1);
}

void valve_open(uint8_t valve_num) {
	sr_update(_BV(valve_num));
}


void valve_close_all() {
	sr_update(0);
}

void valve_enable_sr(uint8_t enable) {
	if (enable)	{
		sr_set_oe_low();
	} else {
		sr_set_oe_high();		
	}
	
}