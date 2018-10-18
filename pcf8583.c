/*
 * pcf8583.c
 *
 * Created: 09.05.2017 18:52:04
 *  Author: USER
 */ 

#include <stdint.h>
#include "pcf8583.h"
#include <avr/interrupt.h>

#define TRUE  1
#define FALSE 0

extern uint8_t i2c_start(uint8_t address);
extern uint8_t i2c_write(uint8_t data);
extern uint8_t i2c_read_ack(void);
extern uint8_t i2c_read_nack(void);
extern uint8_t i2c_transmit(uint8_t address, uint8_t* data, uint16_t length);
extern uint8_t i2c_receive(uint8_t address, uint8_t* data, uint16_t length);
extern uint8_t i2c_writeReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);
extern uint8_t i2c_readReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);
extern void i2c_stop(void);

extern void UARTPrintInt16(uint16_t i, uint8_t raddix);
extern void UARTPrintUint(uint8_t ui, uint8_t raddix);
extern void UARTPrint( const char *str );
extern void UARTPrintln( const char *str );

uint8_t rtc_base_year;

void correct_feb_days(uint8_t year8_t);

uint8_t dec2bcd(uint8_t d) {
	return ((d/10 * 16) + (d % 10));
}

uint8_t bcd2dec(uint8_t b) {
	return ((b/16 * 10) + (b % 16));
}

void pcf8583_set_month_days(rtc_time* rt) {
	*(rtc_days_in_month)     = 31;    //Jan
	*(rtc_days_in_month + 1) = 28;    //Feb
	*(rtc_days_in_month + 2) = 31;    //Mar

	*(rtc_days_in_month + 3) = 30;    //Apr
	*(rtc_days_in_month + 4) = 31;    //May
	*(rtc_days_in_month + 5) = 30;    //Jun

	*(rtc_days_in_month + 6) = 31;    //Jul
	*(rtc_days_in_month + 7) = 31;    //Avg
	*(rtc_days_in_month + 8) = 30;    //Sep
	
	*(rtc_days_in_month + 9) = 31;    //Oct
	*(rtc_days_in_month + 10) = 30;   //Nov
	*(rtc_days_in_month + 11) = 31;   //Dec			
	
	correct_feb_days(rt->year8_t);
// 		31, 28, 31,
// 		30, 31, 30,
// 		31, 31, 30,
// 		31, 30, 31
//	};  
}
//----------------------------------------------------------------------
//  set hours minutes and seconds to pcf8583
//----------------------------------------------------------------------
uint8_t pcf8583_read_byte(uint8_t offset) {
	register uint8_t res;
	i2c_start(PCF8583_ADDR_WRITE);
	i2c_write(offset);
	i2c_start(PCF8583_ADDR_READ);
	res = i2c_read_nack();
	i2c_stop();
	return res;
}

uint8_t pcf8583_write_byte(uint8_t offset, uint8_t value) {
	if(!i2c_start(PCF8583_ADDR_WRITE)) {
		return FALSE;
	}
	i2c_write(offset);
	i2c_write(value);                                        // transform dec to dec 
	i2c_stop();
	return TRUE;
}

//----------------------------------------------------------------------
// read hours, minutes and seconds from pcf8583
// to be used as internal function due cli(), sei() "stack"
//----------------------------------------------------------------------
void pcf8583_get_ss_mi_hh24(rtc_time* rt) {
	if (!i2c_start(PCF8583_ADDR_WRITE)){ 
		return;
	}
	i2c_write(PCF8583_SECONDS_REG);
	i2c_start(PCF8583_ADDR_READ);	
	*((uint8_t*)rt)     = bcd2dec(i2c_read_ack());
	*((uint8_t*)rt + 1) = bcd2dec(i2c_read_ack());
	*((uint8_t*)rt + 2) = bcd2dec(i2c_read_nack());
	i2c_stop();

}

//----------------------------------------------------------------------
// set hours minutes and seconds to pcf8583
// to be used as internal function due cli(), sei() "stack"
//----------------------------------------------------------------------
uint8_t pcf8583_set_ss_mi_hh24(rtc_time* rt) {
	if(!i2c_start(PCF8583_ADDR_WRITE)) {
		return FALSE;
	}
	i2c_write(PCF8583_SECONDS_REG);
	for(uint8_t i = 0; i < 3; i++) {
		i2c_write(dec2bcd(*((uint8_t*)rt + i)));                             // get first 3 member of the rtc_time struct
	}
	i2c_stop();
	return TRUE;
}
//----------------------------------------------------------------------
//  check if year is leap year
//----------------------------------------------------------------------
uint8_t is_leap_year(uint16_t year) {

	if( (year%400 == 0 || year%100 != 0) && (year%4 == 0)) {
		return TRUE;
	}
	return FALSE;
}
//----------------------------------------------------------------------
//  
//----------------------------------------------------------------------
void correct_feb_days(uint8_t year8_t) {
	if (is_leap_year((uint16_t)year8_t + 2000)) {
		rtc_days_in_month[1] = 29;
	} else {
		rtc_days_in_month[1] = 28;
	}	
}
//----------------------------------------------------------------------
//  set time to pcf8583, take data from rtc_time struct
//----------------------------------------------------------------------
void pcf8583_set_time(rtc_time* rt) {
	
	cli();                                                                    // disable interrupts: due to rtc write ongoing transaction 

	pcf8583_set_ss_mi_hh24(rt);
	
	register uint8_t tmp = ((rt->year8_t % 4) << 6) | dec2bcd(rt->mday);     // calculating num in pcf8583 leap year cycle and converting day to bcd
	pcf8583_write_byte(PCF8583_YEAR_DAY_REG, tmp);
	rtc_base_year = rt->year8_t - (rt->year8_t % 4);

	tmp = (rt->wday << 5) | dec2bcd(rt->mon);
	pcf8583_write_byte(PCF8583_DOW_MONTHS_REG, tmp);
	
	correct_feb_days(rt->year8_t);

	sei();
	
}
//----------------------------------------------------------------------
//  read time from pcf8583to rtc_time struct
//----------------------------------------------------------------------
void pcf8583_get_time(rtc_time *rt) {
	
	cli();                                                                    // disable interrupts: due to rtc read ongoing transaction 

	pcf8583_get_ss_mi_hh24(rt);
	
	register uint8_t tmp = pcf8583_read_byte(PCF8583_YEAR_DAY_REG);
	rt->mday    = bcd2dec(0b00111111 & tmp);
	rt->year8_t = (tmp >> 6) + rtc_base_year;                                     // @todo: take base year from eeprom
	
	tmp = pcf8583_read_byte(PCF8583_DOW_MONTHS_REG);
	rt->mon = bcd2dec(0b00011111 & tmp);
	rt->wday = tmp >> 5;
	
	sei();
}
//----------------------------------------------------------------------
//  print time for debug
//----------------------------------------------------------------------
void pcf8583_print_time(rtc_time *rt) {
	UARTPrintUint(rt->mday, 10); 
	UARTPrint("-");
	UARTPrintUint(rt->mon, 10);
	UARTPrint("-");
	UARTPrintUint(rt->year8_t, 10);
	UARTPrint(" ");
	UARTPrintUint(rt->wday, 10); 
	UARTPrint(" ");
	UARTPrintUint(rt->hour, 10);
	UARTPrint(":");
	UARTPrintUint(rt->min, 10);
	UARTPrint(":");
	UARTPrintUint(rt->sec, 10);
	UARTPrintln("");
}
//----------------------------------------------------------------------
//  calc and update day of week
//----------------------------------------------------------------------
void pcf8583_refresh_dow(rtc_time* rt) {
	register uint8_t  d = rt->mday;
	register uint8_t  m = rt->mon;
	register uint16_t y = ((uint16_t)rt->year8_t) + 2000;
	
	register uint8_t dow = (d+=m<3?y--:y-2,23*m/9+d+4+y/4-y/100+y/400)%7;
	
	if (dow) {																//decode from sun, mon, tue... to mon, tue, wed...
		--dow;
	} else {
		dow = 6;
	}

	rt->wday = dow;
	
}
//----------------------------------------------------------------------
uint8_t pcf8583_get_base_year() {
	return rtc_base_year;
}
//----------------------------------------------------------------------
void pcf8583_set_base_year(uint8_t by) {
	if( by < 100 ) {
		rtc_base_year = by;
	} else {
		rtc_base_year = 0;		
	}
}
