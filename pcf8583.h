/*
 * pcf8583.h
 *
 * Created: 09.05.2017 18:47:32
 *  Author: USER
 */ 


#ifndef PCF8583_H_
#define PCF8583_H_

//#define RTC_BASE_YEAR 16              // to be refactored -> base year has to be stored in eeprom 

// Register addresses
#define PCF8583_CTRL_STATUS_REG   0x00
#define PCF8583_100S_REG          0x01
#define PCF8583_SECONDS_REG       0x02
#define PCF8583_MINUTES_REG       0x03
#define PCF8583_HOURS_REG         0x04
#define PCF8583_YEAR_DAY_REG      0x05
#define PCF8583_DOW_MONTHS_REG    0x06
#define PCF8583_TIMER_REG         0x07
#define PCF8583_ALARM_CONTROL_REG 0x08
#define PCF8583_ALARM_100S_REG    0x09
#define PCF8583_ALARM_SECS_REG    0x0A
#define PCF8583_ALARM_MINS_REG    0x0B
#define PCF8583_ALARM_HOURS_REG   0x0C
#define PCF8583_ALARM_DATE_REG    0x0D
#define PCF8583_ALARM_MONTHS_REG  0x0E
#define PCF8583_ALARM_TIMER_REG   0x0F
// Use the first NVRAM address for the year byte.
#define PCF8583_YEAR_REG          0x10

#define PCF8583_ADDR_WRITE 0b10100000
#define PCF8583_ADDR_READ  0b10100001

// !!!! DO NOT TOUCH the sequence !!!
typedef struct {
	uint8_t sec;      // 0 to 59
	uint8_t min;      // 0 to 59
	uint8_t hour;     // 0 to 23
	uint8_t wday;     // 1-7
	uint8_t mday;     // 1 to 31
	uint8_t mon;      // 1 to 12
	uint8_t year8_t;  // year - starting from 2000
} rtc_time;

uint8_t rtc_days_in_month[12];

void pcf8583_set_month_days(rtc_time* rt);
void pcf8583_set_time(rtc_time* rt);
void pcf8583_get_time(rtc_time* rt);
void pcf8583_print_time(rtc_time* rt);
void pcf8583_refresh_dow(rtc_time* rt);
uint8_t pcf8583_get_base_year();
void pcf8583_set_base_year(uint8_t by);

// uint8_t pcf8583_read_byte(uint8_t offset);
// uint8_t pcf8583_write_byte(uint8_t offset, uint8_t value);
// uint8_t pcf8583_set_ss_mi_hh24(struct tm* tm_);
// uint8_t pcf8583_get_ss_mi_hh24(struct tm* tm_);

// uint8_t dec2bcd(uint8_t d);
// uint8_t bcd2dec(uint8_t b);

#endif /* PCF8583_H_ */