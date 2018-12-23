/*
 * test_nokia5110_lcd.c
 *
 * Created: 27.05.2017 18:26:19
 * Author : Sergey Shelepin <Sergey.Shelepin@gmail.com>
 */ 

// to_do list:
// 1. day of month array correction (29 February) on year change (01.01)
// 2. menu selection blinking
// 3. implement ic2 protocol in rtc on interrupts not on flag raise/drop waiting
// 4. 100 ms delay in ar button processing

//#define F_CPU 8000000UL

#define FALSE 0
#define TRUE  1

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>

#include "nokia5110.h"
#include "SPIsi.h"
#include "UARTSI.h"
#include "pcf8583.h"
#include "rotenc.h"
#include "nokia5110_dig_gen.h"
#include "valve_sr595.h"

//#define DEBUG_L

#ifdef DEBUG_L
#define DPln(x)			UARTPrintln(x)
#define DP(x)			UARTPrint(x)
#define DPln_P(x)		UARTPrintln_P(PSTR(x))
#define DPlnVal(s, val) UARTPrintlnValDec(s, val)
#define SPVal(s, val)   UARTPrintValDec(s, val)
#else
#define DPln(x) 
#define DP(x)			
#define DPln_P(x)		
#define DPlnVal(s, val) 
#define SPVal(s, val)   
#endif

extern void test_setup();
extern void test_run();

#define SLED_DDR  DDRB
#define SLED_PORT PORTB
#define SLED_PIN  PB6

#define LCD_BL_DDR  DDRC
#define LCD_BL_PORT PORTC
#define LCD_BL_PIN  PC3

#define SIR_LATCH_DDR  DDRD                // shift in register latch pin
#define SIR_LATCH_PORT PORTD
#define SIR_LATCH_PIN  PD4


#define int0_set_as_logical_change()   EICRA |= _BV(ISC00)
#define int0_enable()                  EIMSK |= _BV(INT0)
#define ar_but_is_up()                 (PIND & _BV(PD2)) 

#define MAX_ZONES            4
#define MAX_WSLOTS_PER_ZONE  3

#define DS_HELLO_STATE       0
#define DS_RAIN_DELAY        1
#define DS_WATERING_CONFIG   2
#define DS_VIEW_SCHEDULE     3
#define DS_SEASON_ADJ        4
#define DS_MANUAL_ZONES      5 
#define DS_SET_DATE_TIME     6   //------------

#define DAS_STATUS           0   //------
#define DAS_VIEW_CONFIG      1   // do not touch sequence
#define DAS_VIEW_SCHEDULE    2   //------


#define DSS_NONE                   0
#define DSS_ZONE_SELECTION		   1
#define DSS_ZONE_SETUP			   2
#define DSS_TWS_TABLE			   3
#define DSS_MANUAL_VALVE_SELECTION 4
#define DSS_MANUAL_VALVE_SETUP     5
#define DSS_DT_SELECTION           6
#define DSS_RAIN_DELAY_SELECTION   7
#define DSS_ZONE_VIEW              8
#define DSS_SADJ_SETUP			   9
#define DSS_AUTORUN_MENU           30


#define SEIT_NONE				  0
#define SEIT_ZONE_1               1  //  -------
#define SEIT_ZONE_2		          2  //
#define SEIT_ZONE_3		          3  //  must be one block, DO NOT TOUCH THE NUMBERS AND ORDER !!!!
#define SEIT_ZONE_4               4  //
#define SEIT_ZONE_BACK            5  //  -------
#define SEIT_MAN_VALVE_ZONE       6  //
#define SEIT_MAN_VALVE_TIME       7  //  must be one block, DO NOT TOUCH THE NUMBERS AND ORDER !!!!
#define SEIT_MAN_VALVE_DISCARD    8  //  -------


//selected fied values
#define SF_NONE        0
#define SF_WS1         1
#define SF_WS1_HH      2
#define SF_WS1_MI      3
#define SF_WS1_DUR     4
#define SF_WS2         5
#define SF_WS2_HH      6
#define SF_WS2_MI      7
#define SF_WS2_DUR     8
#define SF_WS3         9
#define SF_WS3_HH      10
#define SF_WS3_MI      11
#define SF_WS3_DUR     12
#define SF_REPEAT      13
#define SF_MON         14
#define SF_TUE         15
#define SF_WED         16
#define SF_THU         17
#define SF_FRI         18
#define SF_SAT         19
#define SF_SUN         20
#define SF_ZS_OK       21
#define SF_ZS_DISCARD  22                // DO NOT TOUCH!!!! SF defines from here and above
#define SF_MV_ZONE     23
#define SF_MV_DUR      24
#define SF_SDT_HH      25
#define SF_SDT_MI      26
#define SF_SDT_SS      27
#define SF_SDT_DD      28
#define SF_SDT_MM      29
#define SF_SDT_YY      30
#define SF_SDT_OK      31
#define SF_SDT_DISCARD 32
#define SF_SSA_RATIO   33
#define SF_SSA_OK      34
#define SF_SSA_DISCARD 35


// selection result definition
#define SR_NONE   1
#define SR_BACK   2
#define SR_NEXT   3
#define SR_OK     4


//TWSE states in twse.status field
// states  model:
//empty-> planned -> active-> passeed
//       (      )----------> canceled 
#define TWSE_STATE_MASK       0x07   //LSB 0-2 bits of status field
#define TWSE_STATE_NEW           0
#define TWSE_STATE_PLANNED       1
#define TWSE_STATE_PREPARE       2
#define TWSE_STATE_ACTIVE        3
#define TWSE_STATE_PASSED        4
#define TWSE_STATE_CANCELED      5

#define TWSE_STATE_DOW_MASK   0xE0    // mask to get day of week from twse status field 3 MSB 

//TWSE properties
#define TWSE_CORRECTED           3     // 3 bit
#define TWSE_SHIFTED             4     // 4 bit

#define TWSE_STATE_PREPARE_DUR   5     // 5 valves_conter cycles

// zoneslot filed
#define TWSE_Z_NUM_MASK       0xFC   // 6 MSB 
#define TWSE_WS_NUM_MASK      0x03   // 2 LSB

typedef struct {
	uint8_t status;
	uint8_t zoneslot;
	uint8_t start_day;
	uint8_t start_month;
	uint8_t start_hh;
	uint8_t start_mi;
	uint8_t stop_hh;
	uint8_t stop_mi;
} true_water_schedule_element;

#define TWSEA_LEN     10
true_water_schedule_element twsea[TWSEA_LEN];

//render tws item
#define RTI_FULL         0
#define RTI_DATE_ONLY    1
#define RTI_NUM_ONLY     2


volatile uint8_t ar_but_event;
uint8_t run_mode;
uint8_t device_state;                  // device configuration modes state
uint8_t device_ar_state;               // device autorun mode state. separation need to save conf menu position
uint8_t device_sub_state;

#define RM_CONFIG       0
#define RM_AUTORUN      1
#define RM_MANUAL       2
#define RM_RAIN_DELAY   3

// LCD UPDATE FLAG
#define LUF_NONE        0               // or FALSE 
#define LUF_MANUAL      1               // or TRUE 
#define LUF_AUTO        2               // TRUE  but differ  -- need to distinguish lcd upd manual and every hs in autorun. status

uint8_t selected_item;
uint8_t selected_field;
uint8_t field_under_change_flag;
uint8_t lcd_update_flag;
uint8_t valve_delay;
uint8_t lcd_backlight_off_delay;
uint8_t menu_return_timeout;

#define MRT_HS                     60   // 30 sec menu return timeout

volatile uint8_t main_register;

#define MR_HS_FLAG                  0   // half-sec flag
#define MR_HS_LCD_UPD_FLAG          1   // half-sec lcd update flag
#define MR_HS_VALVE_CNT_DOWN_FLAG   2   // half-sec valve count down flag
#define MR_HS_SIGNAL_LED_FLAG       3   // half-sec signal led flag
#define MR_HS_LCD_BACKLIGHT_FLAG    4   // half-sec lcd backlight flag
#define MR_HS_MRT_FLAG              5   // half-sec menu return flag
#define MR_S_RTC_SYNCH_FLAG         6   // sec RTC synch flag
#define MR_S_VCS_FLAG               7   // sec VCS flag


uint8_t valves_proc_status;                // twsea[0] prev status and auto run button change events 
#define VPS_AUTORUN_ON_EVENT        7
#define VPS_AUTORUN_OFF_EVENT       6
#define VPS_TWS_STATES_MASK      0x07       // LSB 0-2 bits 

typedef struct {
	uint8_t start_hour;
	uint8_t start_min;
	uint8_t duration;
	uint8_t is_on; 
} wslot;

typedef struct {
	uint8_t type;
	uint8_t data;
} repeat_config;

#define RC_EVERY_DAY  0
#define RC_WEEKDAYS   1
#define RC_PERIOD     2

typedef struct {
 	wslot ws[MAX_WSLOTS_PER_ZONE];
 	repeat_config rc;	
	uint8_t valve; 
} zone_watering_config;

zone_watering_config wca[MAX_ZONES];   // watering config array
zone_watering_config twcs;             // temporary water config for zone setup

typedef struct {    // current setup parameters
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t dow;    // from 0 to 6
	uint8_t hh;
	uint8_t mi;
	uint8_t zone;
	uint8_t ws;
} curent_schedule_parameters;

curent_schedule_parameters cur_sp;

rtc_time rt;
rtc_time rt_s;                              // rtc_time structure for date time setup 

uint8_t s_adj;                              // season adjustment in 10%, i.e. a_adj = 10 means 100% 
uint8_t s_adj_s;                            // season adjustment - setup buffer
#define  SADJ_MIN 5
#define  SADJ_MAX 15 

#define MVP_ALL_ZONES           2           // do not touch without refactoring! this bit set to 1 is equal to 4, this is used with cycle_var() in states prcessing 
#define MVP_ZONE_BITS_MASK      0b11

#define LCD_BACKLIGHT_HS_DUR    20          // 10 sec lcd backlight delay duration
#define VALVE_OPEN_DELAY_HS     4           // 2 sec delay before opening the valve
  
typedef struct {                            // use for 1) manual valve and for 2) rain delay function
	uint8_t  status;                        // cur_zone + mode: one zone, all zones
	uint8_t  duration;                      // duration in minutes for manual valve, delay in hours for rain delay
	uint16_t duration_half_sec_count_down;	// overall countdown for manual valve, 1 hour count down for rain delay
} manual_valve_params;

manual_valve_params mvp = {
	.duration = 5
};

typedef struct {
	uint8_t cur_window_start_line;
	uint8_t cur_lines_total;
} tws_renderer_params;
#define TWS_RENDERER_LINES_SIZE   5

tws_renderer_params twsrp;

const char DOW_NAMES[][4] PROGMEM = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"}; 
	
uint8_t EEMEM eeprom_base_year;
uint8_t EEMEM eeprom_s_adj;
zone_watering_config EEMEM eeprom_wca[MAX_ZONES];


#define VCS_MEASURED_BIT      7           // indicates whether vcs has been measured 
uint8_t valves_connection_status;

void states_processing();
void lcd_processing();

void increment_cur_sp();
void increment_cur_sp_time();
void print_cur_sp();
void tws_rebuild();

void print_wca();
void tws_print();
void tws_status_processing();
void auto_run_button_pocessing();
void run_timer1_at_500msec();
void valves_processing();
void rtc_synch_processing();
void check_wca_consistency_and_correct();
void reset_cur_sp();
void display_valves_conection_status();

uint8_t tws_cleanup();
uint8_t SIR_get_state();

inline void play_intro_animation();
inline void leds_and_mrt_processing();


//*******************************************************************************************************//
int main(void) {
	
	// 1. Ports setup: SLED, LCD BL, BUZ, SIR Latch
	
	SLED_DDR |= _BV(SLED_PIN);                            // signal led DDR as output
	SLED_PORT |= _BV(SLED_PIN);                           // light up the signal led
	SIR_LATCH_DDR |= _BV(SIR_LATCH_PIN);                  // shift-in latch pin as output 
	SIR_LATCH_PORT |= _BV(SIR_LATCH_PIN);				  // set sir latch pin high
	LCD_BL_DDR |= _BV(LCD_BL_PIN);                        // lcd backlight DDR as output
	DDRD |= _BV(PD6);									  // buzzer DDR as output

	// 2. UART start
	UARTInitRXTX_conf(103, 1);                            // UBRRnL = 103 and double speed - settings for 9600 @ 8Mhz
	sei();
	DPln_P("Starting");
	
	// 3. SPI bus init
	SPI_master_start();		

	// 4. LCD init
	nokia_lcd_init();
	LCD_BL_PORT |= _BV(LCD_BL_PIN);						  // turn on lcd back light
	
	// 5. Play animation
	play_intro_animation();
	
	// 6. reading and checking configuration from eeprom
	eeprom_read_block(&wca, &eeprom_wca, sizeof(wca));    // reading configuration from eeprom
	wca[0].valve = VALVE_1;                               // this is like base line 
	wca[1].valve = VALVE_2;
	wca[2].valve = VALVE_3;
	wca[3].valve = VALVE_4;
	check_wca_consistency_and_correct();                  // check and correct water config - need in case if loaded config is not consistent

	s_adj = eeprom_read_byte(&eeprom_s_adj);              // reading season adjustment val
	if(s_adj > 15 || s_adj < 5) {
		s_adj = 10;
	}
	
	// 7. setup int for autorun button	
	int0_set_as_logical_change();						  // setup interrupt for autorun button 
	int0_enable();
	
	// 8. run main timer 
	run_timer1_at_500msec();							  // run timer1 processed main_register variable 
/*	
	// test data !!! need to remove rtc.settime if remove this assignment
	rt.hour = 23;
	rt.min = 29;
	rt.sec = 00;
	rt.wday = 5;										  // from 0 to 6
	rt.mon = 2;
	rt.mday = 27;
	rt.year8_t = 17;
	
	wca[0].ws[0].start_hour = 23;
	wca[0].ws[0].start_min = 30;
	wca[0].ws[0].duration = 5;
	wca[0].ws[0].is_on = TRUE;	
	wca[0].rc.type = RC_EVERY_DAY;

	wca[2].ws[2].start_hour = 23;
	wca[2].ws[2].start_min = 30;
	wca[2].ws[2].duration = 90;
	wca[2].ws[2].is_on = TRUE;
	wca[2].rc.type = RC_EVERY_DAY;

	wca[3].ws[0].start_hour = 9;
	wca[3].ws[0].start_min = 15;
	wca[3].ws[0].duration = 85;
	wca[3].ws[0].is_on = TRUE;	
	wca[3].rc.type = RC_WEEKDAYS;
	wca[3].rc.data = 0b00100101;
//	wca[3].rc.data = 0b11111111;	
*/

	//------------------------

	// 9. valves on shift register init
	valve_sr_init();	
	
	//10. RTC: load base year, refresh day of week, read time from rtc crystal
	pcf8583_set_base_year(eeprom_read_byte(&eeprom_base_year));                    // restore base year from eeprom
//	pcf8583_refresh_dow(&rt);
//	pcf8583_set_time(&rt);	
 	pcf8583_get_time(&rt);
// 	pcf8583_print_time(&rt);

	// 10.1. refresh days in month based on year8)t in rt struct
	pcf8583_set_month_days(&rt);

	// 11. init rotary encoder @ 8Mhz mcu frequency
	re_init_at_mcu8Mhz();															
	
	// 12. true watering schedule (tws) calc: reset cur_sp - move it to current time, then rebuild tws 
	reset_cur_sp();
	tws_rebuild();

	// 12. detect current run mode and set current state
	ar_but_event = TRUE;                  // simulate autorun but event to determine initial state 
	auto_run_button_pocessing();
	device_state = DS_RAIN_DELAY;         // hello state is 0 but not used
	
	DPln_P("Privet!");
	
	// 13. check valves connection and show their status 
	display_valves_conection_status();
		
	lcd_update_flag = TRUE;
			
    while (1) {
		
		leds_and_mrt_processing();
		rtc_synch_processing();
		auto_run_button_pocessing();
		states_processing();
		if(tws_cleanup()){
			tws_rebuild();
		}
		tws_status_processing();
		lcd_processing();
		valves_processing();
		

		
    }
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void check_wca_consistency_and_correct() {
	for (uint8_t z = 0; z < MAX_ZONES; z++) {
		for (uint8_t s = 0; s < MAX_WSLOTS_PER_ZONE; s++) {                                      // check and correct wslots
			register wslot* ws = &wca[z].ws[s];
			if(ws->start_hour > 23 || ws->start_min > 59 || ws->duration > 90) {
				ws->start_hour = 0;
				ws->start_min = 0;
				ws->duration = 0;
				ws->is_on = 0;
			}
		}
		
		if(wca[z].rc.type > RC_WEEKDAYS) {                                                      // check and correct repeat config
			wca[z].rc.type = RC_EVERY_DAY; 
			wca[z].rc.data = 0;
		}
	}
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void rtc_synch_processing() {
	
	if (main_register & _BV(MR_S_RTC_SYNCH_FLAG)) {
		pcf8583_get_time(&rt);
		main_register &= ~_BV(MR_S_RTC_SYNCH_FLAG);                                                  // drop rtc synch flag
	}

}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void valves_manual_processing() {
	
	if(main_register & _BV(MR_HS_VALVE_CNT_DOWN_FLAG)) {
		if(!valve_delay) {																	         // opening valve after open delay
			valve_open((mvp.status & MVP_ZONE_BITS_MASK) + 1);                                       // zone <-> valve num is hardcoded	here !!!
			valve_delay = 255;
		} else if(valve_delay < 255) {
			--valve_delay;
		} 
		
		if(!mvp.duration_half_sec_count_down) {                                                      // if time is over
			if (mvp.status & _BV(MVP_ALL_ZONES) && (mvp.status & MVP_ZONE_BITS_MASK) != 3) {         // if all zones and current zone is not the last zone
				++mvp.status;																		 // go with the next zone
				mvp.duration_half_sec_count_down = mvp.duration * 60 * 2;                            // set count down timer
				valve_delay = VALVE_OPEN_DELAY_HS;													 // set valve delay
			} else {                                                                                 // else 
				valve_close_all();                                                                   // close all
				run_mode = RM_AUTORUN;																 // set mode as autorun
				valves_proc_status |= _BV(VPS_AUTORUN_ON_EVENT);                                     // generate autorun on event				
				mvp.status = 0;																		 // clear mvp status
			}
		} else if(valve_delay == 255) {                                                              // decree timer after valve delay is over
			--mvp.duration_half_sec_count_down;
		}
		main_register &= ~_BV(MR_HS_VALVE_CNT_DOWN_FLAG);                                            // drop valve count down flag in main register 
		lcd_update_flag = TRUE;
	}
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void valves_rain_delay_pocessing() {
	
	if(main_register & _BV(MR_HS_VALVE_CNT_DOWN_FLAG)) {                                             
		if (!mvp.duration && !mvp.duration_half_sec_count_down)	{                                    //if time is over go to autorun mode
			run_mode = RM_AUTORUN;
			valves_proc_status |= _BV(VPS_AUTORUN_ON_EVENT);                                         // generate autorun on event			
		} else {
			if(mvp.duration_half_sec_count_down) {                                                   // descrease mvp.duration_half_sec_count_down if it's not zero
				--mvp.duration_half_sec_count_down;
			} else {                                                                                 // decrease mvp.duration and reset mvp.duration_half_sec_count_down otherwise
				--mvp.duration;
				mvp.duration_half_sec_count_down = 60*60*2 - 1;
			}
		}

		main_register &= ~_BV(MR_HS_VALVE_CNT_DOWN_FLAG);                                            // drop valve count down flag in main register
		lcd_update_flag = TRUE;		
		
	}
	
}
//-----------------------------------------------------------------------------------
//  move current tws element from status active to status prepare
//-----------------------------------------------------------------------------------
uint8_t cur_twse_move_to_prepare_if_active() {

	true_water_schedule_element* e = (true_water_schedule_element*)&twsea;                    // make pointer to first twsea element
	if((e->status & TWSE_STATE_MASK) == TWSE_STATE_ACTIVE){
		e->status &= ~TWSE_STATE_MASK;
		e->status |= TWSE_STATE_PREPARE;
		valve_delay = VALVE_OPEN_DELAY_HS;
		return TRUE;
	}
	
	return FALSE;
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void valves_processing() {
	
	if(run_mode == RM_MANUAL) {
		valves_manual_processing();
		return;
	} else if (run_mode == RM_RAIN_DELAY) {
		valves_rain_delay_pocessing();
		return;	
	}

	true_water_schedule_element* e = (true_water_schedule_element*)&twsea;                              // make pointer to first twsea element
	
	register uint8_t cur_tws0_state = e->status & TWSE_STATE_MASK;
	register uint8_t prev_tws0_status = valves_proc_status & VPS_TWS_STATES_MASK;
	
	if(cur_tws0_state != prev_tws0_status) {                                                            // 1. process status changing events 
		
		if(cur_tws0_state == TWSE_STATE_ACTIVE && run_mode == RM_AUTORUN) {                             // could open valve only in AUTORUN mode
			register uint8_t zone = (e->zoneslot >> 2);
			valve_open(wca[zone].valve);                                                                // open target valve if twsea[0] status Active
		} else if(cur_tws0_state == TWSE_STATE_PASSED) {
			valve_close_all();															                // close all on passed status
		}
	}
	
	if(valves_proc_status &= ~VPS_TWS_STATES_MASK) {                                                   // 2. process autorun on / off when zone is active
		
		if(valves_proc_status & _BV(VPS_AUTORUN_ON_EVENT) && cur_tws0_state == TWSE_STATE_ACTIVE) {    // could open only if twsea[0] status Active 
			
			cur_twse_move_to_prepare_if_active();			
// 			register uint8_t zone = (e->zoneslot >> 2);
// 			valve_open(wca[zone].valve);                                                                
		} else if(valves_proc_status & _BV(VPS_AUTORUN_OFF_EVENT)) {
			valve_close_all();															               // close all on autorun off event
		}
		
	}
	
	valves_proc_status = cur_tws0_state;                                                              // drop all events' flags, save cur status as prev status 
	
}
//-----------------------------------------------------------------------------------
//  set status = NEW to all tasks in twsea array
//-----------------------------------------------------------------------------------
void tws_fast_clean() {
	for(uint8_t i = 0; i < TWSEA_LEN; i++) {
		twsea[i].status = TWSE_STATE_NEW; 	
	}
}
//-----------------------------------------------------------------------------------
//  intraday function
//-----------------------------------------------------------------------------------
int16_t get_diff_in_sec_with_curtime(uint8_t next_hh, uint8_t next_mi, uint8_t next_sec) {
	
	return (int16_t)(next_hh - rt.hour)*3600 + (int16_t)(next_mi - rt.min)*60 + (uint16_t)(next_sec - rt.sec);
}

//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
int16_t get_diff_in_min(uint8_t next_hh, uint8_t next_mi, uint8_t prev_hh, uint8_t prev_mi ) {
	return (int16_t)(next_hh - prev_hh)*60 + (int16_t)(next_mi - prev_mi);
}
//-----------------------------------------------------------------------------------
// switching statuses of tws elements base on time 
//-----------------------------------------------------------------------------------
void tws_status_processing() {
	
	true_water_schedule_element* e = (true_water_schedule_element*)&twsea;                    // make pointer to first twsea element
	
	if (e->start_day != rt.mday || e->start_month != rt.mon) {                                // consider only today actions
		return;
	}
	
//	int16_t minutes_diff = (int16_t)(next_start_hh - prev_stop_hh)*60 + (int16_t)(next_start_mi - prev_stop_mi);	
	
	switch (e->status & TWSE_STATE_MASK) {
		case TWSE_STATE_PLANNED:
			if(get_diff_in_min(rt.hour, rt.min, e->start_hh, e->start_mi) >= 0){
				e->status &= ~TWSE_STATE_MASK;											     // could be coded as e->status++
				e->status |= TWSE_STATE_PREPARE;
				valve_delay = VALVE_OPEN_DELAY_HS;
			}
		break;

		case TWSE_STATE_PREPARE:
			if(!valve_delay) {
				e->status &= ~TWSE_STATE_MASK;											     // could be coded as e->status++
				e->status |= TWSE_STATE_ACTIVE;
			} else if(main_register & _BV(MR_HS_VALVE_CNT_DOWN_FLAG)) {
				--valve_delay;
				main_register &= ~_BV(MR_HS_VALVE_CNT_DOWN_FLAG);
			}
		break;
		
		case TWSE_STATE_ACTIVE:
			if(get_diff_in_min(rt.hour, rt.min, e->stop_hh, e->stop_mi) >= 0) {
				e->status &= ~TWSE_STATE_MASK;												 // could be coded as e->status++
				e->status |= TWSE_STATE_PASSED;                                               
			}
		break;

		case TWSE_STATE_PASSED:
		case TWSE_STATE_CANCELED:
		case TWSE_STATE_NEW:
		default:
		
		break;
	}
}
//-----------------------------------------------------------------------------------
// auto run button handling !! 50 ms delay inside!!
//-----------------------------------------------------------------------------------
void auto_run_button_pocessing() {
	if (ar_but_event) {
		_delay_ms(100);                   // wait till button stabilized
		if(ar_but_is_up()) {
			DPln_P("Config mode");
			run_mode = RM_CONFIG;
//			device_state = DS_RAIN_DELAY;
			device_sub_state = DSS_NONE;
			valves_proc_status |= _BV(VPS_AUTORUN_OFF_EVENT);			
		} else {
			if (device_state == DS_MANUAL_ZONES && device_sub_state != DSS_NONE) {      // process manual valve start

				mvp.duration_half_sec_count_down = (uint16_t)mvp.duration * 60 * 2;
				valve_delay = VALVE_OPEN_DELAY_HS;
				run_mode = RM_MANUAL;

				DPln_P("Manual!");
			} else if(device_state == DS_RAIN_DELAY && device_sub_state != DSS_NONE) {
				run_mode = RM_RAIN_DELAY;
				mvp.duration--;                                                        // decrease duration by one hour
				mvp.duration_half_sec_count_down = 60*60*2 - 1;                      // and put this hour into half sec count down counter
				
				DPln_P("Rain delay!");				
				
			} else {
				run_mode = RM_AUTORUN;
				valves_proc_status |= _BV(VPS_AUTORUN_ON_EVENT);
				
				DPln_P("Autorun mode");
			}
			device_ar_state = DAS_STATUS;
			device_sub_state = DSS_NONE;
		}
		ar_but_event = FALSE;
		lcd_update_flag = TRUE;
	}
}

//-----------------------------------------------------------------------------------
// aligns 1 part;
// input: first member number
//-----------------------------------------------------------------------------------
void align_pair_twe(uint8_t fmn) {
	register uint8_t prev_stop_hh  = twsea[fmn].stop_hh;
	register uint8_t prev_stop_mi  = twsea[fmn].stop_mi;
	register uint8_t next_start_hh = twsea[fmn + 1].start_hh;
	register uint8_t next_start_mi = twsea[fmn + 1].start_mi;
	
//	int16_t minutes_diff = (int16_t)(next_start_hh - prev_stop_hh)*60 + (int16_t)(next_start_mi - prev_stop_mi);
	int16_t minutes_diff = get_diff_in_min(next_start_hh, next_start_mi, prev_stop_hh, prev_stop_mi);
	
	if (minutes_diff < 0) {
		
		uint16_t new_min = (uint16_t)next_start_mi - minutes_diff; 
		
		twsea[fmn + 1].start_hh += new_min / 60;
		twsea[fmn + 1].start_mi = new_min % 60;
		
		new_min = (uint16_t)twsea[fmn + 1].stop_mi - minutes_diff;

		twsea[fmn + 1].stop_hh += new_min / 60;;
		twsea[fmn + 1].stop_mi = new_min % 60;;
		
		twsea[fmn + 1].status |= _BV(TWSE_SHIFTED);                                     // put is_shifted flag
		
	}

}
//-----------------------------------------------------------------------------------
// tws grooming. 
// assuming ALL items are FILLED(!) 
//-----------------------------------------------------------------------------------
void groom_tws() {
	
	// 1. alignment 
	for (uint8_t i = 0; i < TWSEA_LEN - 1; i++) {                                      // align all twse in TWS cycle
		
		if( twsea[i + 1].start_month == twsea[i].start_month &&                        // if same start day and (to be on the safe side) month
			twsea[i + 1].start_day == twsea[i].start_day) {
				align_pair_twe(i);                                                     // align that pair
		}
		
	}
	
#ifdef DEBUG_L
	print_wca();
	UARTPrintln("");
	UARTPrintln("after alignment");
	tws_print();
#endif
	
	//2. correct or cancel some twse
	for (uint8_t i = 0; i < TWSEA_LEN - 1; i++) {												// align all twse in TWS cycle
			
		if(twsea[i].start_hh >= 24 ) {													// 1. could not start in the next day - cancel watering
			
			twsea[i].status &= ~TWSE_STATE_MASK;                                        // clear 3 lsb states bits
			twsea[i].status |= TWSE_STATE_CANCELED; 
			
		} else if(twsea[i].stop_hh >= 24 ) {											// 2. could not move over the day - correct watering
			twsea[i].stop_hh = 23;
			twsea[i].stop_mi = 59;
			twsea[i].status |= _BV(TWSE_CORRECTED);
		}
	}
#ifdef DEBUG_L
 	UARTPrintln("after cancel/correct");
 	tws_print();
#endif
	
	
}

//-----------------------------------------------------------------------------------
// checking wce start based on cur_sp data
//-----------------------------------------------------------------------------------
uint8_t check_wce_start_at_cur_sp() {
	
	register uint8_t z = cur_sp.zone;
	register uint8_t s = cur_sp.ws;
	
	if(wca[z].rc.type == RC_WEEKDAYS) {                                                // return FALSE if rc_type is RC_WEEKDAYS and cur_sp week day is not setup in wca[z]
/*		UARTPrintlnValDec("rc.data", wca[z].rc.data );
		UARTPrintlnValDec("cur_sp.dow", cur_sp.dow);		
		UARTPrintlnValDec("!res", wca[z].rc.data & _BV(cur_sp.dow));				
		UARTPrintlnValDec("res", !(wca[z].rc.data & _BV(cur_sp.dow)));			
*/		if(!(wca[z].rc.data & _BV(cur_sp.dow))) {
			return FALSE;
		}
	}
	

	
	return 
	wca[z].ws[s].is_on && 
	wca[z].ws[s].start_hour == cur_sp.hh &&
	wca[z].ws[s].start_min == cur_sp.mi;
	
}

//-----------------------------------------------------------------------------------
//  checks if there any active water slot configured,
//-----------------------------------------------------------------------------------
uint8_t is_any_active_ws() {
	for (uint8_t z = 0; z < MAX_ZONES; z++) {
		for(uint8_t s = 0; s < MAX_WSLOTS_PER_ZONE; s++) {
			if (wca[z].ws[s].is_on)	{
				return TRUE;
			}
		}
	}
	
	return FALSE;
	
}
//-----------------------------------------------------------------------------------
// incrementing time in cur_sp structure 
//-----------------------------------------------------------------------------------
void increment_cur_sp_time() {
	
	const uint8_t inc = 5;   // 5 min increment  only 1, 2, 3, 4, 5, 10, 12, 15, 20, 30 are ok
	
	if (cur_sp.mi != (60-inc)) {
		cur_sp.mi += inc; 
	} else {                            // new hour
		cur_sp.mi = 0;
		
		if(cur_sp.hh != 23){
			++cur_sp.hh;
		} else {                       // new day
			cur_sp.hh = 0;
			
			if (cur_sp.dow == 6) {     //cycle day of week
				cur_sp.dow = 0;
			} else{
				++cur_sp.dow;
			}
			
			if(cur_sp.day != rtc_days_in_month[cur_sp.month - 1]) {
				++cur_sp.day;
			} else {                    //new month
				cur_sp.day = 1;
				if (cur_sp.month !=12)	{
					++cur_sp.month;
				} else {                  //new year
					cur_sp.month = 1;
					++cur_sp.year;
				}
			}
		}
	}
}
//-----------------------------------------------------------------------------------
// incrementing cur_sp starting form water slots
//-----------------------------------------------------------------------------------
void increment_cur_sp() {
	if(cur_sp.ws < MAX_WSLOTS_PER_ZONE -1) {
		++cur_sp.ws;
	} else {
		cur_sp.ws = 0;
		if(cur_sp.zone < MAX_ZONES - 1 ) {        // new zone
			++cur_sp.zone;
		} else {
			cur_sp.zone = 0;
			increment_cur_sp_time();         // new time
		}
	}
}
//-----------------------------------------------------------------------------------
// Build tws based on watering config array (wca) and current setup parameters (cur_sp)
// replacing elements with NEW status (!twsea[i].status & TWSE_STATE_MASK).
// Need grooming afterwards.
//-----------------------------------------------------------------------------------
void tws_rebuild() {
	
	if(!is_any_active_ws()) {
		return;
	}
	
	register uint8_t tws_changed = FALSE;
	
	for (uint8_t i = 0; i < TWSEA_LEN; i++) {                                                   // checking all the array of twse
 		if(!(twsea[i].status & TWSE_STATE_MASK)) {                                              // if twse is new - assuming TWSE_STATE_NEW is 0
			
			uint8_t mf = FALSE;
			while (!mf) {																	    // get zone and ws og
				mf = check_wce_start_at_cur_sp();
				if (mf) {
					twsea[i].start_month = cur_sp.month;
					twsea[i].start_day = cur_sp.day;
					twsea[i].start_hh = cur_sp.hh;
					twsea[i].start_mi = cur_sp.mi;
					
					twsea[i].zoneslot = (cur_sp.zone << 2);
					twsea[i].zoneslot |= cur_sp.ws;
					
					twsea[i].status = TWSE_STATE_PLANNED;                                       // set planned state
					twsea[i].status |=  (cur_sp.dow << 5);										// put dow into status 
					register uint8_t dur = wca[cur_sp.zone].ws[cur_sp.ws].duration * s_adj / 10;// calc duration with season adjustment 
					register uint8_t mi = cur_sp.mi + dur;										// simple calc and update of stop hour and stop minute
					twsea[i].stop_hh = cur_sp.hh + mi/60;
					twsea[i].stop_mi = mi%60;
			
					tws_changed = TRUE;		
					
				}
				increment_cur_sp();
			}
 		}
	}
#ifdef DEBUG_L
	UARTPrintln("");
	UARTPrintln("tws rebuild:");
	tws_print();
#endif	
	
	if(tws_changed) {
		groom_tws();
	}
	
}
//-----------------------------------------------------------------------------------
//  removes first queue passed and canceled tasks from tws 
//  return true if records were removed
//-----------------------------------------------------------------------------------
uint8_t tws_cleanup() {
	
	register uint8_t sf = TRUE;
	register uint8_t res = FALSE;
	
	while(sf) {
		
		sf = FALSE;
		
		uint8_t twse_state = twsea[0].status & TWSE_STATE_MASK;
		
		if(twse_state == TWSE_STATE_PASSED || twse_state == TWSE_STATE_CANCELED) {
			memcpy(twsea, twsea + 1, sizeof(twsea)*(TWSEA_LEN - 1)/TWSEA_LEN);
			twsea[TWSEA_LEN - 1].status = 0;
			sf = TRUE;
			res = TRUE;
		}
	}
	
	return res;
}

//-----------------------------------------------------------------------------------
// listening to rotation signal, 
// if it is - cycling variable, turn off rotation signal flag, returns true
//  otherwise returns false
//-----------------------------------------------------------------------------------
uint8_t cycle_var_on_rotation_signal(uint8_t *var, uint8_t min, uint8_t max) {
	if(re_rotation_signal) {
		
		register uint8_t rs = re_rotation_signal;
		re_rotation_signal = RE_NO_SIGNAL; 
		
		if(rs == RE_CLOCKWISE_STEP) {
			if(*var == max) {
				*var = min;
				return TRUE;
			}	
			++*var;
		}

		if(rs == RE_COUNTER_CLOCKWISE_STEP) {
			if(*var == min) {
				*var = max;
				return TRUE;
			}
			--*var;
		}
		return TRUE;
			
	}
	return FALSE;
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void cycle_var(uint8_t *var, uint8_t min, uint8_t max, uint8_t step, uint8_t is_up) {
	
	if(is_up) {
		if ((*var + step) > max) {
			*var = min;
			return;
		}
		*var += step;
	} else {
		if ( *var < step  || *var == min) {
			*var = max;
			return;
		}
		*var -= step;
	}

}

//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
uint8_t change_field_val_on_rotation_signal() {
	
	if(!re_rotation_signal) {
		return FALSE;
	}
	
	uint8_t is_up = (re_rotation_signal == RE_CLOCKWISE_STEP) ? TRUE : FALSE; 
	re_rotation_signal = RE_NO_SIGNAL;
	
	switch (selected_field)	{
		case SF_WS1_HH:
		case SF_WS2_HH:
		case SF_WS3_HH:
			cycle_var(&twcs.ws[(selected_field - 2)/4].start_hour, 0, 23, 1, is_up);    // 2, 6, 10 -> 0, 1, 2
		break;

		case SF_WS1_MI:
		case SF_WS2_MI:		
		case SF_WS3_MI:				
			cycle_var(&twcs.ws[(selected_field - 3)/4].start_min, 0, 55, 5, is_up);    // 3, 7, 11 -> 0, 1, 2 
		break;
		
		case SF_WS1_DUR:
		case SF_WS2_DUR:
		case SF_WS3_DUR:
			cycle_var(&twcs.ws[(selected_field - 4)/4].duration, 0, 90, 5, is_up);    // 4, 8, 12 -> 0, 1, 2
		break;
	}
	
	return TRUE;
	
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
uint8_t zone_setup_processing() {  
	
	if(!field_under_change_flag) {
		
		if(re_rotation_signal) {
			register uint8_t is_up = (re_rotation_signal == RE_CLOCKWISE_STEP)? 1 : 0;
			re_rotation_signal = RE_NO_SIGNAL;
			lcd_update_flag = TRUE;			
			
			do{
				cycle_var(&selected_field, SF_WS1, SF_ZS_DISCARD, 1, is_up);
			} while( selected_field < SF_REPEAT                                               // skip off water slots
					&& !twcs.ws[(selected_field - 1)/4].is_on 
			        && (selected_field - 1)%4 > 0 );

			if (twcs.rc.type != RC_WEEKDAYS) {                                                // skip dow selected fields if rc.type is not RC_WEEKDAYS
				if (selected_field == SF_SUN) {
					selected_field = SF_REPEAT;					
				} else if(selected_field == SF_MON) {
					selected_field = SF_ZS_OK;					
				}
			}
		}
		
	} else {
		if(change_field_val_on_rotation_signal()){
			lcd_update_flag = TRUE;
		}
	}
		
	if(re_button_signal) {
		re_button_signal = RE_NO_SIGNAL;
		switch(selected_field) {
			case SF_ZS_DISCARD:
				return TRUE;		
			break;

			case SF_ZS_OK:
				for (uint8_t i = 0; i < 3; i++) {                                          // turn off all water slots with zero duration
					if(twcs.ws[i].duration == 0) {
						twcs.ws[i].is_on = FALSE;
					}
				}
				memcpy(wca + selected_item - 1, &twcs, sizeof(twcs));		               // copy config buffer to configuration array	
				eeprom_write_block(&wca, &eeprom_wca, sizeof(wca));                        // write water config to eeprom
				tws_fast_clean();														   // clean current tws to recalculate it
				reset_cur_sp();															   // reset cur_sp to current time (rt)
				tws_rebuild();															   // rebuild tws
				return TRUE;			
			break;
			
			case SF_REPEAT:
				twcs.rc.type ^= 1;                                                        // works only for two consequent types. to be changes id third type added
			break;
			
			case SF_MON:
			case SF_TUE:
			case SF_WED:
			case SF_THU:
			case SF_FRI:
			case SF_SAT:
			case SF_SUN:									
				twcs.rc.data ^= _BV(selected_field - SF_MON);                             // set/clear bits in rc.data.  mon, tue, wed... are LSB first in rc.data
			break;
			
			// 1, 5, 9
			case SF_WS1: 
			case SF_WS2:
			case SF_WS3:
				twcs.ws[(selected_field - 1)/4].is_on ^= 1;                                // 1, 5, 9 -> 0, 1, 2;   unverting is_on flag
			break;

			default:
				field_under_change_flag ^= 1; 	                                           //inverting FUC_flag
			break;
		}
		lcd_update_flag = TRUE;
	}
	
	return FALSE;
	
}

//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
uint8_t zone_selection_processing() {

	if(cycle_var_on_rotation_signal(&selected_item, SEIT_ZONE_1, SEIT_ZONE_BACK)) {
		lcd_update_flag = TRUE;
	}
	
	if (re_button_signal) {
		
		re_button_signal = RE_NO_SIGNAL;
		
		switch(selected_item) {                                                       // if selected zone clicked go to next page - zone setup
			case SEIT_ZONE_1:
			case SEIT_ZONE_2:
			case SEIT_ZONE_3:
			case SEIT_ZONE_4:
				return SR_NEXT;
			break;
			
			case SEIT_ZONE_BACK:                                                      // return SR_BACK status if trash bin clicked 
				selected_item = SEIT_NONE;
				return SR_BACK;
			break;																
		}
	}
	return SR_NONE;	
	
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
uint8_t set_date_time_selection_processing() {

    if(!field_under_change_flag) {
		if(cycle_var_on_rotation_signal(&selected_field, SF_SDT_HH, SF_SDT_DISCARD)) {
			lcd_update_flag = TRUE;
		}
	} else {
		register uint8_t* f = &rt_s.hour;
		register uint8_t max = 23;
		register uint8_t min = 0;
		
		switch (selected_field)	{
			case SF_SDT_HH:
				f = &rt_s.hour;
			break;
			case SF_SDT_MI:
				f = &rt_s.min;
				max = 59;
			break;
			case SF_SDT_SS:
				f = &rt_s.sec;
				max = 59;
			break;
			case SF_SDT_DD:
				f = &rt_s.mday;
				max = rtc_days_in_month[rt_s.mon - 1];
				min = 1;
			break;
			case SF_SDT_MM:
				f = &rt_s.mon;
				max = 12;
				min = 1;
			break;
			case SF_SDT_YY:
				f = &rt_s.year8_t;
				max = 99;
			break;
		}

		if(cycle_var_on_rotation_signal(f, min, max)) {
// 			UARTPrintlnValDec("min", min);
// 			UARTPrintlnValDec("val", *f);
// 			UARTPrintlnValDec("max", max);
			if (rt_s.mday > rtc_days_in_month[rt_s.mon-1])	{
				rt_s.mday = rtc_days_in_month[rt_s.mon-1];
			}
			lcd_update_flag = TRUE;
		}
	}

	if (re_button_signal) {
		re_button_signal = RE_NO_SIGNAL;
		switch (selected_field) {
			case SF_SDT_OK:
				return SR_OK;						
			break;
			case SF_SDT_DISCARD:
				return SR_BACK;
			break;
			default:
				field_under_change_flag ^= 1;
			break;
		}
		
	}
	return SR_NONE;

}

//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
uint8_t manual_valve_selection_processing() {
	if(cycle_var_on_rotation_signal(&selected_item, SEIT_MAN_VALVE_ZONE, SEIT_MAN_VALVE_DISCARD)) {
		lcd_update_flag = TRUE;
	}
	
	if (re_button_signal) {

		re_button_signal = RE_NO_SIGNAL;
		
		switch (selected_item) {
			case SEIT_MAN_VALVE_ZONE:
				selected_field = SF_MV_ZONE;
				return SR_NEXT;
			break;

			case SEIT_MAN_VALVE_TIME:
				selected_field = SF_MV_DUR;
				return SR_NEXT;
			break;
		
			case SEIT_MAN_VALVE_DISCARD:
				return SR_BACK;
			break;
		}		
	}
	return SR_NONE;
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
uint8_t manual_valve_setup() {

	if(re_rotation_signal) {
		
		uint8_t is_up = (re_rotation_signal == RE_CLOCKWISE_STEP)?TRUE:FALSE;         // calc direction

		if (selected_field == SF_MV_ZONE) {
			cycle_var(&mvp.status, 0, 4, 1, is_up);                                   // 4 is all zones (bit 2 is set)
		} else if (selected_field == SF_MV_DUR)	{
			cycle_var(&mvp.duration, 5, 90, 5, is_up);
		}
		re_rotation_signal = RE_NO_SIGNAL;
		lcd_update_flag = TRUE;

	}
	
	if (re_button_signal) {
		re_button_signal = RE_NO_SIGNAL;
		selected_field = SF_NONE;
		return TRUE;
	}
	
	return FALSE;
	
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
uint8_t rain_delay_selection_processing() {
	
	if (re_rotation_signal)	{
		register uint8_t is_up = (re_rotation_signal == RE_CLOCKWISE_STEP)? TRUE : FALSE ;
		re_rotation_signal = RE_NO_SIGNAL; 
		cycle_var(&mvp.duration, 4, 72, 4, is_up);
		lcd_update_flag = TRUE;
	}
	
	if(re_button_signal) {
		re_button_signal = RE_NO_SIGNAL;
		return FALSE;
	}
	return TRUE;
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
uint8_t tws_view_processing() {
	if(re_button_signal) {
		re_button_signal = RE_NO_SIGNAL;
		return FALSE;
	}
	
	if(re_rotation_signal) {
		register uint8_t is_forward = (re_rotation_signal == RE_CLOCKWISE_STEP) ? TRUE: FALSE;
		re_rotation_signal = RE_NO_SIGNAL;
		
		if(is_forward){
			if (twsrp.cur_window_start_line < twsrp.cur_lines_total - TWS_RENDERER_LINES_SIZE) {
				++twsrp.cur_window_start_line;
			}
			} else {
			if (twsrp.cur_window_start_line > 0) {
				--twsrp.cur_window_start_line;
			}
		}
		
		lcd_update_flag = TRUE;
	}
	
	return TRUE;
	
}

//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void states_processing_ar() {
	
	switch(device_sub_state) {
		
		case DSS_AUTORUN_MENU:																		// cycle autorun menu
			if(cycle_var_on_rotation_signal(&device_ar_state, DAS_STATUS, DAS_VIEW_SCHEDULE)) {
				lcd_update_flag = TRUE;
			}
		break;
		
		case DSS_NONE:
			if(device_ar_state == DAS_VIEW_SCHEDULE) {
				if(!tws_view_processing()) {
					device_sub_state = DSS_AUTORUN_MENU;
					lcd_update_flag = TRUE;
				}
			} else {
				if(re_rotation_signal) {
					re_rotation_signal = RE_NO_SIGNAL;
					lcd_update_flag = TRUE;
				}
			}
		break;
		
		case DSS_ZONE_SELECTION:
			switch(zone_selection_processing()) {
				case SR_BACK:
					device_sub_state = DSS_AUTORUN_MENU;
					lcd_update_flag = TRUE;
				break;
				case SR_NEXT:
					device_sub_state = DSS_ZONE_VIEW;											  // switch to DSS_ZONE_VIEW substate to render zone config in read only
					memcpy(&twcs, wca + selected_item - 1, sizeof(twcs));                         // copying selected one to config buf
					lcd_update_flag = TRUE;
				break;				
			}
		break;
		
		case DSS_ZONE_VIEW:
			if(re_button_signal) {
				re_button_signal = RE_NO_SIGNAL;
				device_sub_state = DSS_ZONE_SELECTION;
				lcd_update_flag = TRUE;
			}
		break;
	}

	
	if(re_button_signal) {
		if (device_sub_state == DSS_NONE) {													      // select autorun state
			device_sub_state = DSS_AUTORUN_MENU;
		} else {

			switch(device_ar_state) {
				case DAS_VIEW_SCHEDULE:
					twsrp.cur_window_start_line = 0;
					twsrp.cur_lines_total = TWSEA_LEN;
					device_sub_state = DSS_NONE;
				break;
				
				case DAS_VIEW_CONFIG:
					selected_item = SEIT_ZONE_1;		
					device_sub_state = DSS_ZONE_SELECTION;
				break;	
							
				default:
					device_sub_state = DSS_NONE;
				break;
			}
		}

		re_button_signal = RE_NO_SIGNAL;
		lcd_update_flag = TRUE;
	} 
	
	if(device_ar_state == DAS_STATUS && device_sub_state == DSS_NONE && (main_register & _BV(MR_HS_LCD_UPD_FLAG)) ) {    // refresh lcd on AR status screen every HS
		main_register &= ~_BV(MR_HS_LCD_UPD_FLAG);
		lcd_update_flag = LUF_AUTO;
	}
}
//-----------------------------------------------------------------------------------
// here we do all setup in fields terms. Ratio header (item) adn ratio value is
// effectively considered as one field. Not good approach but don't want recoding...
//-----------------------------------------------------------------------------------
uint8_t s_adj_setup() {
	
	if(re_button_signal) {
		re_button_signal = RE_NO_SIGNAL;
		switch (selected_field)	{
			case SF_SSA_RATIO:
				field_under_change_flag ^= 1;
				lcd_update_flag = TRUE;
			break;
			case SF_SSA_OK:
				s_adj = s_adj_s;
				eeprom_write_byte(&eeprom_s_adj, s_adj);
				tws_fast_clean();														   // clean current tws to recalculate it
				reset_cur_sp();															   // reset cur_sp to current time (rt)
#ifdef DEBUG_L
				print_cur_sp();
				pcf8583_print_time(&rt);
#endif				
				tws_rebuild();															   // rebuild tws			
				return SR_OK;
			break;
			case SF_SSA_DISCARD:
				return SR_BACK;
			break;
		}
	}

	if(re_rotation_signal) {
		if(re_rotation_signal == RE_CLOCKWISE_STEP) {
			if(field_under_change_flag) {
				if(s_adj_s < SADJ_MAX) { ++s_adj_s;	} 
			} else {
				cycle_var(&selected_field, SF_SSA_RATIO, SF_SSA_DISCARD, 1, TRUE);
			}
		} else if(re_rotation_signal == RE_COUNTER_CLOCKWISE_STEP) {
			if(field_under_change_flag) {
				if(s_adj_s > SADJ_MIN) { --s_adj_s;	}
			} else {
				cycle_var(&selected_field, SF_SSA_RATIO, SF_SSA_DISCARD, 1, FALSE);
			}
		}
		re_rotation_signal = RE_NO_SIGNAL;
		lcd_update_flag = TRUE;
	}

	
	return SR_NONE;
	
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void states_processing() {
	
	if(run_mode) {                                                                     // autorun or manual
		states_processing_ar();
		return;
	}
	
	if(device_sub_state == DSS_NONE) {
		if(cycle_var_on_rotation_signal(&device_state, 1, 6)) {
			lcd_update_flag = TRUE;
		}
	}
	
	switch (device_state) {
// DS_SEASON_ADJ
	case DS_SEASON_ADJ:		
		switch (device_sub_state){
			case DSS_NONE:
				if(re_button_signal) {
					re_button_signal = RE_NO_SIGNAL;
					device_sub_state = DSS_SADJ_SETUP;
					field_under_change_flag	= FALSE;
					selected_field = SF_SSA_RATIO;
					lcd_update_flag = TRUE;
					s_adj_s = s_adj;
				}
			break;
			case DSS_SADJ_SETUP:
				switch(s_adj_setup()) {
					case SR_BACK:
						device_sub_state = DSS_NONE;
						selected_field = SF_NONE;
						lcd_update_flag = TRUE;
					break;
					case SR_OK:
						device_sub_state = DSS_NONE;
						lcd_update_flag = TRUE;
					break;
				}
			break;

		}
	break;
// DS_SET_DATE_TIME
	case DS_SET_DATE_TIME:
		switch(device_sub_state) {
			case DSS_NONE:
				if(re_button_signal) {
					re_button_signal = RE_NO_SIGNAL;
					device_sub_state = DSS_DT_SELECTION;
					selected_field = SF_SDT_HH;
					lcd_update_flag = TRUE;
					memcpy(&rt_s, &rt, sizeof(rt));
				}
			break;
			case DSS_DT_SELECTION:
				switch(set_date_time_selection_processing()) {
					case SR_BACK:
						device_sub_state = DSS_NONE;
						lcd_update_flag = TRUE;
					break;
					case SR_OK:
						memcpy(&rt, &rt_s, sizeof(rt));
						pcf8583_refresh_dow(&rt);
						pcf8583_set_time(&rt);
// 						UARTPrintlnValDec("rt.year8_t ", rt.year8_t );
// 						UARTPrintlnValDec("rt.year8_t - rt.year8_t % 4", rt.year8_t - rt.year8_t % 4 );						
// 						UARTPrintlnValDec("pcf8583_get_base_year()", pcf8583_get_base_year());
						eeprom_write_byte(&eeprom_base_year, rt.year8_t - rt.year8_t % 4);
						device_sub_state = DSS_NONE;
						lcd_update_flag = TRUE;
					break;
				}
			break;
		}
	break;
		
// DS_WATERING_CONFIG:
		case DS_WATERING_CONFIG:
			switch(device_sub_state){
				case DSS_NONE:
					if(re_button_signal) {
//						UARTPrintln("re_but signal in dss_wc");
						re_button_signal = RE_NO_SIGNAL;
						device_sub_state = DSS_ZONE_SELECTION;   
						selected_item = SEIT_ZONE_1;
						lcd_update_flag = TRUE;
					}
				break;
				
				case DSS_ZONE_SELECTION:
					switch(zone_selection_processing()) {
						case SR_BACK:
							device_sub_state = DSS_NONE;
							lcd_update_flag = TRUE;
						break;
						case SR_NEXT:
							device_sub_state = DSS_ZONE_SETUP;
							memcpy(&twcs, wca + selected_item - 1, sizeof(twcs));  // copying selected one to config buf
							selected_field = SF_WS1;
							lcd_update_flag = TRUE;							
						break;
					}
				break;
				
				case DSS_ZONE_SETUP:
					if(zone_setup_processing()) {
						device_sub_state = DSS_ZONE_SELECTION;
						lcd_update_flag = TRUE;
					}
				break;
			}
		break;
// DS_VIEW_SCHEDULE:
		case DS_VIEW_SCHEDULE:
			switch(device_sub_state) {
				case DSS_NONE:
					if(re_button_signal) {
						re_button_signal = RE_NO_SIGNAL;
						twsrp.cur_window_start_line = 0;
						twsrp.cur_lines_total = TWSEA_LEN;						
						device_sub_state = DSS_TWS_TABLE;
						lcd_update_flag = TRUE;
					}
				break;
				case DSS_TWS_TABLE:
					if(!tws_view_processing()) {
						device_sub_state = DSS_NONE;
						lcd_update_flag = TRUE;
					}
				break;
			}
		break;
// DS_RAIN_DELAY:
		case DS_RAIN_DELAY:
			switch(device_sub_state) {
				case DSS_NONE:
					if(re_button_signal) {
						re_button_signal = RE_NO_SIGNAL;
						device_sub_state = DSS_RAIN_DELAY_SELECTION;
						mvp.duration = 4;                                         //using rt_s for rain delay 
						lcd_update_flag = TRUE;
					}
				break;
				case DSS_RAIN_DELAY_SELECTION:
					if(!rain_delay_selection_processing()) {
						device_sub_state = DSS_NONE;
						lcd_update_flag = TRUE;
					}
				break;
			}
		break;

// DS_MANUAL_ZONES:		
		case DS_MANUAL_ZONES:
			switch(device_sub_state) {
				case DSS_NONE:
					if (re_button_signal)	{
						mvp.status = 0;
						mvp.duration = 5;
						device_sub_state = DSS_MANUAL_VALVE_SELECTION;
						selected_item = SEIT_MAN_VALVE_ZONE;
						selected_field = SF_NONE;
						re_button_signal = RE_NO_SIGNAL;
						lcd_update_flag = TRUE;
					}					
				break;
				case DSS_MANUAL_VALVE_SELECTION:
					switch(manual_valve_selection_processing()) {
						case SR_BACK:
							device_sub_state = DSS_NONE;
							selected_item = SEIT_NONE;
							lcd_update_flag = TRUE;
						break;
						case SR_NEXT:
							device_sub_state = DSS_MANUAL_VALVE_SETUP;
							lcd_update_flag = TRUE;
						break;
					}
				break;
				case DSS_MANUAL_VALVE_SETUP:
					if(manual_valve_setup()) {
						device_sub_state = DSS_MANUAL_VALVE_SELECTION;
						lcd_update_flag = TRUE;
					}
				break;
				
			}
		break;
		
		default:
		break;
	}

}
//----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
inline void draw_datetime_selector() {
	nokia_lcd_set_dig_font(SL_FONT_4x7);
	nokia_lcd_print_uint(23, 5, rt_s.hour, 2);
	nokia_lcd_print_uint(37, 5, rt_s.min, 2);
	nokia_lcd_print_uint(51, 5, rt_s.sec, 2);
	
	nokia_lcd_set_pixel(34, 6, 1);
	nokia_lcd_set_pixel(34, 10, 1);
	nokia_lcd_set_pixel(48, 6, 1);
	nokia_lcd_set_pixel(48, 10, 1);

	nokia_lcd_print_uint(29, 16, rt_s.mday, 2);
	nokia_lcd_gen_slash(40, 16);
	nokia_lcd_print_uint(46, 16, rt_s.mon, 2);

	nokia_lcd_print_uint(33, 27, 20, 2);
	nokia_lcd_print_uint(43, 27, rt_s.year8_t, 2);
	
	nokia_lcd_put_pic(18, 40, PIC_OK);
	nokia_lcd_put_pic(58, 40, PIC_DISCARD);

}
//----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void render_datetime_selection() {
	draw_datetime_selector();
	
	switch(selected_field) {
		case SF_SDT_HH:
			nokia_lcd_invert_area(22, 4, 32, 12);
		break;
		case SF_SDT_MI:
			nokia_lcd_invert_area(36, 4, 46, 12);
		break;
		case SF_SDT_SS:
			nokia_lcd_invert_area(50, 4, 60, 12);
		break;
		case SF_SDT_DD:
			nokia_lcd_invert_area(28, 15, 38, 23);					
		break;
		case SF_SDT_MM:
			nokia_lcd_invert_area(45, 15, 55, 23);		
		break;
		case SF_SDT_YY:
			nokia_lcd_invert_area(32, 26, 53, 34);
		break;
		case SF_SDT_OK:
			nokia_lcd_invert_area(0, 38, 41, 47);			
		break;
		case SF_SDT_DISCARD:				
			nokia_lcd_invert_area(42, 38, 83, 47);
		break;
		
	}
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
inline void draw_adj_selector() {
	nokia_lcd_set_cursor(26, 1);
	nokia_lcd_write_string_P(PSTR("RATIO"));
	nokia_lcd_set_dig_font(DL_FONT_8x14);
	nokia_lcd_print_uint(18, 15, s_adj_s, 2);
	nokia_lcd_gen_dig(42, 15, 0);
	
	#define SAP_X 54
	#define SAP_Y 15
	
	nokia_lcd_gen_slash(SAP_X, SAP_Y);
	nokia_lcd_draw_rect(SAP_X, SAP_Y + 1, SAP_X + 2, SAP_Y + 3 );
	nokia_lcd_draw_rect(SAP_X + 6, SAP_Y + 11, SAP_X + 8, SAP_Y + 13 );
	
	nokia_lcd_put_pic(18, 40, PIC_OK);
	nokia_lcd_put_pic(58, 40, PIC_DISCARD);
	
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void render_sadj_setup() {

	draw_adj_selector();
	
	switch(selected_field) {
		case SF_SSA_RATIO:
			nokia_lcd_invert_area(0, 0, 83, 9);	
		break;
		case SF_SSA_OK:
			nokia_lcd_invert_area(0, 38, 41, 47);
		break;
		case SF_SSA_DISCARD:
			nokia_lcd_invert_area(42, 38, 83, 47);
		break;
	}
	
	if(field_under_change_flag) {
		nokia_lcd_invert_area(16, 13, 64, 31);
	}
	
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
inline void draw_dss_zone_selector() {
	nokia_lcd_set_cursor(3, 6);
	nokia_lcd_write_string("ZONE ", 1);
	nokia_lcd_write_string("1", 1);
	nokia_lcd_set_cursor(3, 25);
	nokia_lcd_write_string("ZONE ", 1);
	nokia_lcd_write_string("3", 1);
	nokia_lcd_set_cursor(46, 6);
	nokia_lcd_write_string("ZONE ", 1);
	nokia_lcd_write_string("2", 1);
	nokia_lcd_set_cursor(46, 25);
	nokia_lcd_write_string("ZONE ", 1);
	nokia_lcd_write_string("4", 1);
	nokia_lcd_set_cursor(30, 39);
	nokia_lcd_write_string("back ", 1);
				
	nokia_lcd_draw_vline(0, 37, 42);
	nokia_lcd_draw_hline(0, 83, 18);
	nokia_lcd_draw_hline(0, 83, 37);
	
	nokia_lcd_draw_hline(0, 83, 0);
	nokia_lcd_draw_hline(0, 83, 47);	
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void render_zone_selection() {
	draw_dss_zone_selector();
	switch (selected_item)	{
		case SEIT_ZONE_1:
			nokia_lcd_invert_area(0, 1, 41, 17);		
		break;

		case SEIT_ZONE_2:
			nokia_lcd_invert_area(43, 1, 83, 17);
		break;

		case SEIT_ZONE_3:		
			nokia_lcd_invert_area(0, 19, 41, 36);		
		break;
		
		case SEIT_ZONE_4:
			nokia_lcd_invert_area(43, 19, 83, 36);
		break;
		
		case SEIT_ZONE_BACK:
			nokia_lcd_invert_area(0, 38, 83, 46);		
		break;
	}
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void render_field_selection() {
	
	if (selected_field == SF_NONE) {
		return;
	}
		
	uint8_t x1 = 0; uint8_t y1 = 0; uint8_t x2 = 0;	uint8_t y2 = 0;
	
//rendering wslot fields
	if (selected_field <= SF_ZS_DISCARD)	{
		
		switch(selected_field) {                                            //x1, x2 selection
			case SF_WS1: case SF_WS2: case SF_WS3: case SF_REPEAT:
				x1 = 0;
				x2 = 19;
			break;
			case SF_WS1_HH: case SF_WS2_HH: case SF_WS3_HH:
				x1 = 31;
				x2 = 41;
			break;
			case SF_WS1_MI: case SF_WS2_MI: case SF_WS3_MI:
				x1 = 46;
				x2 = 56;
			break;
			case SF_WS1_DUR: case SF_WS2_DUR: case SF_WS3_DUR:
				x1 = 70;
				x2 = 80;
			break;
			case SF_MON:
			case SF_TUE:
			case SF_WED:
			case SF_THU:			
			case SF_FRI:			
			case SF_SAT:			
			case SF_SUN:
				x1 = (selected_field - SF_MON)*9 + 21;
				x2 = x1 + 8;
			break;
			case SF_ZS_OK:
				x1 = 20;
				x2 = 51;
			break;
			case SF_ZS_DISCARD:
				x1 = 52;
				x2 = 83;
			break;
		}

		switch(selected_field) {                                            //y1, y2 selection		
			case SF_WS1: case SF_WS1_HH: case SF_WS1_MI: case SF_WS1_DUR:
				y1 = 0;
				y2 = 9;			
			break;
			case SF_WS2: case SF_WS2_HH: case SF_WS2_MI: case SF_WS2_DUR:
				y1 = 9;
				y2 = 18;
			break;			
			case SF_WS3: case SF_WS3_HH: case SF_WS3_MI: case SF_WS3_DUR:
				y1 = 18;
				y2 = 27;
			break;
			case SF_REPEAT:
				y1 = 27;
				y2 = 36;
			break;
			case SF_MON:
			case SF_TUE:
			case SF_WED:
			case SF_THU:
			case SF_FRI:
			case SF_SAT:
			case SF_SUN:									
				y1 = 27;
				y2 = 37;
			break;
			
			case SF_ZS_OK:
			case SF_ZS_DISCARD:
				y1 = 39;
				y2 = 47;
			break;
			
		}
	}

	nokia_lcd_invert_area(x1, y1, x2, y2);

	
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void render_twcs_wslot(uint8_t ws_anum) {

	register uint8_t y = ws_anum * 9 + 1;

	nokia_lcd_put_pic(1, y, PIC_CONFO_SLOT);
	nokia_lcd_gen_dig(12, y + 1, ws_anum + 1); 
	
	if (twcs.ws[ws_anum].is_on) {
		nokia_lcd_put_pic(24, y, PIC_VALVE_READY);
		nokia_lcd_print_uint(32, y + 1, twcs.ws[ws_anum].start_hour, 2);
		nokia_lcd_draw_hline(42, 45, y + 4);
		nokia_lcd_print_uint(47, y + 1 , twcs.ws[ws_anum].start_min, 2);

		nokia_lcd_put_pic(63, y , PIC_VALVE_WATERING);
		nokia_lcd_print_uint(71, y + 1, twcs.ws[ws_anum].duration, 2);
		nokia_lcd_put_char_3x7(81, y + 1, 'm');	
	} else {
		// 0 for O		
		nokia_lcd_set_dig_font(SL_FONT_4x7);
		nokia_lcd_gen_dig(32, y + 1, 0);
		// drawing letter 2 two times
		for(uint8_t i = 0; i < 2; i++) {                     
			nokia_lcd_draw_vline(y + 1, y + 7, 37 + i*5);
			nokia_lcd_draw_hline(38 + i*5, 40 + i*5, y + 1);
			nokia_lcd_draw_hline(38 + i*5, 39 + i*5, y + 4);
		}
		
	}
	
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void render_zone_setup(uint8_t is_read_only) {
	
	nokia_lcd_set_dig_font(SL_FONT_4x7);

	for(uint8_t i = 0; i < 3; i++) {
		render_twcs_wslot(i);
	}
	
	#define L_RC_POS_Y  28
	
	nokia_lcd_put_pic(2, L_RC_POS_Y , PIC_REPEAT_TYPE);
	switch (twcs.rc.type) {
		case RC_EVERY_DAY:
			nokia_lcd_set_cursor(32, L_RC_POS_Y );
			nokia_lcd_write_string_P(PSTR("ALL DAYS"));
		break;
		case RC_WEEKDAYS:
			nokia_lcd_put_str(22, L_RC_POS_Y , "Mo");
			nokia_lcd_put_str(31, L_RC_POS_Y , "Tu");
			nokia_lcd_put_str(40, L_RC_POS_Y , "We");			
			nokia_lcd_put_str(49, L_RC_POS_Y , "Th");			
			nokia_lcd_put_str(58, L_RC_POS_Y , "Fr");			
 			nokia_lcd_put_str(67, L_RC_POS_Y , "Sa");
 			nokia_lcd_put_str(76, L_RC_POS_Y , "Su");			
			 
			for(uint8_t i = 0; i < 7; i++) {                                                // render selected week days as under line
				if(twcs.rc.data & _BV(i)) {
					register uint8_t x_start = 22 + i*9;
					nokia_lcd_draw_hline(x_start, x_start + 6, L_RC_POS_Y + 8);
				}
			}
		break;
	}
	nokia_lcd_put_char_3x7(0, 40, 'z');
	nokia_lcd_gen_dig(5, 40, selected_item);

	if(is_read_only) {
		nokia_lcd_set_cursor(35, 39);
		nokia_lcd_write_string("back ", 1);		
		nokia_lcd_invert_area(15, 38, 83, 47);
	} else {
		nokia_lcd_put_pic(30, 40, PIC_OK);
		nokia_lcd_put_pic(65, 40, PIC_DISCARD);		
		render_field_selection();
	}

}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void render_tws_item(uint8_t num, uint8_t line, uint8_t rti_type) {
	
	register uint8_t wline_end = twsrp.cur_window_start_line + TWS_RENDERER_LINES_SIZE - 1;
	
	if(line < twsrp.cur_window_start_line || line > wline_end) {
		return;
	}
	
	line -= twsrp.cur_window_start_line;

	register uint8_t y = 8*(line + 1) + 1;

	if(rti_type == RTI_DATE_ONLY) {
		nokia_lcd_print_uint(18, y, twsea[num].start_day, 2);
		nokia_lcd_set_dig_font(SL_FONT_3x5);
		nokia_lcd_gen_slash(26, y);
		nokia_lcd_print_uint(30, y, twsea[num].start_month, 2);
		register uint8_t dow = (twsea[num].status & TWSE_STATE_DOW_MASK) >> 5;                   // get day of week 
		nokia_lcd_put_str_3x5_P(45, y, DOW_NAMES[dow]);
	} else if(rti_type == RTI_NUM_ONLY) {
		nokia_lcd_gen_dig(1, y, num);                                                            // record num
	} else {
		nokia_lcd_gen_dig(1, y, num);                                                            // record num
		nokia_lcd_gen_dig(8, y, ((twsea[num].zoneslot & TWSE_Z_NUM_MASK ) >> 2) + 1);			 // zone num
		nokia_lcd_gen_dig(14, y, (twsea[num].zoneslot & TWSE_WS_NUM_MASK) + 1);					 // water slot num
	
		nokia_lcd_print_uint(20, y, twsea[num].start_hh, 2 );                                    // render start time
		nokia_lcd_draw_hline(28, 29, y + 2 );
		nokia_lcd_print_uint(31, y, twsea[num].start_mi, 2 );	

		nokia_lcd_print_uint(43, y, twsea[num].stop_hh, 2 );									 // render stop time
		nokia_lcd_draw_hline(51, 29, y + 2 );
		nokia_lcd_print_uint(54, y, twsea[num].stop_mi, 2 );
		
		--y;
	 	nokia_lcd_draw_dot_vline(y, y + 7, 6);
	 	nokia_lcd_draw_dot_vline(y, y + 7, 12);
	 	nokia_lcd_draw_dot_vline(y, y + 7, 18);
	 	nokia_lcd_draw_dot_vline(y, y + 7, 40);
	 	nokia_lcd_draw_dot_vline(y, y + 7, 62);
		++y;
	
		register uint8_t pic_type = 0;

		register uint8_t status = twsea[num].status;

		switch (status & TWSE_STATE_MASK) {
			case TWSE_STATE_ACTIVE:
				pic_type = PIC_ACTIVE;
			break;
			case TWSE_STATE_CANCELED:
				pic_type = PIC_CANCELED;
			break;
		}
		if(pic_type) {
			nokia_lcd_put_pic(71, y, pic_type);
		}
	
		pic_type = 0;
		if (status & _BV(TWSE_SHIFTED) && status & _BV(TWSE_CORRECTED) ) {
			pic_type = PIC_SHIFT_COR;
		} else if(status & _BV(TWSE_SHIFTED)) {
			pic_type = PIC_SHIFTED;		
		} else if(status & _BV(TWSE_CORRECTED)) {
			pic_type = PIC_CORRECTED;		
		}

		if(pic_type) {
			nokia_lcd_put_pic(64, y, pic_type);
		}
	
	}
	nokia_lcd_draw_hline(0, 77, y + 6);

	
	
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void render_tws() {
	
	nokia_lcd_set_dig_font(SL_FONT_3x5);					
	
	nokia_lcd_draw_vline(1, 5, 1);																    // # char
	nokia_lcd_draw_vline(1, 5, 3);
	nokia_lcd_draw_vline(2, 2, 2);
	nokia_lcd_draw_vline(4, 4, 2);
	
	nokia_lcd_put_char_3x5(8, 1, 'Z');
	nokia_lcd_put_char_3x5(14, 1, 'S');	
	
	nokia_lcd_put_str_3x5(20, 1, "START");		
	nokia_lcd_put_str_3x5(44, 1, "STOP");
	nokia_lcd_put_str_3x5(64, 1, "INF");
	
 	nokia_lcd_draw_dot_vline(0, 7, 6);
 	nokia_lcd_draw_dot_vline(0, 7, 12);	
 	nokia_lcd_draw_dot_vline(0, 7, 18);	
 	nokia_lcd_draw_dot_vline(0, 7, 40);	
 	nokia_lcd_draw_dot_vline(0, 7, 62);	
	nokia_lcd_draw_vline(7, 47, 77);
	
	
	//TWS rendering
	
	uint8_t items_cnt = 0;
	
	if((twsea[0].status & TWSE_STATE_MASK) != TWSE_STATE_NEW) {														// render 0 element of tws array
		render_tws_item(0, items_cnt++, RTI_DATE_ONLY);
		render_tws_item(0, items_cnt++, RTI_FULL);
	} else {
		render_tws_item(0, items_cnt++, RTI_NUM_ONLY);
	}
	
	for(uint8_t i = 1; i < TWSEA_LEN; i++) {
		if((twsea[i].status & TWSE_STATE_MASK) != TWSE_STATE_NEW) {
			if(twsea[i].start_day != twsea[i-1].start_day || twsea[i].start_month != twsea[i-1].start_month) {     // render other elements of tws array    
				render_tws_item(i, items_cnt++, RTI_DATE_ONLY);
			}
			render_tws_item(i, items_cnt++, RTI_FULL);
		} else {
			render_tws_item(i, items_cnt++, RTI_NUM_ONLY);			
		}
	}
	
	twsrp.cur_lines_total = items_cnt;
	
	if(twsrp.cur_window_start_line != 0) {
		nokia_lcd_put_pic(79, 9, PIC_SCROLL_UP);
	}
	if(twsrp.cur_window_start_line != twsrp.cur_lines_total - TWS_RENDERER_LINES_SIZE) {
		nokia_lcd_put_pic(79, 41, PIC_SCROLL_DOWN);
	}

	nokia_lcd_draw_hline(0, 83, 0);																	// top h line
	nokia_lcd_draw_hline(0, 83, 7);																	// header h line
	nokia_lcd_draw_hline(0, 83, 47);																// bottom h line
	
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void render_manual_valve() {
	nokia_lcd_set_cursor(11, 1);
	nokia_lcd_write_string("ZONE", 1);
	nokia_lcd_set_cursor(53, 1);
	nokia_lcd_write_string("TIME", 1);

// 	nokia_lcd_draw_vline(0, 37, 43);	
// 	nokia_lcd_draw_hline(0, 83, 10);
// 	nokia_lcd_draw_hline(0, 83, 38);
		
	nokia_lcd_put_pic(38, 40, PIC_DISCARD);
	
	nokia_lcd_set_dig_font(DL_FONT_8x14);
	if (mvp.status & _BV(MVP_ALL_ZONES)) {
		nokia_lcd_set_cursor(7, 15);
		nokia_lcd_write_string("ALL", 2);
	} else {
		nokia_lcd_gen_dig(17, 15, mvp.status + 1);
	}
	nokia_lcd_print_uint(53, 15, mvp.duration, 2);

	switch(selected_field) {
		case SF_MV_ZONE:
			nokia_lcd_invert_area(2, 14, 40, 30);
		break;
		
		case SF_MV_DUR:
			nokia_lcd_invert_area(45, 14, 81, 30);
		break;
	}	

	//rendering selection
	
	switch (selected_item)	{
		case SEIT_MAN_VALVE_ZONE:
			nokia_lcd_invert_area(0, 0, 42, 9);
		break;
		case SEIT_MAN_VALVE_TIME:
			nokia_lcd_invert_area(43, 0, 83, 9);		
		break;
		case SEIT_MAN_VALVE_DISCARD:
			nokia_lcd_invert_area(0, 39, 83, 47);
		break;
	}

}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void render_ar_manual_status() {
	
	nokia_lcd_set_cursor(48, 2);
	nokia_lcd_write_string_P(PSTR("MANUAL"));
	if(mvp.status & _BV(MVP_ALL_ZONES)) {
		nokia_lcd_put_str(48, 11, "ALL");
		nokia_lcd_put_str(64, 11, "ZONES");
		nokia_lcd_set_dig_font(DL_FONT_8x14);
		nokia_lcd_gen_dig(48, 21, (mvp.status & MVP_ZONE_BITS_MASK) + 1);
		nokia_lcd_put_str(63, 29, "of");
		nokia_lcd_gen_dig(74, 21, 4);
	} else {
		nokia_lcd_set_cursor(54, 11);
		nokia_lcd_write_string("ZONE", 1);
		nokia_lcd_set_dig_font(DL_FONT_8x14);
		nokia_lcd_gen_dig(61, 21, (mvp.status & MVP_ZONE_BITS_MASK) + 1);
	}
	if(valve_delay != 255) {
		nokia_lcd_put_pic(63, 39, PIC_SAND_WATCH);
	} else {
		nokia_lcd_set_dig_font(SL_FONT_4x7);
		nokia_lcd_print_uint(56, 39, mvp.duration_half_sec_count_down/120, 2);
		nokia_lcd_set_dig_font(SL_FONT_3x5);
		nokia_lcd_print_uint(67, 41, (mvp.duration_half_sec_count_down%120)/2, 2);
	}
	
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void render_ar_rain_delay_status() {
	nokia_lcd_set_cursor(54, 2);
	nokia_lcd_write_string("RAIN", 1);	
	nokia_lcd_set_cursor(51, 11);
	nokia_lcd_write_string("DELAY", 1);
	
	nokia_lcd_set_dig_font(DL_FONT_8x14);
	nokia_lcd_print_uint(54, 21, mvp.duration, 2);
	nokia_lcd_set_dig_font(SL_FONT_4x7);
	nokia_lcd_print_uint(54, 39, mvp.duration_half_sec_count_down/120, 2);	
	nokia_lcd_set_dig_font(SL_FONT_3x5);	
	nokia_lcd_print_uint(68, 41, (mvp.duration_half_sec_count_down%120)/2, 2);
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void render_autorun_status() {
	
	nokia_lcd_draw_hline(0, 83, 0);
	nokia_lcd_draw_hline(0, 83, 47);	
	
	nokia_lcd_set_dig_font(DL_FONT_7x12);														// set font for time digits
	nokia_lcd_print_uint(1, 2, rt.hour, 2);														// print hours
	nokia_lcd_print_uint(26, 2, rt.min, 2);														// print minutes

	if(main_register & _BV(MR_HS_FLAG) ) {														// blink with dots
		nokia_lcd_draw_hline(22, 23, 5);
		nokia_lcd_draw_hline(22, 23, 6);	
		nokia_lcd_draw_hline(22, 23, 9);	
		nokia_lcd_draw_hline(22, 23, 10);		
	}

	nokia_lcd_set_dig_font(SL_FONT_4x7);														// set font for date
	nokia_lcd_print_uint(1, 19, rt.mday, 2);													// print day
	nokia_lcd_print_uint(14, 19, rt.mon, 2);													// print month
	
	nokia_lcd_set_cursor(27, 19);
	nokia_lcd_write_string_P(DOW_NAMES[rt.wday]);												// print dow
	
	nokia_lcd_print_uint(12, 29, 20, 2);														// print year
	nokia_lcd_print_uint(22, 29, rt.year8_t, 2);
	
	nokia_lcd_draw_vline(0, 47, 45);
	
	nokia_lcd_draw_hline(0, 45, 37);															// s adj above line
	nokia_lcd_set_dig_font(SL_FONT_3x5);
	nokia_lcd_put_str(1, 39, "adj");
	#define SAS_POS_X 20
	#define SAS_POS_Y 39


	nokia_lcd_set_dig_font(SL_FONT_4x7);	

	nokia_lcd_print_uint(SAS_POS_X, SAS_POS_Y, s_adj, 2);	
	nokia_lcd_gen_dig(SAS_POS_X + 10, SAS_POS_Y, 0);	
	nokia_lcd_gen_slash(SAS_POS_X + 16, SAS_POS_Y);	
	
 	nokia_lcd_set_pixel(SAS_POS_X + 16, SAS_POS_Y + 1, 1);
 	nokia_lcd_set_pixel(SAS_POS_X + 19, SAS_POS_Y + 5, 1);	
	
 	nokia_lcd_set_pixel(SAS_POS_X + 16 +2, SAS_POS_Y+1, 0);	
 	nokia_lcd_set_pixel(SAS_POS_X + 16 +3, SAS_POS_Y+1, 1);		

	
	if(run_mode == RM_MANUAL) {
		render_ar_manual_status();
	} else if(run_mode == RM_RAIN_DELAY) {
		render_ar_rain_delay_status();
	} else {
		true_water_schedule_element* e = (true_water_schedule_element*)&twsea;                    // make pointer to first twsea element
		switch (e->status & TWSE_STATE_MASK ) {
			case TWSE_STATE_PLANNED:
				nokia_lcd_set_cursor(54, 2);
				nokia_lcd_write_string_P(PSTR("NEXT"));
				nokia_lcd_set_dig_font(DL_FONT_8x14);
				nokia_lcd_gen_dig(58, 15,  ((e->zoneslot & TWSE_Z_NUM_MASK) >> 2) + 1);
				nokia_lcd_set_dig_font(SL_FONT_4x7);
				nokia_lcd_gen_dig(71, 23,  (e->zoneslot & TWSE_WS_NUM_MASK) + 1);
				
				nokia_lcd_put_pic(53, 38, PIC_VALVE_WATERING);                                     // display duration of a session
				register uint8_t dur = (uint8_t) get_diff_in_min(e->stop_hh, e->stop_mi, e->start_hh, e->start_mi);
 				nokia_lcd_print_uint(62, 39, dur, 2);                                              
				nokia_lcd_set_cursor(72, 39);
				nokia_lcd_write_string("m", 1);
			break;
			
			case TWSE_STATE_PREPARE:
				nokia_lcd_set_cursor(54, 2);
				nokia_lcd_write_string_P(PSTR("WAIT"));
				nokia_lcd_set_dig_font(DL_FONT_8x14);
				nokia_lcd_gen_dig(58, 15,  ((e->zoneslot & TWSE_Z_NUM_MASK) >> 2) + 1);
				nokia_lcd_set_dig_font(SL_FONT_4x7);
				nokia_lcd_gen_dig(71, 23,  (e->zoneslot & TWSE_WS_NUM_MASK) + 1);
				nokia_lcd_put_pic(61, 35, PIC_SAND_WATCH);
			break;
			
			case TWSE_STATE_ACTIVE:
				nokia_lcd_set_cursor(48, 2);
				nokia_lcd_write_string_P(PSTR("ACTIVE"));
				nokia_lcd_set_dig_font(DL_FONT_8x14);
				nokia_lcd_gen_dig(58, 11,  ((e->zoneslot & TWSE_Z_NUM_MASK) >> 2) + 1);
				nokia_lcd_set_dig_font(SL_FONT_4x7);
				nokia_lcd_gen_dig(71, 19,  (e->zoneslot & TWSE_WS_NUM_MASK) + 1);
				
				register uint16_t sec_left = get_diff_in_sec_with_curtime(e->stop_hh, e->stop_mi, 0);
				nokia_lcd_print_uint(58, 29, (uint8_t)(sec_left / 60), 2);                                              // time left
				nokia_lcd_set_dig_font(SL_FONT_3x5);
				nokia_lcd_print_uint(69, 31, (uint8_t)(sec_left % 60), 2);

				nokia_lcd_draw_hline(46, 83, 37);
				nokia_lcd_put_str(47, 39, "next");
				register true_water_schedule_element* en = (true_water_schedule_element*)&twsea;                        // searching for next twse
				register uint8_t i = 0;
				do {
					++en;
					++i;
				} while((en->status & TWSE_STATE_MASK) != TWSE_STATE_PLANNED && i < TWSEA_LEN);
				
				nokia_lcd_gen_dig(72, 41, ((en->zoneslot & TWSE_Z_NUM_MASK) >> 2) + 1);
				nokia_lcd_gen_dig(79, 41, (en->zoneslot & TWSE_WS_NUM_MASK) + 1);
			break;
			
			default:
				nokia_lcd_set_cursor(48, 4);
				nokia_lcd_write_string_P(PSTR("PLEASE"));
				
				nokia_lcd_set_cursor(48, 15);
				nokia_lcd_write_string_P(PSTR("CONFIG"));				
				
				nokia_lcd_set_cursor(57, 26);
				nokia_lcd_write_string_P(PSTR("THE"));								
				
				nokia_lcd_set_cursor(48, 37);				
				nokia_lcd_write_string_P(PSTR("DEVICE"));
				
			break;
		} 
		
	}
	
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void render_autorun() {
//	UARTPrintUint(device_sub_state, 10);
	if(device_sub_state == DSS_AUTORUN_MENU) {
		nokia_lcd_draw_hline(0, 83, 15);
		nokia_lcd_draw_hline(0, 83, 31);
		nokia_lcd_set_cursor(24, 4);
		nokia_lcd_write_string("STATUS", 1);
		nokia_lcd_set_cursor(9, 19);
		nokia_lcd_write_string("VIEW CONFIG", 1);
		nokia_lcd_set_cursor(3, 35);
		nokia_lcd_write_string("VIEW SCHEDULE", 1);
	
		switch(device_ar_state) {
			case DAS_STATUS:
				nokia_lcd_invert_area(0, 1, 83, 14);
			break;
			case DAS_VIEW_CONFIG:
				nokia_lcd_invert_area(0, 15, 83, 30);
			break;
			case DAS_VIEW_SCHEDULE:
				nokia_lcd_invert_area(0, 32, 83, 47);
			break;
		}
		
		nokia_lcd_draw_hline(0, 83, 0);
		nokia_lcd_draw_hline(0, 83, 47);		

		
	} else {
		switch(device_ar_state) {
			case DAS_STATUS:
				render_autorun_status();
			break;
			case DAS_VIEW_CONFIG:
				switch(device_sub_state) {
					case DSS_ZONE_SELECTION:
						render_zone_selection();
					break;
					case DSS_ZONE_VIEW:
						render_zone_setup(TRUE);
					break;
				}
			break;
			case DAS_VIEW_SCHEDULE:
				render_tws();			
//				nokia_lcd_write_string("SCH SC", 2);
			break;
		}
		
	}
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void render_rain_delay_selection() {
	nokia_lcd_set_cursor(0, 0);
	nokia_lcd_write_string_P(PSTR("SET RAIN DELAY"));
	nokia_lcd_set_dig_font(DL_FONT_8x14);
	nokia_lcd_print_uint(16, 15, mvp.duration, 2);
	nokia_lcd_set_cursor(48, 23);	
	nokia_lcd_write_string_P(PSTR("hours"));	
	nokia_lcd_put_pic(38, 40, PIC_DISCARD);
	nokia_lcd_invert_area(0, 39, 83, 47);
}
//-----------------------------------------------------------------------------------
//  6 items are hardcoded
//-----------------------------------------------------------------------------------
void render_config_mode_menu_num(uint8_t num) {
	
	const uint8_t one_item_height = 8; // 48/6; 
	#define CMI_ITEMS_MAX    6
	#define CMI_OFFSET       3	
	#define CMI_SELECTED_OFFSET  7	
	
	for(uint8_t i = 0; i < CMI_ITEMS_MAX; i++) {
		if(i == num) {
			nokia_lcd_draw_vline(i * one_item_height, (i + 1 )* one_item_height -1, 83 - CMI_SELECTED_OFFSET);			
			nokia_lcd_draw_hline(83 - CMI_SELECTED_OFFSET, 83 - CMI_OFFSET, i * one_item_height - 1); 
			nokia_lcd_draw_hline(83 - CMI_SELECTED_OFFSET, 83 - CMI_OFFSET, (i +1) * one_item_height);			
		} else {
			nokia_lcd_draw_vline(i * one_item_height, (i + 1 )* one_item_height -1, 83 - CMI_OFFSET);
		}
	}
	
}
//-----------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------
void lcd_processing() {
	if(!lcd_update_flag) {
		return;
	}
	
	if(lcd_update_flag == LUF_MANUAL) {                                                              // reset delay only in case of manual actions
		lcd_backlight_off_delay = LCD_BACKLIGHT_HS_DUR;                                              // set lcd back light delay 
		LCD_BL_PORT |= _BV(LCD_BL_PIN);																 // turn on lcd back light
		
		menu_return_timeout = MRT_HS;																 // set MRT 
	}

	lcd_update_flag = FALSE;
	
	nokia_lcd_clear();
	
//-------------------------------------------------------------------------------------------
//-------- AUTORUN SCREENS-------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
	if(run_mode != RM_CONFIG) {
		render_autorun();
		nokia_lcd_render();
		return;	
	}
//-------------------------------------------------------------------------------------------
//       CONFIG SCREENS
//-------------------------------------------------------------------------------------------
	#define LCD_PROC_CFG_MENU_LEFT_OFFSET  12
	#define LCD_PROC_CFG_MENU_TOP_OFFSET  7
	
	nokia_lcd_set_cursor(LCD_PROC_CFG_MENU_LEFT_OFFSET, LCD_PROC_CFG_MENU_TOP_OFFSET);

	if(device_sub_state == DSS_NONE){
		render_config_mode_menu_num(device_state - 1);											 // not applcable for hello state !!!
	}
	
	switch (device_state) {
		case DS_HELLO_STATE:
// 			nokia_lcd_write_string("HELLO", 2);
			nokia_lcd_invert_area(10, 10, 30, 30 );
		break;
//-------------------------------------------------------------------------------------------
//-------- DS RAIN DELAY --------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
		case DS_RAIN_DELAY:
			switch(device_sub_state){
				case DSS_NONE:
					nokia_lcd_write_string_bold_P(PSTR("SET"));
					nokia_lcd_set_cursor(LCD_PROC_CFG_MENU_LEFT_OFFSET, LCD_PROC_CFG_MENU_TOP_OFFSET + 10);
					nokia_lcd_write_string_bold_P(PSTR("RAIN"));
					nokia_lcd_set_cursor(LCD_PROC_CFG_MENU_LEFT_OFFSET, LCD_PROC_CFG_MENU_TOP_OFFSET + 20);
					nokia_lcd_write_string_bold_P(PSTR("DELAY"));
				break;
				case DSS_RAIN_DELAY_SELECTION:
					render_rain_delay_selection(); 
				break;
			}
		break;
//-------------------------------------------------------------------------------------------
//-------- DS WATERING CONFIG----------------------------------------------------------------
//-------------------------------------------------------------------------------------------
		case DS_WATERING_CONFIG:
			switch(device_sub_state) {
				case DSS_NONE:
					nokia_lcd_write_string_bold_P(PSTR("ZONE"));
					nokia_lcd_set_cursor(LCD_PROC_CFG_MENU_LEFT_OFFSET, LCD_PROC_CFG_MENU_TOP_OFFSET + 10);
					nokia_lcd_write_string_bold_P(PSTR("WATERING"));
					nokia_lcd_set_cursor(LCD_PROC_CFG_MENU_LEFT_OFFSET, LCD_PROC_CFG_MENU_TOP_OFFSET + 20);
					nokia_lcd_write_string_bold_P(PSTR("CONFIG"));
				break;
				case DSS_ZONE_SELECTION:                    //refactor this!
					render_zone_selection();
				break;
				case DSS_ZONE_SETUP:
					render_zone_setup(FALSE);
				break;
			}
		break;
//-------------------------------------------------------------------------------------------
//-------- SET DATE TIME---------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
		case DS_SET_DATE_TIME:
			switch(device_sub_state) {
				case DSS_NONE:
					nokia_lcd_write_string_bold_P(PSTR("SET"));
					nokia_lcd_set_cursor(LCD_PROC_CFG_MENU_LEFT_OFFSET, LCD_PROC_CFG_MENU_TOP_OFFSET + 10);
					nokia_lcd_write_string_bold_P(PSTR("TIME AND"));
					nokia_lcd_set_cursor(LCD_PROC_CFG_MENU_LEFT_OFFSET, LCD_PROC_CFG_MENU_TOP_OFFSET + 20);
					nokia_lcd_write_string_bold_P(PSTR("DATE"));
				break;
				case DSS_DT_SELECTION:
					render_datetime_selection();
				break;
			}
		break;
//-------------------------------------------------------------------------------------------
//-------- VIEW SCHEDULE --------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
		case DS_VIEW_SCHEDULE:
			switch(device_sub_state) {
				case DSS_NONE:
					nokia_lcd_write_string_bold_P(PSTR("VIEW"));
					nokia_lcd_set_cursor(LCD_PROC_CFG_MENU_LEFT_OFFSET, LCD_PROC_CFG_MENU_TOP_OFFSET + 10);
					nokia_lcd_write_string_bold_P(PSTR("WATERING"));
					nokia_lcd_set_cursor(LCD_PROC_CFG_MENU_LEFT_OFFSET, LCD_PROC_CFG_MENU_TOP_OFFSET + 20);
					nokia_lcd_write_string_bold_P(PSTR("SCHEDULE"));				
				break;
				case DSS_TWS_TABLE:
					render_tws();
				break;
			}
		break;
//-------------------------------------------------------------------------------------------
//-------- SEASON ADJ------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
		case DS_SEASON_ADJ:
			switch(device_sub_state) {
				case DSS_NONE:
					nokia_lcd_write_string_bold_P(PSTR("SEASONAL"));
					nokia_lcd_set_cursor(LCD_PROC_CFG_MENU_LEFT_OFFSET, LCD_PROC_CFG_MENU_TOP_OFFSET + 10);
					nokia_lcd_write_string_bold_P(PSTR("ADJUST"));
				break;
				case DSS_SADJ_SETUP:
					render_sadj_setup();
				break;
			}
		break;
//-------------------------------------------------------------------------------------------
//-------- MANUAL ZONES ---------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
		case DS_MANUAL_ZONES      :
			switch(device_sub_state) {
				case DSS_NONE:
					nokia_lcd_write_string_bold_P(PSTR("MANUAL"));
					nokia_lcd_set_cursor(LCD_PROC_CFG_MENU_LEFT_OFFSET, LCD_PROC_CFG_MENU_TOP_OFFSET + 10);
					nokia_lcd_write_string_bold_P(PSTR("ZONES"));
					nokia_lcd_set_cursor(LCD_PROC_CFG_MENU_LEFT_OFFSET, LCD_PROC_CFG_MENU_TOP_OFFSET + 20);
					nokia_lcd_write_string_bold_P(PSTR("WATERING"));
				break;
				case DSS_MANUAL_VALVE_SELECTION:
				case DSS_MANUAL_VALVE_SETUP:
					render_manual_valve();
				break;
				
			}
		break;

		default:
		break;
	}
	
	nokia_lcd_render();
	
}
//-----------------------------------------------------------------------------------
//  reset cur_sp based on current time (rt)
//-----------------------------------------------------------------------------------
void reset_cur_sp() {
	
	cur_sp.year = rt.year8_t;
	cur_sp.month = rt.mon;
	cur_sp.day = rt.mday;
	cur_sp.dow = rt.wday;
	cur_sp.hh = rt.hour;
	cur_sp.mi = rt.min;
	cur_sp.zone = 0;
	cur_sp.ws = 0;
	
	do {                                                                      // adjust cur_sp minutes to 5 min quantization
		++cur_sp.mi;
	} while (cur_sp.mi%5 != 0);
	
	if(cur_sp.mi > 55) {
		cur_sp.mi = 55;
		increment_cur_sp_time();
	}
	
}

//-----------------------------------------------------------------------------------
ISR (INT0_vect) {
	ar_but_event = TRUE;
}
//-----------------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect) {
	main_register ^= _BV(MR_HS_FLAG);
	main_register |= _BV(MR_HS_LCD_UPD_FLAG) 
				   | _BV(MR_HS_VALVE_CNT_DOWN_FLAG) 
				   | _BV(MR_HS_SIGNAL_LED_FLAG) 
				   | _BV(MR_HS_LCD_BACKLIGHT_FLAG)
				   | _BV(MR_HS_MRT_FLAG);
	
	if(main_register & _BV(MR_HS_FLAG)) {                                                    // make 1 sec flags
		main_register |= _BV(MR_S_RTC_SYNCH_FLAG) | _BV(MR_S_VCS_FLAG);
	}

}
//-----------------------------------------------------------------------------------
void print_wca() {
	UARTPrintln("");
	UARTPrintln_P(PSTR("WATER CONFIG:"));
	
	for(uint8_t i = 0; i < MAX_ZONES; i++) {
		UARTPrintln("");
		UARTPrintValDec("ZONE ", i);
		UARTPrintValDec(" Valve=", wca[i].valve);
		UARTPrintValDec(" rc.type=", wca[i].rc.type);
		UARTPrintlnValDec(" rc.data=", wca[i].rc.data);
		
		for (uint8_t j = 0; j < MAX_WSLOTS_PER_ZONE; j++) {
			UARTPrint_P(PSTR(" ->wslot "));
			UARTPrintUint(j, 10);
			UARTPrint(": ");
			UARTPrintValDec("is_on=", wca[i].ws[j].is_on);
			UARTPrintValDec(" sh=", wca[i].ws[j].start_hour);
			UARTPrintValDec(" sm=", wca[i].ws[j].start_min);
			UARTPrintlnValDec(" dur=", wca[i].ws[j].duration);
		}
	 
	}
}
//-----------------------------------------------------------------------------------
void tws_print() {
	UARTPrintln_P(PSTR("TWSEA:"));
	for(uint8_t i = 0; i < TWSEA_LEN; i++) {
		for (uint8_t j = 0; j < 8; j ++) {
			UARTPrintUint( *((uint8_t*)twsea + j + i*8), 10) ;
			UARTPrint(" ");
		}
		
		UARTPrint(" |  ");		
		uint8_t tt = *((uint8_t*)twsea + i*8 + 1);
		UARTPrintUint(tt  >> 2, 10);
		UARTPrint(" ");		
		UARTPrintUint((tt & TWSE_WS_NUM_MASK), 10);		
		UARTPrint(" ");		

		tt = *((uint8_t*)twsea + i*8 );	
		register uint8_t ch = 33;	
		
		switch (tt & TWSE_STATE_MASK){
			case TWSE_STATE_NEW:
				ch = 'N';
			break;
			case TWSE_STATE_PLANNED:
				ch = 'P';
			break;
			case TWSE_STATE_PREPARE:
				ch = 'R';
			break;			
			case TWSE_STATE_ACTIVE:
				ch = 'A';
			break;			
			case TWSE_STATE_PASSED:
				ch = 'D';
			break;
			case TWSE_STATE_CANCELED:
				ch = 'N';
				UARTPrint_P(PSTR("CA"));
			break;
		}
		UARTPutChar(ch);
		UARTPrint(" ");
		if (tt & _BV(TWSE_SHIFTED)) {
			UARTPrint("S");
			UARTPrint(" ");
		}		
		if (tt & _BV(TWSE_CORRECTED)) {
			UARTPrint_P(PSTR("COR"));
			UARTPrint(" ");
		}
		UARTPrintln("");
	}	
	
}

//-----------------------------------------------------------------------------------
void print_cur_sp(){
	UARTPrint_P(PSTR("cur_sp: "));
	for (uint8_t i = 0; i < 7; i++)	{
		UARTPrintUint(*( (uint8_t*) &cur_sp + i), 10);
		UARTPrint(" ");
	}
	UARTPrintln("");
}

//-----------------------------------------------------------------------------------
//  
//-----------------------------------------------------------------------------------

inline void run_timer1_at_500msec() {
	OCR1A  = 16625; //31249;                     // hopefully compiler do this 16-bit assignment :)
	TIMSK1 = _BV(OCIE1A);
	TCCR1B = _BV(WGM12) | _BV(CS12);   // run at CTC mode with prescaler = 256 
	
}

//-----------------------------------------------------------------------------------
//  
//-----------------------------------------------------------------------------------
inline void leds_and_mrt_processing() {

	if(main_register & _BV(MR_HS_SIGNAL_LED_FLAG)) {                                                 // signal led processing
		switch(run_mode) {
			case RM_AUTORUN:
				if(main_register & _BV(MR_HS_FLAG)) {
					SLED_PORT |= _BV(SLED_PIN);
				} else {
					SLED_PORT &= ~_BV(SLED_PIN);					
				}
			break;
			case RM_MANUAL:
			case RM_RAIN_DELAY:
				SLED_PORT |= _BV(SLED_PIN);
			break;
			case RM_CONFIG:
				SLED_PORT &= ~_BV(SLED_PIN);
			break;
		}
		main_register &= ~_BV(MR_HS_SIGNAL_LED_FLAG);
	}

	if(main_register & _BV(MR_HS_LCD_BACKLIGHT_FLAG)) {												// process lcd backlight 
		
		main_register &= ~_BV(MR_HS_LCD_BACKLIGHT_FLAG);		
		
		if(lcd_backlight_off_delay) {
			--lcd_backlight_off_delay;
		} else {
			LCD_BL_PORT &= ~_BV(LCD_BL_PIN);
		}
		
	}
	
	if(main_register & _BV(MR_HS_MRT_FLAG)) {														// MRT processing
		main_register &= ~_BV(MR_HS_MRT_FLAG);
		
		if (menu_return_timeout) {
			--menu_return_timeout;
		} else {
			lcd_update_flag = LUF_AUTO;
			if (run_mode == RM_CONFIG) {
				if(device_sub_state) {
					device_sub_state = DSS_NONE;
				}
			} else {
				if (device_sub_state || device_ar_state) {
					device_sub_state = DSS_NONE;
					device_ar_state = DAS_STATUS;
				}
			}		
		}
	}
}
//-----------------------------------------------------------------------------------
//  
//-----------------------------------------------------------------------------------
void play_intro_animation() {

	nokia_lcd_put_screen(0);
	nokia_lcd_render();

	_delay_ms(700);
	nokia_lcd_set_cursor(30, 0);
	nokia_lcd_write_string("IRIGO", 2);
	nokia_lcd_render();
	_delay_ms(700);

	nokia_lcd_put_screen(1);
	nokia_lcd_set_cursor(30, 0);
	nokia_lcd_write_string("IRIGO", 2);
	nokia_lcd_set_cursor(54, 16);
	nokia_lcd_write_string_P(PSTR("WATER"));
	nokia_lcd_render();
	_delay_ms(700);

	nokia_lcd_put_screen(2);
	nokia_lcd_set_cursor(30, 0);
	nokia_lcd_write_string("IRIGO", 2);
	nokia_lcd_set_cursor(54, 16);
	nokia_lcd_write_string_P(PSTR("WATER"));
	nokia_lcd_set_cursor(60, 25);
	nokia_lcd_write_string_P(PSTR("tech"));
	nokia_lcd_render();
	_delay_ms(1000);
	
	nokia_lcd_invert_area(0, 0, 83, 47);
	nokia_lcd_render();
	_delay_ms(1200);

}

//-----------------------------------------------------------------------------------
//  return state of Shift-in register
//-----------------------------------------------------------------------------------
uint8_t SIR_get_state() {
	SIR_LATCH_PORT &= ~_BV(SIR_LATCH_PIN);
	SIR_LATCH_PORT |= _BV(SIR_LATCH_PIN);
	
	return SPI_MasterTransmit(0);
	
}

//-----------------------------------------------------------------------------------
//  
//-----------------------------------------------------------------------------------
void display_valves_conection_status() {
	
	re_button_signal = RE_NO_SIGNAL;

	while(!re_button_signal) {
		uint8_t vs = SIR_get_state();
		nokia_lcd_clear();
	
		for(uint8_t i = 0; i < 4; i++) {
			nokia_lcd_set_cursor(0, i*9);
			nokia_lcd_write_string_P(PSTR("Zone"));
			nokia_lcd_set_cursor(27, i*9);
			nokia_lcd_write_char(i + 49, 1);
			nokia_lcd_set_cursor(43, i*9);
			nokia_lcd_write_char('-', 1);
			nokia_lcd_set_cursor(60, i*9);
			if(vs & _BV(i)) {
				nokia_lcd_write_string_P(PSTR("OK"));			
			} else {
				nokia_lcd_write_string_P(PSTR("N/C"));
			}
		}
		nokia_lcd_set_cursor(0,40);
		nokia_lcd_write_string_P(PSTR("Push the knob"));	
	
		nokia_lcd_render();
	
		_delay_ms(100);										// have not idea why it's not working (in proteus) without a delay. mb will be ok in hardware.
	}

	re_button_signal = RE_NO_SIGNAL;

}

