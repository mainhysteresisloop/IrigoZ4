/*
 * SPIsi.c
 *
 * Created: 25.10.2016 15:09:20
 *  Author: USER
 */ 
#include <avr/io.h>
#include "SPIsi.h"

//--------------------------------------------------------------------------------------------------------------------------------
// to be absolete
void SPI_MasterInit() {
	DDRB |= _BV(PB3) | _BV(PB5) | _BV(PB2); // set MOSI and SCK and SS (!) pins as output, leaving MISO as input. SPI doesn't work if SS is not configured as OUTPUT (!!!)	
	SPSR = _BV(SPI2X); // wow !!!
	SPCR = _BV(SPE) | _BV(MSTR); // turn spi on, set as master, MSB transfered fist (DORD bit is 0)
/*	        SPI2X  SPR1  SPR0       SCK frequency
	        0     0     0           F_CPU/4
	        0     0     1           F_CPU/16
	        0     1     0           F_CPU/64
	        0     1     1           F_CPU/128
	        1     0     0           F_CPU/2
	        1     0     1           F_CPU/8
	        1     1     0           F_CPU/32
	        1     1     1           F_CPU/64 */
  /*    SPIF  - Флаг прерывания от SPI. уст в 1 по окончании передачи байта
        WCOL  - Флаг конфликта записи. Уст в 1 если байт не передан, а уже попытка записать новый.
        SPI2X - Удвоение скорости обмена.*/	
}
//--------------------------------------------------------------------------------------------------------------------------------
// to de retested everywhere and to be used instead ofSPI_MasterInit()
void SPI_master_start() {
	DDRB |= _BV(PB3) | _BV(PB5) | _BV(PB2); // set MOSI and SCK and SS (!) pins as output, leaving MISO as input. SPI doesn't work if SS is not configured as OUTPUT (!!!)	
	SPCR |= _BV(SPI2X)| _BV(SPE) | _BV(MSTR); // turn spi on, set as master, MSB transfered fist (DORD bit is 0)
/*	        SPI2X  SPR1  SPR0       SCK frequency
	        0     0     0           F_CPU/4
	        0     0     1           F_CPU/16
	        0     1     0           F_CPU/64
	        0     1     1           F_CPU/128
	        1     0     0           F_CPU/2
	        1     0     1           F_CPU/8
	        1     1     0           F_CPU/32
	        1     1     1           F_CPU/64 */
  /*    SPIF  - Флаг прерывания от SPI. уст в 1 по окончании передачи байта
        WCOL  - Флаг конфликта записи. Уст в 1 если байт не передан, а уже попытка записать новый.
        SPI2X - Удвоение скорости обмена.*/	
}
//--------------------------------------------------------------------------------------------------------------------------------
uint8_t SPI_MasterTransmit(uint8_t data) {
	/* Start transmission */
	SPDR = data;
	/* Wait for transmission complete */
	while(bit_is_clear(SPSR, SPIF));
	/*return data received from SPI slave device */
	return SPDR;
}
//--------------------------------------------------------------------------------------------------------------------------------
uint8_t SPI_get_data() {
	return SPDR;
}
//--------------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------------------

