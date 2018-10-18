/*
 * SPIsi.h
 *
 * Created: 25.10.2016 22:03:39
 *  Author: USER
 */ 


#ifndef SPISI_H_
#define SPISI_H_


/*
The four modes combine polarity and phase according to this table:
Mode	  | Clock Polarity | Clock Phase  |    Output Edge	| Data Capture   |
		  |		(CPOL)     |    (CPHA)    |                 |				 |
SPI_MODE0		  0				  0			    Falling			 Rising
SPI_MODE1		  0				  1			    Rising			 Falling
SPI_MODE2		  1				  0			    Rising			 Falling
SPI_MODE3		  1				  1			    Falling			 Rising
*/
#define SPI_set_mode3()   SPCR |= _BV(CPOL) | _BV(CPHA)


void SPI_MasterInit();
void SPI_master_start();
uint8_t SPI_MasterTransmit(uint8_t data);
uint8_t SPI_get_data();




#endif /* SPISI_H_ */