/*****************************************************************************
*** Arquivo: sensoray526.h
*** Conteudo: modulo sensoray526.
*** Autor: G. A. Borges. e F. B. Cavalcanti
*** Atualizacoes: 
	- 18-01-2011: criacao
	- 02-04-2011: contadores/encoders, LEDs, ADs
*****************************************************************************/
/*! \file sensoray526.h 
* \brief Arquivo cabecalho do modulo sensoray526. */
#ifndef SENSORAY526_H
#define SENSORAY526_H

#ifdef __cplusplus
 extern "C" {
#endif 

// Definicoes de uso externo:
#define S526_REG_TCR 0x00  /* Timer control register */
#define S526_REG_WDC 0x02  /* Watchdog timer control register */
#define S526_REG_DAC 0x04  /* DAC control */
#define S526_REG_ADC 0x06  /* ADC control */
#define S526_REG_ADD 0x08  /* DAC data ADC data */
#define S526_REG_DIO 0x0A  /* Digital I/O control Digital I/O data */
#define S526_REG_IER 0x0C  /* Interrupt enable register */
#define S526_REG_ISR 0x0E  /* Interrupt status register Interrupt status register */
#define S526_REG_MSC 0x10  /* Miscellaneous register Miscellaneous register */
#define S526_REG_C0L 0x12  /* Counter 0 preload register low word Counter 0 data low word */
#define S526_REG_C0H 0x14  /* Counter 0 preload register high word Counter 0 data high word */
#define S526_REG_C0M 0x16  /* Counter 0 mode register */
#define S526_REG_C0C 0x18  /* Counter 0 control register Counter 0 status register */
#define S526_REG_C1L 0x1A  /* Counter 1 preload register low word Counter 1 data low word */
#define S526_REG_C1H 0x1C  /* Counter 1 preload register high word Counter 1 data high word */
#define S526_REG_C1M 0x1E  /* Counter 1 mode register */
#define S526_REG_C1C 0x20  /* Counter 1 control register Counter 1 status register */
#define S526_REG_C2L 0x22  /* Counter 2 preload register low word Counter 2 data low word */
#define S526_REG_C2H 0x24  /* Counter 2 preload register high word Counter 2 data high word */
#define S526_REG_C2M 0x26  /* Counter 2 mode register */
#define S526_REG_C2C 0x28  /* Counter 2 control register Counter 2 status register */
#define S526_REG_C3L 0x2A  /* Counter 3 preload register low word Counter 3 data low word */
#define S526_REG_C3H 0x2C  /* Counter 3 preload register high word Counter 3 data high word */
#define S526_REG_C3M 0x2E  /* Counter 3 mode register */
#define S526_REG_C3C 0x30  /* Counter 3 control register Counter 3 status register */
#define S526_REG_EED 0x32  /* EEPROM data EEPROM data */
#define S526_REG_EEC 0x34  /* EEPROM interface command Signature */

/* Counter x mode register bit */
#define S526_REG_CxM_PRELOADREGISTER_PR0			(0)  
#define S526_REG_CxM_PRELOADREGISTER_PR1			(1<<14)
#define S526_REG_CxM_LATCHONREAD					(0)  
#define S526_REG_CxM_LATCHONEVENT 					(1<<13)
#define S526_REG_CxM_COUNTDIRECTIONMODE_QUADRATURE 	(0)  
#define S526_REG_CxM_COUNTDIRECTIONMODE_SOFTWARE 	(1<<12)
#define S526_REG_CxM_COUNTDIRECTION_UP 				(0)  
#define S526_REG_CxM_COUNTDIRECTION_DOWN 			(1<<11)
#define S526_REG_CxM_CLOCKSOURCE_CLKA_RISE			(0<<9)  
#define S526_REG_CxM_CLOCKSOURCE_CLKA_FALL			(1<<9)  
#define S526_REG_CxM_CLOCKSOURCE_INTERNAL			(2<<9)  
#define S526_REG_CxM_CLOCKSOURCE_INTERNAL_2			(3<<9)  
#define S526_REG_CxM_CLOCKSOURCE_QUAD_X1			(0<<9)  
#define S526_REG_CxM_CLOCKSOURCE_QUAD_X2			(1<<9)  
#define S526_REG_CxM_CLOCKSOURCE_QUAD_X4			(2<<9)  
#define S526_REG_CxM_COUNTENABLE_DISABLED			(0<<7)  
#define S526_REG_CxM_COUNTENABLE_ENABLED			(1<<7)  
#define S526_REG_CxM_COUNTENABLE_HARDWARE			(2<<7)  
#define S526_REG_CxM_COUNTENABLE_HARDWARE_INVERTED 	(3<<7)  
#define S526_REG_CxM_HARDWARECOUNTENABLE_CEN		(0<<5)  
#define S526_REG_CxM_HARDWARECOUNTENABLE_INDEX		(1<<5)  
#define S526_REG_CxM_HARDWARECOUNTENABLE_INDEXTRANSITION 	(2<<5)  
#define S526_REG_CxM_HARDWARECOUNTENABLE_NOTRCAP 	(3<<5)  
#define S526_REG_CxM_AUTOPRELOAD_DISABLE 			(0)  
#define S526_REG_CxM_AUTOPRELOAD_INDEX_RISE 		(1<<4)  
#define S526_REG_CxM_AUTOPRELOAD_INDEX_FALL 		(1<<3)  
#define S526_REG_CxM_AUTOPRELOAD_RO 				(1<<2)  
#define S526_REG_CxM_COUTPOLARITY_NORMAL 			(0)  
#define S526_REG_CxM_COUTPOLARITY_INVERTED 			(1<<1)  
#define S526_REG_CxM_COUTSOURCE_RCAP 				(0)  
#define S526_REG_CxM_COUTSOURCE_RTGL 				(1)  

#define S526_ADC_SETTLING_DELAY_12US				(1<<15)
#define S526_ADC_SETTLING_DELAY_OFF					(1<<15)
#define S526_ADC_ENABLE_CHANNEL_0					(1<<5)
#define S526_ADC_ENABLE_CHANNEL_1					(1<<6)
#define S526_ADC_ENABLE_CHANNEL_2					(1<<7)
#define S526_ADC_ENABLE_CHANNEL_3					(1<<8)
#define S526_ADC_ENABLE_CHANNEL_4					(1<<9)
#define S526_ADC_ENABLE_CHANNEL_5					(1<<10)
#define S526_ADC_ENABLE_CHANNEL_6					(1<<11)
#define S526_ADC_ENABLE_CHANNEL_7					(1<<12)
#define S526_ADC_ENABLE_REFERENCE_0					(1<<13)
#define S526_ADC_ENABLE_REFERENCE_1					(1<<14)
#define S526_ADC_READ_REFERENCE_1					(0x0012)
#define S526_ADC_READ_REFERENCE_0					(0x0010)
#define S526_ADC_START								(1<<0)


// Prototipos de uso externo:
int sensoray526_init(void);
int sensoray526_close(void);
int sensoray526_set_dio(unsigned char value, unsigned char mask);
unsigned char sensoray526_get_dio(unsigned char mask);
void sensoray526_write_register(unsigned int value16bits, int registeroffset);
unsigned int sensoray526_read_register(int registeroffset);
int sensoray526_led_on(void);
int sensoray526_led_off(void);
int sensoray526_set_counter_preload(unsigned char counternumber, unsigned long value24bits);
int sensoray526_write_PR0(unsigned char counternumber);
int sensoray526_write_PR1(unsigned char counternumber);
int sensoray526_configure_encoder(unsigned char encodernumber);
long sensoray526_read_counter(unsigned char counternumber);
int sensoray526_reset_counter(unsigned char counternumber);
int sensoray526_configure_AD(unsigned int channel);
int sensoray526_perform_AD_conversion(void);
int sensoray526_read_AD_raw(unsigned char channel);
double sensoray526_read_AD_voltage(unsigned char channel);

#ifdef __cplusplus
}
#endif 

#endif // SENSORAY526_H
