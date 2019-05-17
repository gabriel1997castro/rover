/*****************************************************************************
*** Arquivo: sensoray526.c
*** Conteudo: modulo exemplo.
*** Autor: G. A. Borges. e F. B. Cavalcanti
*** Atualizacoes: 
	- 18-01-2011: criacao
	- 02-04-2011: contadores/encoders, LEDs, ADs
*****************************************************************************/
/*! \file sensoray526.cpp
* \brief Arquivo exemplo de modulo. */

/*OBS.: O código foi feito pra uasr Xenomai, mas como atualmente não está sendo
usado, foram comentadas todas as linhas referentes a isso.*/

// Cabecalhos des biblioteca padrao C:
#include <stdio.h>
#include <math.h>
#include <unistd.h> /* for libc5 */
#include <sys/io.h> /* for glibc */

//#include <native/task.h>
//#include <native/timer.h>
//#include <native/sem.h>
//#include <native/mutex.h>



// Cabecalhos especificos do modulo:
#include "rover/sensoray526.h"

// Definicoes internas:
#define S526_IOSIZE		0x40  /* 64 bytes */

// #define SENSORAY526_ENABLE_MUTEX	1

// Prototipos internos:

// Variaveis do modulo:
int sensoray526baseaddress = 0x0100;
int sensoray526_current_AD_setting=0x00;
//RT_MUTEX sensoray526_mutex;

/*****************************************************************************
******************************************************************************
** Funcoes de inicializacao e encerramento
******************************************************************************
*****************************************************************************/

/*! \fn int sensoray526_init(void)
* \brief Funcao de inicializacao.
* \param none
* \return If success, 1. Otherwise, 0.
*/
int sensoray526_init(void)
{
	int status=0;
	
	// Inicializa variaveis globais, aloca memoria, etc.
	if(iopl(3)<0){
		printf("\n sensoray526_init: iopl error");
		return 0;
	}
	
	#if (SENSORAY526_ENABLE_MUTEX)
	status = rt_mutex_create(&sensoray526_mutex,"Sensoray526Mutex");
	if (status != 0)
	{
		printf("    Criacao do mutex 'sensoray526_mutex' falhou: ");
		if(status == (-ENOMEM)) printf("-ENOMEM\n");
		if(status == (-EEXIST)) printf("-EEXIST\n");
		if(status == (-EPERM)) printf("-EPERM\n");
		status = rt_mutex_delete(&sensoray526_mutex);
		if (status != 0) {
			printf("    Falha na tentativa de deletar 'sensoray526_mutex'.\n");
		}
		return 0;
	}
	#endif
	
/*	if (!request_region(sensoray526baseaddress, S526_IOSIZE, "s526")) {
		printf("\n sensoray526_init: I/O port conflict");
		return 0;
	}	
*/
	// Guarda configurao.

	// Inicializa threads.

	// Retorna 
	return 1; 
}                      

/*! \fn int sensoray526_close(void)
* \brief Funcao de encerramento
* \param none
* \return If success, 1. Otherwise, 0.
*/

int sensoray526_close(void)
{
	// Procedimentos de encerramento
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_delete(&sensoray526_mutex);
	#endif
	// Retorna 
   	return 1; 
}                      


/*****************************************************************************
******************************************************************************
** Funcoes de interface
******************************************************************************
*****************************************************************************/
int sensoray526_set_dio(unsigned char value, unsigned char mask)
{
	outw_p(((value & mask) & 0xFF) | (1<<10) | (1<<11), sensoray526baseaddress + S526_REG_DIO);
	
	return 1;
}

unsigned char sensoray526_get_dio(unsigned char mask)
{
	return ((inw_p(sensoray526baseaddress + S526_REG_DIO) & mask) & 0xFF);
}

void sensoray526_write_register(unsigned int value16bits, int registeroffset)
{
	outw_p(value16bits, sensoray526baseaddress + registeroffset);
}

unsigned int sensoray526_read_register(int registeroffset)
{
	unsigned int result;
	result=(inw_p(sensoray526baseaddress + registeroffset) & 0xFFFF);
	return result;
}

int sensoray526_led_on(void)
{
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_acquire(&sensoray526_mutex,TM_INFINITE);
	#endif
	
	sensoray526_write_register(0x01, S526_REG_MSC);
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_release(&sensoray526_mutex);
	#endif
	
	return 1;
}

int sensoray526_led_off(void)
{
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_acquire(&sensoray526_mutex,TM_INFINITE);
	#endif

	sensoray526_write_register(0x00, S526_REG_MSC);
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_release(&sensoray526_mutex);
	#endif
	
	return 1;
}

int sensoray526_set_counter_preload(unsigned char counternumber, unsigned long value24bits)
{
	//Coloca o valor inicial no counter. Detalhe - o contador tem 24 bits
	
	//Endereços
	unsigned char counter_offset_low=0;
	unsigned char counter_offset_high=0;
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_acquire(&sensoray526_mutex,TM_INFINITE);
	#endif
	
	//Número do contador, entre 0-3
	switch(counternumber)
	{
		case 0:
			counter_offset_low=S526_REG_C0L;
			counter_offset_high=S526_REG_C0H;
			break;
		case 1:
			counter_offset_low=S526_REG_C1L;
			counter_offset_high=S526_REG_C1H;
			break;
		case 2:
			counter_offset_low=S526_REG_C2L;
			counter_offset_high=S526_REG_C2H;
			break;
		case 3:
			counter_offset_low=S526_REG_C3L;
			counter_offset_high=S526_REG_C3H;
			break;
		default:
			#if (SENSORAY526_ENABLE_MUTEX)
			rt_mutex_release(&sensoray526_mutex);
			#endif
			return -1; //Erro! Out of range
	}
	
	//Low word (16 bits menos significativos)
	sensoray526_write_register((value24bits&0x0000FFFF), counter_offset_low);
	
	//High word (8 bits mais siginificativos)
	sensoray526_write_register(((value24bits&0x00FF0000)>>16), counter_offset_high);
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_release(&sensoray526_mutex);
	#endif
	
	return 1;
}

int sensoray526_write_PR0(unsigned char counternumber)
{
	//Carrega efetivamente os valores dos registradores de preload no PR0

	//Endereços
	unsigned char counter_offset=0;
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_acquire(&sensoray526_mutex,TM_INFINITE);
	#endif
	
	//Número do contador, entre 0-3
	switch(counternumber)
	{
		case 0:
			counter_offset=S526_REG_C0M;
			break;
		case 1:
			counter_offset=S526_REG_C1M;
			break;
		case 2:
			counter_offset=S526_REG_C2M;
			break;
		case 3:
			counter_offset=S526_REG_C3M;
			break;
		default:
			#if (SENSORAY526_ENABLE_MUTEX)
			rt_mutex_release(&sensoray526_mutex);
			#endif
			return -1; //Erro! Out of range
	}
	
	sensoray526_write_register(S526_REG_CxM_PRELOADREGISTER_PR0, counter_offset);

	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_release(&sensoray526_mutex);
	#endif

	return 1;
}

int sensoray526_write_PR1(unsigned char counternumber)
{
	//Carrega efetivamente os valores dos registradores de preload no PR1

	//Endereços
	unsigned char counter_offset=0;
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_acquire(&sensoray526_mutex,TM_INFINITE);
	#endif
	
	//Número do contador, entre 0-3
	switch(counternumber)
	{
		case 0:
			counter_offset=S526_REG_C0M;
			break;
		case 1:
			counter_offset=S526_REG_C1M;
			break;
		case 2:
			counter_offset=S526_REG_C2M;
			break;
		case 3:
			counter_offset=S526_REG_C3M;
			break;
		default:
			#if (SENSORAY526_ENABLE_MUTEX)
			rt_mutex_release(&sensoray526_mutex);
			#endif
			return -1; //Erro! Out of range
	}
	
	sensoray526_write_register(S526_REG_CxM_PRELOADREGISTER_PR1, counter_offset);

	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_release(&sensoray526_mutex);
	#endif

	return 1;
}

int sensoray526_configure_encoder(unsigned char encodernumber)
{
	//Carrega efetivamente os valores dos registradores de preload no PR1

	//Endereços
	unsigned char counter_mode_offset=0;
	unsigned char counter_control_offset=0;
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_acquire(&sensoray526_mutex,TM_INFINITE);
	#endif
	
	//Número do contador, entre 0-3
	switch(encodernumber)
	{
		case 0:
			counter_mode_offset=S526_REG_C0M;
			counter_control_offset=S526_REG_C0C;
			break;
		case 1:
			counter_mode_offset=S526_REG_C1M;
			counter_control_offset=S526_REG_C1C;
			break;
		case 2:
			counter_mode_offset=S526_REG_C2M;
			counter_control_offset=S526_REG_C2C;
			break;
		case 3:
			counter_mode_offset=S526_REG_C3M;
			counter_control_offset=S526_REG_C3C;
			break;
		default:
			#if (SENSORAY526_ENABLE_MUTEX)
			rt_mutex_release(&sensoray526_mutex);
			#endif
			return -1; //Erro! Out of range
	}
	
	//Latch on read, Quadrature, Quadradure x1, counter enabled. Ver Sensoray Model 526 Manual, página 26.
	sensoray526_write_register((S526_REG_CxM_LATCHONREAD|S526_REG_CxM_COUNTDIRECTIONMODE_QUADRATURE|S526_REG_CxM_CLOCKSOURCE_QUAD_X1|S526_REG_CxM_COUNTENABLE_ENABLED|S526_REG_CxM_COUTPOLARITY_NORMAL|S526_REG_CxM_COUTSOURCE_RCAP), counter_mode_offset);

	//Limpa os bits de erro. Ver Sensoray Model 526 Manual, página 27.
	sensoray526_write_register(((1<<3)|(1<<1)|(1<<2)|(1<<0)), counter_control_offset);
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_release(&sensoray526_mutex);
	#endif
	
	return 1;
}

int sensoray526_reset_counter(unsigned char counternumber)
{
	//Endereços
	unsigned char counter_control_offset=0;
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_acquire(&sensoray526_mutex,TM_INFINITE);
	#endif
	
	//Número do contador, entre 0-3
	switch(counternumber)
	{
		case 0:
			counter_control_offset=S526_REG_C0C;
			break;
		case 1:
			counter_control_offset=S526_REG_C1C;
			break;
		case 2:
			counter_control_offset=S526_REG_C2C;
			break;
		case 3:
			counter_control_offset=S526_REG_C3C;
			break;
		default:
			#if (SENSORAY526_ENABLE_MUTEX)
			rt_mutex_release(&sensoray526_mutex);
			#endif
			return -1; //Erro! Out of range
	}
	
	//Reseta o contador. Ver Sensoray Model 526 Manual, página 27.
	sensoray526_write_register(((1<<15)|(1<<3)|(1<<1)|(1<<2)|(1<<0)), counter_control_offset);
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_release(&sensoray526_mutex);
	#endif
	
	return 1;
}

long sensoray526_read_counter(unsigned char counternumber)
{
	//Le o valor do countador/encoder
	
	//Endereços
	unsigned char counter_offset_low=0;
	unsigned char counter_offset_high=0;
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_acquire(&sensoray526_mutex,TM_INFINITE);
	#endif
	
	//Variáveis temporárias
	long low_result=0;
	long high_result=0;
	
	//Número do contador, entre 0-3
	switch(counternumber)
	{
		case 0:
			counter_offset_low=S526_REG_C0L;
			counter_offset_high=S526_REG_C0H;
			break;
		case 1:
			counter_offset_low=S526_REG_C1L;
			counter_offset_high=S526_REG_C1H;
			break;
		case 2:
			counter_offset_low=S526_REG_C2L;
			counter_offset_high=S526_REG_C2H;
			break;
		case 3:
			counter_offset_low=S526_REG_C3L;
			counter_offset_high=S526_REG_C3H;
			break;
		default:
			#if (SENSORAY526_ENABLE_MUTEX)
			rt_mutex_release(&sensoray526_mutex);
			#endif
			return -1; //Erro! Out of range
	}
	
	//Low word (16 bits menos significativos) - DEVE SER LIDO PRIMEIRO (latch on read)
	low_result=((long)sensoray526_read_register(counter_offset_low))&0xFFFF;
	
	//High word (8 bits mais siginificativos)
	high_result=((long)(sensoray526_read_register(counter_offset_high)))&0xFFFF;
	
	//24 bits
	if((high_result>>7)==1) //Último bit é 1? Precisamos verificar para fazer o complemento de 2 da forma correta
	{
		high_result=high_result|0xFF00; //Faz a extensão do último bit
	}
	
	//16 bits
	//if((low_result>>15)==1) //Último bit é 1? Precisamos verificar para fazer o complemento de 2 da forma correta
	//{
	//	high_result=0xFFFF; //Faz a extensão do último bit
	//}
	
	high_result=high_result<<16; //Torna a variável os bits mais significativos.
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_release(&sensoray526_mutex);
	#endif
	
	return (low_result+high_result); //Retorna a leitura do contador (no caso, o encoder).
}

int sensoray526_configure_AD(unsigned int channel)
{
	//Configura o conversor AD. Recebe as flags de enable definidas no
	//arquivo .h utilizando OU (|). Exemplo: habilitar canais 1 e 5
	//sensoray526_configure_AD(S525_ADC_ENABLE_CHANNEL_1|S525_ADC_ENABLE_CHANNEL_5);
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_acquire(&sensoray526_mutex,TM_INFINITE);
	#endif
	
	if((channel&0x7FE0)==0) return -1; //Canais inválidos
	
	//Salva as configuracoes
	sensoray526_current_AD_setting=(S526_ADC_SETTLING_DELAY_12US|(channel&0x7FE0)|S526_ADC_ENABLE_REFERENCE_0|S526_ADC_ENABLE_REFERENCE_1);
	sensoray526_write_register(sensoray526_current_AD_setting,S526_REG_ADC);
	
	//Habilita interupção do AD
	sensoray526_write_register((1<<2),S526_REG_IER);
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_release(&sensoray526_mutex);
	#endif
	
	return 1;
}

int sensoray526_perform_AD_conversion(void)
{
	//Executa a conversão analógica-digital de todos os canais configurados.
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_acquire(&sensoray526_mutex,TM_INFINITE);
	#endif
	
	//Inicia a conversão de todos os canais selecionados
	sensoray526_write_register(sensoray526_current_AD_setting|S526_ADC_START,S526_REG_ADC);
	
	//Espera fim da conversão
	while(!(sensoray526_read_register(S526_REG_ISR)&0x0004));

	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_release(&sensoray526_mutex);
	#endif
	
	return 1;
}

int sensoray526_read_AD_raw(unsigned char channel)
{
	//Faz a leitura do conversor AD, retornando o valor bruto.
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_acquire(&sensoray526_mutex,TM_INFINITE);
	#endif
	
	//Variáveis internas
	int ad_value=0;
	
	//Seleciona para leitura
	sensoray526_write_register(sensoray526_current_AD_setting|(channel<<1),S526_REG_ADC);
	
	ad_value=sensoray526_read_register(S526_REG_ADD);
	
	if((ad_value>>15)==1) //Último bit é 1? Precisamos verificar para fazer o complemento de 2 da forma correta
	{
		ad_value=ad_value|0xFFFF0000; //Faz a extensão do último bit
	}
	
	#if (SENSORAY526_ENABLE_MUTEX)
	rt_mutex_release(&sensoray526_mutex);
	#endif
	
	return ad_value;
}

double sensoray526_read_AD_voltage(unsigned char channel)
{
	//Faz a leitura do conversor AD, e retorna o valor em volts.
	//NOTA: testes iniciais mostraram um erro de 1% para menos na placa.
	//Referência: Fluke 87V, calibrado com padrão NIST.
	
	if((channel<0)|(channel>7)) return 0; //Erro.
	
	return (((double)sensoray526_read_AD_raw(channel))*((10.0)/((double)((1<<15)-1))));
}


/*****************************************************************************
******************************************************************************
** Funcoes internas
******************************************************************************
*****************************************************************************/

