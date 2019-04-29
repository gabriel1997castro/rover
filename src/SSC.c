/*-------------------------------------------------------------/
/							SSC			                       /
/--------------------------------------------------------------/
/  Autor : Gabriel Guimarães Almeida de Castro                 /
/  Email: gabriel1997.castro@gmail.com                         /
/  Descrição: Código com funções da SSC32                      /
/-------------------------------------------------------------*/

#define _BSD_SOURCE
// Bibliotecas
#include "serialcom.h"
#include <unistd.h>

// Cabecalho especificos do modulo:
#include "rover/SSC.h"

// Carriege return
const char* CR = "\r\n";

// Envia linha de comando através da serialcom
int sendCommand(const char* data)
{
    SERIALPORTCONFIG serialPortConfig;
    int err;
    char ssc[] = "/dev/ttyS0";

    // Abre a porta serial
    if((err = serialcom_init(&serialPortConfig, 1, ssc, 115200)) != SERIALCOM_SUCCESS)
    {
        return err;
    }
    // Envia o comando caractere por vez
    for (int i=0; i<strlen(data); i++)
    {
	    serialcom_sendbyte(&serialPortConfig, (unsigned char*) &data[i]);
	    usleep(5);
    }
    
    serialcom_sendbyte(&serialPortConfig, (unsigned char*) &CR[0]);
	usleep(5);
    serialcom_sendbyte(&serialPortConfig, (unsigned char*) &CR[1]);
	usleep(5);
    // Fecha a porta serial
    if((err = serialcom_close(&serialPortConfig)) != SERIALCOM_SUCCESS)
    {
        return err;
    }
}
