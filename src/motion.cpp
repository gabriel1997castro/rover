/*-----------------------------------------------------------------------------------------------------/
/										                                                 /
/----------------------------------------------------------------------------------------------------- /
/  Autor : Gabriel Guimarães Almeida de Castro                                                         /
/  Descrição:                                                                              /
/-----------------------------------------------------------------------------------------------------*/

// Bibliotecas
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <cstdlib>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/io.h>
#include <sys/time.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <iostream>
#include <sys/select.h>
#include "rover/sensoray526.h"
#include "rover/SSC.h"


// Definicoes internas:
#define MAIN_MODULE_INIT(cmd_init)           \
	if (cmd_init == 0)                       \
	{                                        \
		printf("    Erro em %s", #cmd_init); \
		return (0);                          \
	}
#define MAIN_MODULE_CLOSE(cmd_close)          \
	if (cmd_close == 0)                       \
	{                                         \
		printf("    Erro em %s", #cmd_close); \
	}

// Definições para usar tic e toc pra cálculos de tempo
//---------------------------------------------------------------------------------------------------------
struct
{
	struct timeval time;
	struct timeval timereset;
} tictocctrl;

void tic(void)
{
	gettimeofday(&tictocctrl.timereset, NULL);
}

double toc(void)
{
	gettimeofday(&tictocctrl.time, NULL);
	return ((tictocctrl.time.tv_sec - tictocctrl.timereset.tv_sec) + (tictocctrl.time.tv_usec - tictocctrl.timereset.tv_usec) * 1e-6);
}
//---------------------------------------------------------------------------------------------------------

void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";

    printf("\n*** Encerrando o modulo sensoray526...");
    MAIN_MODULE_CLOSE(sensoray526_close());
    printf("\n\n");
    fflush(stdout); // mostra todos printfs pendentes.

    exit(signum);
}

//Lê os encoder por mais ou menos 1s
//---------------------------------------------------------------------------------------------------------
int readEncoder(void)
{
	float texec;
	unsigned char counter = 0;
	//Configura encoder 0
	sensoray526_configure_encoder(0);
	sensoray526_configure_encoder(1);
	sensoray526_reset_counter(0);
	sensoray526_reset_counter(1);
	tic();
	for (counter = 0; counter < 10; counter++)
	{
		//Le o encoder 0
		printf("\n Encoder 0: (%ld)", sensoray526_read_counter(0));
		printf("\n Encoder 1: (%ld)", sensoray526_read_counter(1));

		// Sleep
		usleep(100000);
	}
	texec = toc(); //us
	printf("\nRead encoder took %f seconds to execute \n", texec * 1e6);
	return 1;
}
//---------------------------------------------------------------------------------------------------------

// Calcula a velocidade das rodas em rad/s
void computeVel(void)
{
	float texec;
	unsigned char counter = 0;
	long n0 = 0;
	long n1 = 0;
	double w0, w1;

	sensoray526_configure_encoder(0);
	sensoray526_configure_encoder(1);

	while (true)
	{
		sensoray526_reset_counter(0);
		sensoray526_reset_counter(1);
		tic();

		// Sleep
		usleep(100000);

		n0 = sensoray526_read_counter(0); //n of pulses encoder 0
		n1 = sensoray526_read_counter(1); //n of pulses encoder 1
		texec = toc();
		w0 = (0.0020943952 * n0) / texec; // (2 * pi) / (100 cycles * 30) = const = 0.0020943952
        w1 = (0.0020943952 * n1) / texec;
		printf("\n Speed1: %f rad/s           Speed2: %f rad/s", w0, w1);
	}
}
//---------------------------------------------------------------------------------------------------------

int main()
{
	int mode = 0;
	std::string command;

	// Cadastra funções para encerrar o programa
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

	//run
	command = "#8 P1700 #9 P1700";
	sendCommand(command.c_str());

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT | MCL_FUTURE);

	// robot module:
	printf("\n*** Iniciando o modulo sensoray526...");
	MAIN_MODULE_INIT(sensoray526_init());

	computeVel();


	command = "#8 P1500 #9 P1500";
	sendCommand(command.c_str());
	sleep(1);
	computeVel();
	printf("\n*** Encerrando o modulo robot...");
	MAIN_MODULE_CLOSE(sensoray526_close());

	printf("\n\n");
	fflush(stdout); // mostra todos printfs pendentes.
	return 1;
}
