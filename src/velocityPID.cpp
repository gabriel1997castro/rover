/*-----------------------------------------------------------------------------------------------------/
/										                                                 /
/----------------------------------------------------------------------------------------------------- /
/  Autor : Gabriel Guimarães Almeida de Castro                                                         /
/  Descrição:                                                                              /
/-----------------------------------------------------------------------------------------------------*/

// Bibliotecas
#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
#include <string.h>
#include <sstream>
#include <math.h>
#include <ctime>
#include <errno.h>
#include <stdlib.h>
#include <csignal>
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
#include "ros/ros.h"
#include "rover/WheelVel.h"
#include "rover/SSC.h"
#include <sstream>
#include <iostream>

//PID constants
double kp = 1;
double ki = 1;
double kd = 0;
 
double currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint = 0;
double cumError, rateError;
double pidLeft, pidRight;
rover::WheelVel vel;

#define SSC_MAX 2000
#define SSC_MIN 1000

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

template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

void signalHandler(int signum)
{   
    std::string command;
    command = "#0 P1500 #1 P1500";
	sendCommand(command.c_str());
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    printf("\n*** Encerrando o modulo sensoray526...");
    MAIN_MODULE_CLOSE(sensoray526_close());
    printf("\n\n");
    fflush(stdout); // mostra todos printfs pendentes.

    exit(signum);
}
//---------------------------------------------------------------------------------------------------------


//
float computePID(double inp)
{
    currentTime =  ros::Time::now().toSec(); // tempo atual
    elapsedTime = (double)(currentTime - previousTime);//tempo gasto
        
    error = setPoint - inp; // determine error / proporcional
    cumError += error * elapsedTime; // calcula integral
    rateError = (error - lastError)/elapsedTime; // calcula derivada
 
    double out = kp*error + ki*cumError + kd*rateError; //Saída do PID               
 
    lastError = error;                                // Guarda erro atual
    previousTime = currentTime;                        //Guarda tempo atual
 
    return out;                                        //have function return the PID output
}

float computeLR_PID()
{
    pidLeft = computePID(vel.left_wheels);
    pidRight = computePID(vel.right_wheels);
}
//---------------------------------------------------------------------------------------------------------

// Calcula a velocidade das rodas em rad/s
void computeVel()
{
	float texec;
	unsigned char counter = 0;
	long n0 = 0;
	long n1 = 0;

	sensoray526_configure_encoder(0);
	sensoray526_configure_encoder(1);

	sensoray526_reset_counter(0);
	sensoray526_reset_counter(1);
	tic();

	// Sleep
	usleep(100000);

	n0 = sensoray526_read_counter(0); //n of pulses encoder 0
	n1 = -sensoray526_read_counter(1); //n of pulses encoder 1
	texec = toc();
	vel.right_wheels = (0.0020943952 * n0) / texec; // (2 * pi) / (100 cycles * 30) = const = 0.0020943952
    vel.left_wheels = (0.0020943952 * n1) / texec;
}

//---------------------------------------------------------------------------------------------------------

int invEscale(int value)
{
    return -value +3000;
}
//Conversões de intervalos [-1, 1] e [500 2500]
std::string PIDToSSC(float value)
{
    int ssc = (int)(25*value+1500);
    if (ssc > SSC_MAX)
    {
        ssc = SSC_MAX;
    }
    if (ssc < SSC_MIN)
    {
         ssc = SSC_MIN;
    }
    return ToString(invEscale(ssc));
}
//---------------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    std::string command;
    // Cadastra funções para encerrar o programa
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Avoids memory swapping for this program
    mlockall(MCL_CURRENT | MCL_FUTURE);
    
    ros::Time::init();
    // robot module:
    printf("\n*** Iniciando o modulo sensoray526...");
    MAIN_MODULE_INIT(sensoray526_init());
    command = "#0 P1500 #1 P1500";

    setPoint = 2;
    int count = 0;
    float tempo;
    while(true)
    {
        if(count == 0)
        {
            tic();
            count++;
        }
        sendCommand(command.c_str());
        computeVel();
        computeLR_PID();
        command = "#0P" + PIDToSSC(pidRight) + " #1P" + PIDToSSC(pidLeft);
        std::cout << command << std::endl;
        sendCommand(command.c_str());
        tempo += toc();
        tic();
        if(tempo >= 5)
        {
            setPoint *= -1;
            count = 0;
            tempo = 0;
        }
    }
    

    fflush(stdout); // mostra todos printfs pendentes.
    return 0;
}
