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
//#include <signal.h>
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
#include "ros/ros.h"
#include "rover/WheelVel.h"

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
//
//---------------------------------------------------------------------------------------------------------

//Lê os encoder por mais ou menos 1s
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
void computeVel(int argc,char **argv)
{
   
	float texec;
	unsigned char counter = 0;
	long n0 = 0;
	long n1 = 0;

    // Initiate new ROS node named "vel_pub"
	ros::init(argc, argv, "vel_pub");

    //create a node handle: it is reference assigned to a new node
	ros::NodeHandle n;
    //create a publisher with a topic "wheels_velocity" that will send a String message
	ros::Publisher wheels_velocity_publisher = n.advertise<rover::WheelVel>("wheels_velocity_pub", 10);
	//Rate is a class the is used to define frequency for a loop. Here we send a message each two seconds.
	ros::Rate loop_rate(10); //1 message per second

	sensoray526_configure_encoder(0);
	sensoray526_configure_encoder(1);
    rover::WheelVel vel, vel0;

	while (ros::ok())
	{
         vel0 = vel;
        //create a wheelVel message
		sensoray526_reset_counter(0);
		sensoray526_reset_counter(1);
		tic();

		// Sleep
		usleep(5000);

		n0 = sensoray526_read_counter(0); //n of pulses encoder 0
		n1 = -sensoray526_read_counter(1); //n of pulses encoder 1
		texec = toc();

		vel.right_wheels = (0.0020943952 * n0) / texec; // (2 * pi) / (100 cycles * 30) = const = 0.0020943952
        vel.left_wheels = (0.0020943952 * n1) / texec;

        if((vel0.left_wheels >= 0.5 || vel0.left_wheels <= -0.5) && (abs(vel.left_wheels) >= abs(5*vel0.left_wheels)))
        {
            printf("Pico2!!!\n");
            vel.left_wheels = vel0.left_wheels;
        }   
        if((vel0.right_wheels >= 0.5 || vel0.right_wheels <= -0.5) && (abs(vel.right_wheels) >= abs(5*vel0.right_wheels)))
        {
            printf("Pico1!!!\n");
            vel.right_wheels = vel0.right_wheels;
        }

        //Publish the message
        wheels_velocity_publisher.publish(vel);

        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages

        loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate

	}
}

//---------------------------------------------------------------------------------------------------------

//Conversões de intervalos [-1, 1] e [500 2500]
float PIDToSSC(float value)
{
    return (value*1000 + 1500);
}
float SSCtoPID(float value)
{
    return (0.001*value + 1.5);
}

//---------------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // Cadastra funções para encerrar o programa
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Avoids memory swapping for this program
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // robot module:
    printf("\n*** Iniciando o modulo sensoray526...");
    MAIN_MODULE_INIT(sensoray526_init());
    
    computeVel(argc, argv);

    fflush(stdout); // mostra todos printfs pendentes.
    return 0;
}
