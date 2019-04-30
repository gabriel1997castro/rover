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
#include "rover/SSC.h"

#include <rosbag/bag.h>
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

rosbag::Bag bag;

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
    bag.close();
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

    //Abre rosbag em modo de escrita para guardar log
    bag.open("test.bag", rosbag::bagmode::Write);

    // Initiate new ROS node named "vel_pub"
	ros::init(argc, argv, "vel_pub");

    //create a node handle: it is reference assigned to a new node
	ros::NodeHandle n;
    //create a publisher with a topic "wheels_velocity" that will send a String message
	ros::Publisher wheels_velocity_publisher = n.advertise<rover::WheelVel>("wheels_velocity", 10);
	//Rate is a class the is used to define frequency for a loop. Here we send a message each two seconds.
	ros::Rate loop_rate(1); //1 message per second

	sensoray526_configure_encoder(0);
	sensoray526_configure_encoder(1);

	while (ros::ok())
	{
        //create a wheelVel message
        rover::WheelVel vel;
		sensoray526_reset_counter(0);
		sensoray526_reset_counter(1);
		tic();

		// Sleep
		usleep(100000);

		n0 = sensoray526_read_counter(0); //n of pulses encoder 0
		n1 = sensoray526_read_counter(1); //n of pulses encoder 1
		texec = toc();
		vel.left_wheels = (0.0020943952 * n0) / texec; // (2 * pi) / (100 cycles * 30) = const = 0.0020943952
        vel.right_wheels = (0.0020943952 * n1) / texec;
        //Publish the message
        wheels_velocity_publisher.publish(vel);

        bag.write("velocities", ros::Time::now(), vel);
        //bag.write("right_wheel", ros::Time::now(), vel.right_wheels);

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


/*
// Calcula a velocidade das rodas em rad/s
void computeVel(void)
{
    float texec;
    unsigned char counter = 0;
    long n0 = 0;
    long n1 = 0;
    double w0, w1;
    double diff = 0.0;
    char logstr[80];
    time_t now;
    struct tm *ptm;
    time(&now);
    ptm = gmtime(&now);
    std::stringstream ss;
    sprintf(logstr, "../log/logFile_%d-0%d-%d_%d:%d:%d.csv", ptm->tm_year+1900, ptm->tm_mon+1, ptm->tm_mday, ptm->tm_hour-3, ptm->tm_min, ptm->tm_sec);

    logFile = fopen(logstr, "w+");
    fprintf(logFile, "Tempo(s), Ciclos_Enc1, Ciclos_Enc 2, Velocidade_Enc 1(rad/s), Velocidade_Enc 2(rad/s)\n");
    sensoray526_configure_encoder(0);
    sensoray526_configure_encoder(1);
    while (true)
    {
	printf("asdasdafsdfasdasdfasdfasdfasdfasdfasdfasd\n");
        sensoray526_reset_counter(0);
        sensoray526_reset_counter(1);
        tic();
        usleep(100000);
        n0 = sensoray526_read_counter(0); //n de ciclos encoder 0
        n1 = sensoray526_read_counter(1); //n de ciclos encoder 1
        texec = toc();
        diff += texec;

        //w = (2*pi*num_of_cilcos)/(resolução_do_encoder* redução_do_motor*tempo)
        w0 = (0.0020943952 * n0) / texec; // (2 * pi) / (100 cycles * 30) = const = 0.0020943952
        w1 = (0.0020943952 * n1) / texec;
        fprintf(logFile, "%lf, %ld, %ld, %lf, %lf\n", diff, n0, n1, w0, w1);
	prinf("cccccccccccccccccccccccccccccc\n");
    }
}

*/
