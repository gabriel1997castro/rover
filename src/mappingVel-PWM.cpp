/*-----------------------------------------------------------------------------------------------------/
/										                                                 /
/----------------------------------------------------------------------------------------------------- /
/  Autor : Gabriel Guimarães Almeida de Castro                                                         /
/  Descrição: Experimento para achar banda morta                                                                             /
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


 

//double pidLeft, pidRight;
rover::WheelVel vel;
rover::WheelVel pwm;
rover::WheelVel inp;
rover::WheelVel vel0;
int ssc_global;

#define SSC_MAX 2200
#define SSC_MIN 800

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
    command = "#8 P1500 #9 P1500";
	sendCommand(command.c_str());
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    printf("\n*** Encerrando o modulo sensoray526...");
    MAIN_MODULE_CLOSE(sensoray526_close());
    printf("\n\n");
    fflush(stdout); // mostra todos printfs pendentes.

    exit(signum);
}
//---------------------------------------------------------------------------------------------------------

// Calcula a velocidade das rodas em rad/s
void computeVel()
{
	vel0 = vel;
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
	usleep(9000);

	n0 = sensoray526_read_counter(0); //n of pulses encoder 0
	n1 = -sensoray526_read_counter(1); //n of pulses encoder 1
	texec = toc();
	vel.right_wheels = (0.0020943952 * n0) / texec; // (2 * pi) / (100 cycles * 30) = const = 0.0020943952
    vel.left_wheels = (0.0020943952 * n1) / texec;
    
    //Filtro de pico
	if((vel0.right_wheels >= 0.5 || vel0.right_wheels <= -0.5) && (abs(vel.right_wheels) >= abs(4*vel0.right_wheels)))
    {
        printf("Pico1!!!\n");
        vel.right_wheels = vel0.right_wheels;
    }
    if((vel0.left_wheels >= 0.5 || vel0.left_wheels <= -0.5) && (abs(vel.left_wheels) >= abs(4*vel0.left_wheels)))
    {
        printf("Pico2!!!\n");
        vel.left_wheels = vel0.left_wheels;
    }
    
}

//---------------------------------------------------------------------------------------------------------


//Conversões de intervalos [-1, 1] e [500 2500]
std::string PIDToSSC(float value)
{
    int ssc = (int)(25*value+1478);
    if (ssc > SSC_MAX)
    {
        ssc = SSC_MAX;
    }
    if (ssc < SSC_MIN)
    {
         ssc = SSC_MIN;
    }
    //invert scale
    ssc = -ssc + 3000;
    ssc_global = ssc;
    return ToString(ssc);
}
//---------------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    std::string command, leftW, rightW;
    // Cadastra funções para encerrar o programa
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Avoids memory swapping for this program
    mlockall(MCL_CURRENT | MCL_FUTURE);
    
    ros::Time::init();
    // robot module:
    printf("\n*** Iniciando o modulo sensoray526...");
    MAIN_MODULE_INIT(sensoray526_init());
    command = "#8 P1500 #9 P1500";

    int count = 0;
    float tempo;


    // Initiate new ROS node named "vel_pub"
	ros::init(argc, argv, "vel_pub");

    //create a node handle: it is reference assigned to a new node
	ros::NodeHandle n;
    //create a publisher with a topic "wheels_velocity" that will send a String message
	ros::Publisher wheels_velocity_publisher = n.advertise<rover::WheelVel>("wheels_velocity", 10);

    //ros::Publisher pid_velocity_publisher = n.advertise<rover::WheelVel>("pid_velocity", 10);
    ros::Publisher inp_velocity_publisher = n.advertise<rover::WheelVel>("inp_pwm", 10);
	//Rate is a class the is used to define frequency for a loop. Here we send a message each two seconds.
	ros::Rate loop_rate(10); //1 message per second

    sendCommand(command.c_str());
    inp.left_wheels = 0;
    inp.right_wheels = 0;
    inp_velocity_publisher.publish(inp);
    float speed = 0.0;
    int k = 1, i = 0;
    while(ros::ok())
    {   
        i++;
        if(vel.left_wheels >= 15.0 || vel.left_wheels <= -15.0)
        {
            if(i > 5)
            {
                 k *= -1;
                 i = 0;
            }
        }
        if (k > 0)
        {
            speed += 0.125;
        }
        if (k < 0)
        {
            speed -=0.125;
        }
        
        inp.left_wheels = speed;
        inp.right_wheels = speed;
        leftW = PIDToSSC(inp.right_wheels);
        pwm.left_wheels = ssc_global;
        rightW = PIDToSSC(inp.left_wheels);
        pwm.right_wheels = ssc_global;
        command = "#8P" + leftW + " #9P" + rightW;
        inp_velocity_publisher.publish(pwm);
        std::cout << command << std::endl;
        sendCommand(command.c_str());
        computeVel();        
        wheels_velocity_publisher.publish(vel);
        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages

        loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
    }
    

    fflush(stdout); // mostra todos printfs pendentes.
    return 0;
}

