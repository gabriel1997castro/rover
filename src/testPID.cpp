/*-----------------------------------------------------------------------------------------------------/
/										     KYLE AND JESSE                                            /
/----------------------------------------------------------------------------------------------------- /
/  Autor : Gabriel Guimarães Almeida de Castro                                                         /
/  Descrição: PID to control velocity of wheels                                                        /
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
double kp = 0.073123;
double ki = 2.7045;
double kd = 0.00049427;
//double kw = 2;
//double kp = 1;
//double ki = 0;
//double kd = 0;

 
double currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint = 0;
double cumError, rateError;
//double pidLeft, pidRight;
//rover::WheelVel vel, vel0;
rover::WheelVel pid;
rover::WheelVel inp;

#define SSC_MAX 1750
#define SSC_MIN 1250

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

    int ssc = (int)(25*out+1468);
    if(ssc >= SSC_MAX || ssc <= SSC_MIN)
    {
        cumError -= error * elapsedTime;
        out = kp*error + ki*cumError + kd*rateError;
    }
    //int ssc = (int)(25*value+1500);
    /*int ssc = -((int)(25*out+1500)) + 3000;
    if(ssc >= SSC_MAX)
    {
        cumError = SSC_MAX;
        out = kp*error + ki*cumError + kd*rateError;
    }
    if(ssc <= SSC_MIN)
    {
        cumError = SSC_MIN;
        out = kp*error + ki*cumError + kd*rateError;
    }*/
 
    return -out;                                        //have function return the PID output
}

float computeLR_PID(const rover::WheelVel vel)
{
    pid.left_wheels = computePID(vel.left_wheels);
    pid.right_wheels = computePID(vel.right_wheels);
}
//---------------------------------------------------------------------------------------------------------


//Conversões de intervalos [-1, 1] e [500 2500]
std::string PIDToSSC(float value)
{
    int ssc = (int)(25*value + 1468);
    if (ssc >= SSC_MAX)
    {
        ssc = SSC_MAX;
    }
    if (ssc <= SSC_MIN)
    {
         ssc = SSC_MIN;
    }

    return ToString(ssc);
    //Retira banda morta
    /*if(ssc >= 1468)
    {
        ssc = ssc + 40;
    }
    else
    {
        ssc = ssc - 40;
    }
    return ToString(ssc);*/
}

//Conversões de intervalos [-1, 1] e [500 2500]
/*std::string PIDToSSC_R(float value)
{
    int ssc = (int)(25*value + 1468);
    if (ssc >= SSC_MAX)
    {
        ssc = SSC_MAX;
    }
    if (ssc <= SSC_MIN)
    {
         ssc = SSC_MIN;
    }

    //Retira banda morta
    if(ssc >= 1468)
    {
        ssc = ssc + 40;
    }
    else
    {
        ssc = ssc - 40;
    }
    return ToString(ssc);
}*/
//---------------------------------------------------------------------------------------------------------

void WheelsVelocityCallback(const rover::WheelVel vel)
{
    ROS_INFO("[Listener] I heard: [%f]\n", vel.left_wheels);
    ROS_INFO("[Listener] I heard: [%f]\n", vel.right_wheels);
    computeLR_PID(vel);
}



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
    command = "#8 P1500 #9 P1500";

    setPoint = 4;
    int count = 0;
    float tempo = 0;


    ros::init(argc, argv, "listener_vel");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("wheels_velocity", 10, WheelsVelocityCallback);
	ros::Rate loop_rate(10); //10 messages per second

    sendCommand(command.c_str());
    inp.left_wheels = 0;
    inp.right_wheels = 0;
    
    while(ros::ok())
    {

        //computeLR_PID();
        
        command = "#8P" + PIDToSSC(pid.right_wheels) + " #9P" + PIDToSSC(pid.left_wheels);
        std::cout << command << std::endl;
        sendCommand(command.c_str());
        tempo += 0.1;
        tic();
        if(tempo >= 5)
        {
            setPoint *=-1;
            count = 0;
            tempo = 0;
            inp.left_wheels = setPoint;
            inp.right_wheels = setPoint;
        }
        
        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages

        loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
    }
    

    fflush(stdout); // mostra todos printfs pendentes.
    return 0;
}
