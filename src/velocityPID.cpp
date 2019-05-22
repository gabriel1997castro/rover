/*-----------------------------------------------------------------------------------------------------/
/										     KYLE AND JESSE                                            /
/----------------------------------------------------------------------------------------------------- /
/  Autor : Gabriel Guimarães Almeida de Castro                                                         /
/  Descrição: PID to control velocity of wheels                                                        /
/-----------------------------------------------------------------------------------------------------*/

// Libraries
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
#include <geometry_msgs/Twist.h>
//---------------------------------------------------------------------------------------------------------

// Defining max and min values to SSC pwm
#define SSC_MAX 1800
#define SSC_MIN 1200
//---------------------------------------------------------------------------------------------------------

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
//---------------------------------------------------------------------------------------------------------

// PID variables
typedef struct PID
{
    double previousTime, currentTime;
    double elapsedTime;
    double error;
    double lastError;
    double input, output;
    double cumError, rateError;
    double setPoint;
    //PID constants
    double kp;
    double ki;
    double kd;
} PID_t;
//---------------------------------------------------------------------------------------------------------

// Defining functions to calculate time    
static struct
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

typedef struct
{
    struct timeval time;
    struct timeval timereset;
    
    void start(void) //chrono 0 begin
    {
        gettimeofday(&timereset, NULL);
    }

    double end(void) //chrono 0 end
    {
        gettimeofday(&time, NULL);
        return ((time.tv_sec - timereset.tv_sec) + (time.tv_usec - timereset.tv_usec) * 1e-6);
    }
}my_timer_t;



static struct
{
    struct timeval time;
    struct timeval timereset;
    int firstTime;
}chronoPid;

void chronoPidB(void) //chrono pid begin
{
    gettimeofday(&chronoPid.timereset, NULL);
}

double chronoPidE(void) //chrono pid end
{
    gettimeofday(&chronoPid.time, NULL);
    return ((chronoPid.time.tv_sec - chronoPid.timereset.tv_sec) + (chronoPid.time.tv_usec - chronoPid.timereset.tv_usec) * 1e-6);
}

void firstPidTime(void)
{
    if (chronoPid.firstTime != 1)
    {
        chronoPidB();
    }
}


//---------------------------------------------------------------------------------------------------------


//double pidLeft, pidRight;
rover::WheelVel vel, vel0;
rover::WheelVel pid;
rover::WheelVel inp;
int z_ang = 0;
int x_lin = 0;

void WheelsVelocityCallback(geometry_msgs::Twist vel_msg)
{
    x_lin = vel_msg.linear.x;
    z_ang = vel_msg.angular.z;
}


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
float computePID(double inp, PID_t *ptrPID)
{
    ptrPID->currentTime =  ros::Time::now().toSec(); // tempo atual
    ptrPID->elapsedTime = (double)(ptrPID->currentTime - ptrPID->previousTime);//tempo gasto
        
    ptrPID->error = ptrPID->setPoint - inp; // determine error / proporcional
    ptrPID->cumError += ptrPID->error * ptrPID->elapsedTime; // calcula integral
    ptrPID->rateError = (ptrPID->error - ptrPID->lastError)/ptrPID->elapsedTime; // calcula derivada
 
    double out = ptrPID->kp*ptrPID->error + ptrPID->ki*ptrPID->cumError + ptrPID->kd*ptrPID->rateError; //Saída do PID               
    ptrPID->lastError = ptrPID->error;                                // Guarda erro atual
    ptrPID->previousTime = ptrPID->currentTime;                        //Guarda tempo atual

    int ssc = -((int)(25*out+1500)) + 3000;
    if(ssc >= SSC_MAX || ssc <= SSC_MIN)
    {
        ptrPID->cumError -= ptrPID->error * ptrPID->elapsedTime;
        out = ptrPID->kp*ptrPID->error + ptrPID->ki*ptrPID->cumError + ptrPID->kd*ptrPID->rateError;
    }
 
    return -out;                                        //have function return the PID output
}

float computeLR_PID(PID_t *ptrPID_L, PID_t *ptrPID_R)
{
    pid.left_wheels = computePID(vel.left_wheels,  ptrPID_L );
    pid.right_wheels = computePID(vel.right_wheels, ptrPID_R);
}
//---------------------------------------------------------------------------------------------------------

void computeVel_R()
{
    double texec;
	unsigned char counter = 0;
	long n0 = 0;
	long n1 = 0;
    my_timer_t timer;

    //Calcula para a roda direita:
    sensoray526_configure_encoder(0);
	sensoray526_reset_counter(0);
    timer.start();
	usleep(100);

    texec = timer.end();
    n0 = sensoray526_read_counter(0); //n of pulses encoder 0
    
    vel.right_wheels = (0.0020943952 * n0) / texec; // (2 * pi) / (100 cycles * 30) = const = 0.0020943952
    //Filtro de pico
	if((vel0.right_wheels >= 0.5 || vel0.right_wheels <= -0.5) && (abs(vel.right_wheels) >= abs(5*vel0.right_wheels)))
    {
        vel.right_wheels = vel0.right_wheels;
    }
}

void computeVel_L()
{
    float texec;
	unsigned char counter = 0;
	long n0 = 0;
	long n1 = 0;   
    my_timer_t timer; 

    //Calcula para a roda esquerda:
	sensoray526_configure_encoder(1);
    sensoray526_reset_counter(1);
    timer.start();
	usleep(100);
	n1 = -sensoray526_read_counter(1); //n of pulses encoder 1
    
    texec = timer.end();
    
	vel.left_wheels = (0.0020943952 * n1) / texec;

    
    if((vel0.left_wheels >= 0.5 || vel0.left_wheels <= -0.5) && (abs(vel.left_wheels) >= abs(5*vel0.left_wheels)))
    {
        vel.left_wheels = vel0.left_wheels;
    }
}

// Calcula a velocidade das rodas em rad/s
void computeVel()
{
    vel0 = vel;
    computeVel_L();
    computeVel_R();
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

int main(int argc, char **argv)
{
    PID_t PID_L; //Pid left
    PID_t *ptrPID_L;
    PID_t PID_R; //Pid right
    PID_t *ptrPID_R;
    PID_L.kp = 0.073123;
    PID_L.ki = 2.7045;
    PID_L.kd = 0.00049427;

    PID_R.kp = 0.073123;
    PID_R.ki = 2.7045;
    PID_R.kd = 0.00049427;
    ptrPID_L = &PID_L;
    ptrPID_R = &PID_R;

    geometry_msgs::Twist vel_msg;
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

    PID_L.setPoint = 3;
    PID_R.setPoint = 3;
    int count = 0;
    float tempo = 0;


    // Initiate new ROS node named "vel_pub"
	ros::init(argc, argv, "vel_pub");

    //create a node handle: it is reference assigned to a new node
	ros::NodeHandle n;
    //create a publisher with a topic "wheels_velocity" that will send a String message
	ros::Publisher wheels_velocity_publisher = n.advertise<rover::WheelVel>("wheels_velocity", 10);

    //ros::Publisher pid_velocity_publisher = n.advertise<rover::WheelVel>("pid_velocity", 10);
    ros::Publisher inp_velocity_publisher = n.advertise<rover::WheelVel>("inp_velocity", 10);
	//Rate is a class the is used to define frequency for a loop. Here we send a message each two seconds.
	ros::Rate loop_rate(10); //10 messages per second
    ros::Subscriber sub = n.subscribe("/turtle1/cmd_vel", 10, WheelsVelocityCallback);

    sendCommand(command.c_str());
    inp.left_wheels = 0;
    inp.right_wheels = 0;

    double temp;

    while(ros::ok())
    {
  
        computeVel();
        if(x_lin != 0)
        {
            PID_L.setPoint = 1.5*x_lin;
            PID_R.setPoint = 1.5*x_lin;
        }
        else
        {
            PID_L.setPoint = 0;
            PID_R.setPoint = 0; 
        }
        if(z_ang != 0)
        {
            PID_L.setPoint = 1.5*z_ang ;
            PID_R.setPoint = -1.5*z_ang;
        }
        else
        {
            PID_L.setPoint = 0;
            PID_R.setPoint = 0; 
        }
        //Publish the message
        wheels_velocity_publisher.publish(vel);

        computeLR_PID(ptrPID_L, ptrPID_R);
        
        command = "#8P" + PIDToSSC(pid.right_wheels) + " #9P" + PIDToSSC(pid.left_wheels);
        std::cout << command << std::endl;
        sendCommand(command.c_str());
        tempo += 0.1;
        if(tempo >= 0.6)
        {
            
            count = 0;
            tempo = 0;
            inp.left_wheels = PID_L.setPoint;
            inp.right_wheels = PID_R.setPoint;
        }

        inp_velocity_publisher.publish(inp);
        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages

        loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
    }
    

    fflush(stdout); // mostra todos printfs pendentes.
    return 0;
}
