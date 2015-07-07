#include <ros/ros.h>
#include <sys/time.h>
#include <signal.h>
#include <sys/resource.h>
#include "dm6604.h"

DM6604Device dac; //0x280 
int count=1;
char port = 0;
double duty[8];

int freq; //hz
int res; //second



void signalHandler(int cause, siginfo_t *HowCome, void *ucontext) 
{
    for(char i=0; i<8; i++)
    {
        if(count < duty[i]*(double)res) 
            dac.setDigitalPin(PORTA, i);//port |= 1<<i;//portA
        else 
            dac.clearDigitalPin(PORTA, i);//port &= ~(1<<i);//portA
    }
    count++;
    if(count == res) count = 1;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "thruster_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    dac.init(640);
    dac.configureIOPorts(OUTPUT,OUTPUT,OUTPUT);
    dac.setDACRange(20, BIPOLAR);

    ros::param::param<int>("~frequency", freq, 10);
    ros::param::param<int>("~resolution", res, 100);


    /* Install our SIGPROF signal handler*/
    struct itimerval itimer;
    struct sigaction sa;
    sa.sa_sigaction = signalHandler;
    sigemptyset( &sa.sa_mask );
    sa.sa_flags = SA_SIGINFO;         // we want a siginfo_t

    /* Request SIGALRM.*/
    if (sigaction (SIGALRM, &sa, 0)) 
    {
        printf("sigaction error\n");
        exit(1);
    }

    itimer.it_interval.tv_sec=(unsigned int)(1/(freq*res));              // Every time: interval seconds
    itimer.it_interval.tv_usec=(unsigned int)(1000000/(freq*res));        // interval microseconds (max: one million)
    
    itimer.it_value.tv_sec=(unsigned int)(1/(freq*res));                 // Once: begin using the interrupt after
    itimer.it_value.tv_usec=(unsigned int)(1000000/(freq*res));          // this long has passed 
    
    setitimer(ITIMER_REAL, &itimer, NULL);  


    while(ros::ok())
    {
        duty[0] = 0;
        duty[1] = 0;
        duty[2] = 0;
        duty[3] = 0;
        duty[4] = 0;
        duty[5] = 0;
        duty[6] = 0.95;
        duty[7] = 0.01;
        
        dac.updateDAC(1, 0.0);
        dac.updateDAC(2, 0.0);
        dac.updateDAC(3, 0.0);
        dac.updateDAC(4, 0.0);
        dac.updateDAC(5, 0.5);
        dac.updateDAC(6, 0.0);
        dac.updateDAC(7, 0.0);
        dac.updateDAC(8, 0.0);

        loop_rate.sleep();
    }

    for (int i; i<8; i++) duty[i]=0;
    //dac.writeDigitalPort(PORTA, 0); //shut off all thrusters

    for (int i=1; i<=8; i++) dac.updateDAC(i, 0.0); //shut off all motors

    return 0;
}