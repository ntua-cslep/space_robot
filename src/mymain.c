/*
** 27/06/13
**
** Read 3 mouse (optical sensors) using non-blocking I/O inside an interrupt.
** Store 2 readings (dx, dy) and timestamp (s, ns) at a circular buffer
**
** Perform PWM for the thrusters inside the ISR
**
** Store MAX data per mouse x and MAX y at an array.
**
** Total runtime is MAX * INTERV (interrupt period)
** MAX and INTERV are defined at mice.h
**
** Compile with:
** gcc mymain1.c dac.o hardware.o circbuffer.o -o mymain1 -lrt -lm
**
** To read we use fmouse = open(MOUSE_DEVICE, O_RDONLY | O_NONBLOCK)
** O_RDONLY : read only
** O_NONBLOCK: non-blocking
** with logical OR (|) so they are both active
**
** WARNING: fmouse returns -1 when there is no movement
**
** fmouse holds the mouse file descriptor.
**
** Define mouse device in mice.h
**
** Remember to turn off mouse acceleration!
**
** Regarding the interrupt code:
** Based on: Orion Lawlor's Short UNIX Examples, olawlor@acm.org 2004/3/5
** http://www.cs.uaf.edu/~olawlor/ref/examples/unix/index.html
** Also read: Understanding the Linux Kernel, p.254.
**
** Original method for reading mice via their char device:
** http://letsmakerobots.com/node/9355
**
*/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <malloc.h>

#ifdef PC104
#include <sys/io.h>
#endif

#include "common.h"
#include "dac.h"
#include "mice.h"
#include "circbuffer.h"

/* Global Variables! */
int fmouse1, fmouse2, fmouse3;  // fmouse = file descriptor for mice
int mouse1[MAX][4];            // array size is MAX, column 0 is xpos, 1 is ypos
int mouse2[MAX][4];           // array size is MAX, column 0 is xpos, 1 is ypos
int mouse3[MAX][4];          // array size is MAX, column 0 is xpos, 1 is ypos

volatile int array_counter=0;      // to check array size < MAX
volatile int duty_0=0, duty_1=0, duty_2=0, duty_3=0, duty_4=0, duty_5=0; // PWM duty cycle: duty_0: thruster 1 etc 

CircularBuffer cb;           // buffer to save mouse data
ElemType elem = { 100,100,100,100,100,100,100,100 }; // struct to save mouse data

/* Returns the difference between two "struct timespec" in seconds: b - a */
double time_diff(struct timespec a, struct timespec b) {
  double diff = fabs(b.tv_sec - a.tv_sec + (b.tv_nsec - a.tv_nsec)/1000000000.0)   ;     //seconds
  return diff;
}

/*
** This is the interrupt handler.
**
** It reads the mouse event data (3 bytes) in the b[] buffer
** and then updates xpos and ypos.
**
** It turns ON/OFF the digital ports for the thruster PWM
**
** Note that when read() returns -1 it is because the mouse did not
** generate any event (i.e. no buttons, no movement) and thus we must
** skip the update of xpos and ypos.
*/

void signalHandler(int cause, siginfo_t *HowCome, void *ucontext) {
  struct timespec tpm;

  static int duty_counter = 0; // Counter used inside the handler to see if we have exceeded the duty cycle
  static int experiment_iteration=0; // # of iterations as defined by the handler to read mice
  
  duty_counter++;

  /* Thruster PWM */

 /* Thruster 1 */

  if (duty_counter <= duty_0) {
    OpenDigital(0,  PORTA);   
//    printf("T1 ON\t");
  }
  else {   
    CloseDigital(0,  PORTA);
//    printf("T1 OFF\t");
  }

 /* Thruster 2 */

  if (duty_counter <= duty_1) {
    OpenDigital(1,  PORTA);   
//    printf("T2 ON\t");
  }
  else {   
    CloseDigital(1,  PORTA);
//    printf("T2 Off\t");
  }

 /* Thruster 3 */

  if (duty_counter <= duty_2) {
    OpenDigital(2,  PORTA);   
//    printf("T3 ON\t");
  }
  else {   
    CloseDigital(2,  PORTA);
//    printf("T3 Off\t");
  }

 /* Thruster 4 */

  if (duty_counter <= duty_3) {
    OpenDigital(3,  PORTA);   
//    printf("T4 ON\t");
  }
  else {   
    CloseDigital(3,  PORTA);
//    printf("T4 Off\t");
  }

 /* Thruster 5 */

  if (duty_counter <= duty_4) {
    OpenDigital(4,  PORTA);   
//    printf("T5 ON\t");
  }
  else {   
    CloseDigital(4,  PORTA);
//    printf("T5 Off\t");
  }

 /* Thruster 6 */

  if (duty_counter <= duty_5) {
    OpenDigital(5,  PORTA);   
//    printf("T6 ON\n");
  }
  else {  
    CloseDigital(5,  PORTA);
//    printf("T6 Off\n");
  }

  if (duty_counter >= PWM_Resolution) {
    duty_counter = 0;
  }
}

void set_timer(int period, int initial) {   // period and initial are in 1/1000000 of a second

  struct itimerval itimer;

  /* Set the interval of the interrupt (i.e. how often it runs).*/
  itimer.it_interval.tv_sec=0;              // Every time: interval seconds
  itimer.it_interval.tv_usec=period;        // interval microseconds (max: one million)

  itimer.it_value.tv_sec=0;                 // Once: begin using the interrupt after
  itimer.it_value.tv_usec=initial;          // this long has passed 

  /*  Actually set the timer.*/
  setitimer(ITIMER_REAL, &itimer, NULL);
}


void init_mice(){
  /* Install our SIGPROF signal handler*/
  struct itimerval itimer;
  struct sigaction sa;
  sa.sa_sigaction = signalHandler;
  sigemptyset( &sa.sa_mask );
  sa.sa_flags = SA_SIGINFO;         // we want a siginfo_t

  /* Request SIGALRM.*/
  if (sigaction (SIGALRM, &sa, 0)) {
    printf("sigaction error\n");
    exit(1);
  }
  
} // end init_mice()


int main(void) {

  int i=0;  
  struct timespec tpnow, tpend;
  double now=0, end=0;

  int testBufferSize = 3; /* buffer size */

  FILE *fp;  
  if((fp =  fopen("optdata", "w")) ==  NULL) {
    printf("Error opening file optdata\n");
    exit(1);
  }
  
  fprintf(fp, "#Mouse data: Mouse 1: Front, Mouse 2: Back Right, Mouse 3: Back Left\n");
  fprintf(fp, "#mx1\tmy1\tmx2\tmy2\tmx3\tmy3\ts\tns\n");  
  
  printf("PWM Frequency is %d HZ\n", PWM_F);
  printf("Resolution is %d\n", PWM_Resolution);
  
   /* Thruster PWM duty cycle (0 -> Thruster 1, etc) */

  duty_0 = 10;
  duty_1 = 20;
  duty_2 = 30;
  duty_3 = 0;
  duty_4 = 0;
  duty_5 = 56;
  
  SetDAC_Board(DAC_BA);
  SendDAC(DAC_BA, 0.0); // ensure that no analogue signal is sent
  WritePORT( 0, PORTA); // ensure all thrusters are OFF

  /* Configure port C for input, all else ouput */
  printf("Configure Ports...\n");
  ConfigureIOPorts(OUTPUT, OUTPUT, INPUT, INPUT); 
  
  printf("Initialize mice...\n");
  cbInit(&cb, testBufferSize);
  init_mice();

  set_timer(INTERV, 10000);    // enable the timer to generate signals
  clock_gettime(CLOCK_REALTIME, &tpnow);
  
  while(array_counter < MAX);     // wait till counter runs out
  
  set_timer(0, 0);          // disable the timer, so no signals are generated
  WritePORT( 0, PORTA);    // ensure all thrusters are OFF
  SendDAC(DAC_BA, 0.0);   // ensure that no analogue signal is sent
  clock_gettime(CLOCK_REALTIME, &tpend);
  
  ReadDigital(0, PORTA); // to check if thruster is off
  ReadDigital(1, PORTA);
  ReadDigital(2, PORTA);
  ReadDigital(3, PORTA);
  ReadDigital(4, PORTA);
  ReadDigital(5, PORTA);
  
  printf("End of experiment. Saving data...\n"); 
  printf("Begin sec %ld  %9ld \t\n ", tpnow.tv_sec, tpnow.tv_nsec);
  printf("End sec %ld  %9ld \t\n ", tpend.tv_sec, tpend.tv_nsec);
  printf("Total time as diff is %lfs\n", time_diff(tpend, tpnow));
  
  /* print results and write data file */
  for (i=0; i<MAX; i++) {
  fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", mouse1[i][0], mouse1[i][1], mouse2[i][0], mouse2[i][1], mouse3[i][0], mouse3[i][1],mouse3[i][2], mouse3[i][3]);
//    printf(      "%d\t %d\t %d\t %d\n", mouse1[i][0], mouse1[i][1], mouse1[i][2], mouse1[i][3]);
//    printf(      "%d\t %d\t %d\t %d\n", mouse2[i][0], mouse2[i][1], mouse2[i][2], mouse2[i][3]);
//    printf(      "%d\t %d\t %d\t %d\n", mouse3[i][0], mouse3[i][1], mouse3[i][2], mouse3[i][3]);
  }

   /* Remove and print all elements */
    while (!cbIsEmpty(&cb)) {
      cbRead(&cb, &elem);
      printf("mouse1x = %d\t, mouse1y = %d\n", elem.mouse1x, elem.mouse1y);
      printf("mouse2x = %d\t, mouse2y = %d\n", elem.mouse2x, elem.mouse2y);
      printf("mouse3x = %d\t, mouse3y = %d\n", elem.mouse3x, elem.mouse3y);
      printf("sec = %d\t, nsec = %d\n", elem.second, elem.nanosecond);
    }

  fclose(fp);

  printf("Program will exit now!\n");

  return(0);

}











