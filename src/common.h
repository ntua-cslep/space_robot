/* common.h 
** Version 27.06.13 */

/*******************************************************/
/*******************************************************/
/* DEFINE THE FOLLOWING LINE IF RUNNING ON PC104! */
/*******************************************************/
/*******************************************************/

// #define PC104 

/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/
/*******************************************************/


/* Define the following to print in/out operations */
//#define DEBUG


/* Common definitions */

#define TRUE 1
#define FALSE 0

#define INPUT  1
#define OUTPUT  0


typedef int BOOLEAN;


/* Data type BYTE is just an unsigned char, values 0-255 */

typedef unsigned char BYTE;


/* Data type BITPOS can take values D0-D15 and indicates bit position.
** In practice, D0 is a "label" which equals zero, D1 is 1 etc.
*/

typedef enum {D0,D1,D2,D3,D4,D5,D6,D7,D8,D9,D10,D11,D12,D13,D14,D15} BITPOS;


/* Data type MEMPORT indicates a port address */

typedef unsigned int MEMPORT;


/* Hardware parameters */

#define DAC_BA 640      /* DAC base address */
#define ENC1_BA 512     /* Base address of 1st encoder */
#define ENC2_BA 768     /* Base address of 2nd encoder */

/* DAC Card Ports A, B, C for digital I/O */

#define PORTA 640
#define PORTB 641
#define PORTC 642
#define PORT_CTRL 643

/* Encoder Card 1, BA=512, is dedicated to the left arm */
 
#define LARM_MOTOR1 512
#define LARM_MOTOR2 516
#define REACTION_WHEEL 520 
 
/* Encoder Card 2, BA=768, is dedicated to the right arm and the reaction wheel */
 
#define RARM_MOTOR1 768
#define RARM_MOTOR2 772
#define FREE_ENC 776 /* encoder not connected to any motor */

/* Software parameters */

/* PWM Definitions */
#define PWM_F 7               // PWM frequency (HZ)
#define PWM_T (1/PWM_F)       // 1/7 = 0.142857143s
#define PWM_Resolution 100    // PWM resolution

/* Definitions for the signal and its ISR (service routine) */

#define MAX 5000             // array size
                             // Programm ends when array has exceeded array size
 
#define INTERV 1429          // interrupt period in microseconds (max 1 MILLION)
                             // change the number if different period is needed
                             // Total runtime is: MAX * INTERV
                             // 0.001428571 seconds =1/7/100 (100 is PWM resolution, PWM @ 7HZ) 
#define LOG_INTERVAL 10000
#define CONTROLLER_INTERVAL 1000
                             



/*******************************************************/
/*******************************************************/
/*                FUNCTION PROTOTYPES                  */
/*******************************************************/
/*******************************************************/

/* Function prototypes for dac.c */

void SetDAC_Board(MEMPORT address);
void UpdateDAC(unsigned int DAC, const double Volts);
void SendDAC(MEMPORT address, const double volts);
void ConfigureIOPorts(unsigned int PortA, unsigned int PortB,
		      unsigned int PortC_UP, unsigned int PortC_LOW);
void OpenDigital(BITPOS b, MEMPORT address);
void CloseDigital(BITPOS b, MEMPORT address);
unsigned int ReadDigital(BITPOS b, MEMPORT address);
BYTE ReadPORT(MEMPORT address);
void WritePORT( BYTE v, MEMPORT address);

/* Function prototypes for encoder.c */

typedef struct {
  int counts;
  double angle;
} arm_type;

void EncoderEnable(MEMPORT address);
void EncoderLoad(MEMPORT address, unsigned int initialValue);
void EncoderSelectRegister(MEMPORT address, unsigned char Select);
void EncoderHold(MEMPORT address);
unsigned int EncoderRead(MEMPORT address);
void EncoderClearChip(MEMPORT address);
void EncoderInitialize(MEMPORT address1, MEMPORT address2, MEMPORT address3);
arm_type ArmAngle1(MEMPORT address);
arm_type ArmAngle2(MEMPORT address);
arm_type WheelAngle(MEMPORT address);

/* Function prototypes for hardware.c */

#ifndef PC104

BYTE inb(MEMPORT m);
void outb(BYTE b, MEMPORT m);

#endif

void outbyte(BYTE b, MEMPORT m); // outb
BYTE inbyte(MEMPORT m);// inb 
void outword(unsigned int v, MEMPORT m); // outw
unsigned int inword(MEMPORT m); // inw

BOOLEAN bitread(BITPOS b, MEMPORT m);
BYTE bitset(BITPOS b, MEMPORT m);
BYTE bitclear(BITPOS b, MEMPORT m);


/* Function prototypes for profile.c */

double profile_tf(const double Amax, const double Vmax, const double Smax);
double profile_t1(const double Amax, const double Vmax, const double Smax);
double profile_t2(const double Amax, const double Vmax, const double Smax);
double profile_disp(double t, const double Amax, const double Vmax, const double Smax);
double profile_velocity(double t, const double Amax, const double Vmax, const double Smax);
double profile_acc(double t, const double Amax, const double Vmax, const double Smax);

/* Function prototypes for mice localization (optloc.c) */

/* Data types */

typedef struct {
  double x,y;
} POINT;

typedef struct {
  POINT p;
  double angle;
  struct timespec t;
} LOCATION;           // spatial + temporal location combo


/* Function prototypes */

POINT read_mouse_point(int fp);
LOCATION read_mouse_location(int fp);
