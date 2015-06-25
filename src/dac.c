/* dac.c 
**
** Version 19 July 11
**
** This file has all the routines for using  the DAC module
** All routines are for the DM6604HR DAC module by RTD
** Each DAC module (card) has eight DAC output channels (1-8) with a 12bit resolution each
** The output voltage range is: [-5,+5]V, [-10,+10]V, [0,+10]V or [0,+5]V
** The manual's routines have been altered, SetDACRange has been replced with 
** #define DACOffset(low) and #define DACSlope(low,high)
** and UpdateDAC has been modified accordingly
**
** changes: unsigned int dac instead of unsigned char dac!
*/

/* When compiling include the -lm flag.
** Otherwise, despite the fact that math.h is included, the floor command is considered undefined.
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#ifdef PC104
#include <sys/io.h>
#endif

#include "common.h"
#include "dac.h"

/* Set DAC Board Permissions */
void SetDAC_Board(MEMPORT address)
{
if ( ioperm(address,23,1) != 0)     /* Set DAC permissions */
  {
    printf("Error: cannot set DAC permissions!\n");
    exit(1);
  }
}

/* UpdateDAC: outputs the specified voltage to the specifed DAC channel (1-8) */
void UpdateDAC(unsigned int dac, const double volts)
{
  int value = (int)(floor(volts * DACSlope(DAC_LOW, DAC_HIGH) + DACOffset(DAC_LOW)));

//  printf("Arguments are DAC=%d, Volts=%f, value=%d\n", dac, volts, value);

  outbyte(value % 256, DAC_BA + DAC_LSB + (dac - 1) * 2);	/* LSB */
  outbyte(value / 256, DAC_BA + DAC_MSB + (dac - 1) * 2);	/* MSB */

  outbyte(0, DAC_BA + DAC_UPDATE);                          /* Update all channels with the specified voltage */
}

/* SendDAC: updates all 8 DAC channels with the same value*/
void SendDAC(MEMPORT address, const double volts)
{
  
  UpdateDAC(1, volts);
  UpdateDAC(2, volts);
  UpdateDAC(3, volts);
  UpdateDAC(4, volts);
  UpdateDAC(5, volts);
  UpdateDAC(6, volts);
  UpdateDAC(7, volts);
  UpdateDAC(8, volts);
}

/* The ConfigureIOPorts procedure is used to configure the ports A, B C upper and C lower on
 the 8255 PPI at the DAC Board for either input or output.  A value of 1 means input, a value
 of 0 is for output. */

void ConfigureIOPorts(unsigned int PortA, unsigned int PortB,
		      unsigned int PortC_UP, unsigned int PortC_LOW)
{
  unsigned int ControlByte;

  ControlByte = 128 + (PortA * 16) + (PortB * 2) + (PortC_UP * 8) + (PortC_LOW * 1);
  outbyte( ControlByte, PORT_CTRL);
}

/* Thrusters are connected to PortA, set to output
Thruster 1 -> PA0
Thruster 2 -> PA1
Thruster 3 -> PA2
Thruster 4 -> PA3
Thruster 5 -> PA4
Thruster 6 -> PA5
N/C -> PA6
N/C -> PA7
*/

void OpenDigital(BITPOS b, MEMPORT address)
{
  BYTE a;
  a = bitset(b, address);
//  printf("Digital %d at address %u is set to %u\n", b, address, a);

}

void CloseDigital(BITPOS b, MEMPORT address)
{
  BYTE a;
  a = bitclear(b, address);
//  printf("Digital %d at address %u is set to %u\n", b, address, a);

}

unsigned int ReadDigital(BITPOS b, MEMPORT address)
{
  unsigned int i;
  if(bitread(b, address)) {
    i=1;
  } else {
    i=0;
  }
  printf("Digital %d at address %u is %u\n", b, address, i );
  return(i);
}

/*ReadPort: returns the value of the specified digital
 input port (8 bits).  Each digital input line is represented by a bit of the
 return value. Digital in 0 is bit 0, digital in 1 is bit 1, and so on. */

BYTE ReadPORT(MEMPORT address)
{
 return(inbyte(address));
}

/* WritePORT: sets the value of the digital output port (8 bits) to
 equal the value passed as parameter v.  Each digital output line is
 represented by a bit of v.  Digital out 0 is bit 0, digital out 1 is bit 1,
 and so on. */

void WritePORT( BYTE v, MEMPORT address)
{
 outbyte( v, address);
}

