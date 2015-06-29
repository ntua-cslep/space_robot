/*
** Author: Ilias Patsiaouras
**
** Version 29 jun 15
**
** This file include a class Definition for using the DAC module
** All routines are for the DM6604HR DAC module by RTD
** This class has been created based on DOS source code example from RTD site
** Each DAC module (card) has eight DAC output channels (1-8) with a 12bit resolution each
** The output voltage range is based on the onboard jumpers
**       [-5,+5]V, [-10,+10]V, [0,+10]V or [0,+5]V
*/

#include <sys/io.h>
#include "dm6604.h"

#include <ros/ros.h>



DM6604Device::DM6604Device(unsigned base_address)
{
  BaseAddress = base_address;
  //ConfigureIOPorts(INPUT, INPUT, OUTPUT);

  if ( ioperm(BaseAddress, 23, 1) != 0 ) {
    ROS_ERROR("Error: cannot set DM6604 permissions with %x base address!", BaseAddress);
  }
  setDACRange(20, BIPOLAR);
}


unsigned char 
DM6604Device::readDigitalIO(unsigned char InputPort)
{
 return(inb(BaseAddress + PPI_A + InputPort));
}


void 
DM6604Device::writeDigitalIO(unsigned char OutputPort, unsigned char v)
{
 outb(v, BaseAddress + PPI_A + OutputPort);
}


void 
DM6604Device::configureIOPorts(unsigned char PortA, unsigned char PortB, unsigned char PortC)
{
 unsigned char ControlByte = 128 + (PortA * 16) + (PortB * 2) + (PortC * 9);

 outb(ControlByte, BaseAddress + PPI_CTRL);
}


void 
DM6604Device::updateDAC(unsigned char DAC, float Volts)
{
 int Value;
 Value = (Volts * DACSlope) + DACOffset;
 outb(Value % 256 , BaseAddress + DAC1_LSB + (DAC - 1) * 2);
 outb(Value / 256 , BaseAddress + DAC1_MSB + (DAC - 1) * 2);
 outb(0           , BaseAddress + DAC_UPDATE);
}


void 
DM6604Device::setDACRange(int Range, unsigned char Polarity)
{
 switch (Range)
  {
   case 20 :
    {
     HighmV = 10000;
     LowmV = -10000;
     DACOffset = 2048;
    } break;
   case 5 :
    {
     HighmV = 5000;
     LowmV = 0;
     DACOffset = 0;
    } break;

   case 10 :
    {
     if (Polarity == UNIPOLAR)
      {
       HighmV = 10000;
       LowmV = 0;
       DACOffset = 0;
      }
     else
      {
       HighmV = 5000;
       LowmV = -5000;
       DACOffset = 2048;
      }
     }
   }
  DACSlope = 4095.0 / ((HighmV - LowmV) / 1000);
}



