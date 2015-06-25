#include <sys/io.h>
#include "dm6604.h"

#include "ros/ros.h"

class DM6604{
  private:
  unsigned int BaseAddress, DACOffset;

  float DACSlope;

  int HighmV, LowmV;

  public:
  /***********
  
   Constructor

  ***********/
  
  DM6604(unsigned base_address);
  {
    BaseAddress = base_address;

    //ConfigureIOPorts(INPUT, INPUT, OUTPUT);

    if ( ioperm(BaseAddress, 23, 1) != 0 ) {
      ROS_ERROR("Error: cannot set DM6604 permissions with %x base address!", BaseAddress);
    }

    SetDACRange(20, BIPOLAR);
  }
  
  /***********
  
   Read DigitalIO
  
   The ReadDigitalIO function returns the value of the specified digital
   input port.  Each digital input line is represented by a bit of the
   return value. Digital in 0 is bit 0, digital in 1 is bit 1, and so on.
  
  ***********/

  unsigned char ReadDigitalIO(unsigned char InputPort)
  {
   return(inb(BaseAddress + PPI_A + InputPort));
  }
  
  /***********
  
   WriteDigitalIO
  
   The WriteDigitalIO function sets the value of the digital output port to
   equal the value passed as parameter v.  Each digital output line is
   represented by a bit of v.  Digital out 0 is bit 0, digital out 1 is bit 1,
   and so on.
  
  ***********/
  
  void WriteDigitalIO(unsigned char OutputPort, unsigned char v)
  {
   outb(v, BaseAddress + PPI_A + OutputPort);
  }
  
  /*****************
  
   ConfigureIOPorts
  
   The ConfigureIOPorts procedure is used to configure the ports A and C on
   the 8255 PPI for either input or output.  A value of 1 means input, a value
   of 0 is for output.  It is advisable to use the INPUT and OUTPUT constants
   defined in this file.
  
  *****************/
  
  void ConfigureIOPorts(unsigned char PortA, unsigned char PortB, unsigned char PortC)
  {
   unsigned char ControlByte = 128 + (PortA * 16) + (PortB * 2) + (PortC * 9);

   outb(ControlByte, BaseAddress + PPI_CTRL);
  }

  /*****************
  
  UpdateDAC
  
  The UpdateDAC function outputs the specified voltage to the specifed
  DAC.  The DACSlope and DACOffset variables must be set to the values
  required for the output range of the DACs.
  
  *****************/

  void UpdateDAC(unsigned char DAC, float Volts)
  {
   int Value;
   Value = (Volts * DACSlope) + DACOffset;
   outb(Value % 256 , BaseAddress + DAC1_LSB + (DAC - 1) * 2);
   outb(Value / 256 , BaseAddress + DAC1_MSB + (DAC - 1) * 2);
   outb(0           , BaseAddress + DAC_UPDATE);
  }

  /*****************
  
   The SetDACRange procedure is used for calculating the constants for
   converting between voltages and bits.  The range passed to this procedure
   is the difference between the high limit and the low limit of the DAC that
   was set using the on board jumpers.  A 0 to 5 volt DAC has a range of 5
   volts, a 0 to 10 volt DAC has a range of 10 volts, a -5 to 5 volt DAC has
   a range of 10 volts, and a -10 to 10 volt DAC has a range of 20 volts.
   Polarity is also based on the selected DAC range.  Ranges of 0 to 5 volts
   or 0 to 10 volts are UNIPOLAR. Ranges of -5 to +5 volts or -10 to +10 volts
   are BIPOLAR.
  
  *****************/
  
  void SetDACRange(int Range, unsigned char Polarity)
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


}