/***************************************************************************

	FILE NAME: DM6604.H

	PROJECT NAME:  DM6604 Example programs

	FILE DESCRIPTION:
	This header file contains definitions of the locations of various
   registers on the DM6604 to make the code more readable.

	VERSION: 1.1

	COMPILER: Borland C++ 3.1

	TARGET: Real-Mode DOS

	Copyright 2003 RTD Embedded Technologies

***************************************************************************/

#define PPI_A  0
#define PPI_B  1
#define PPI_C  2
#define PPI_CTRL  3
#define DAC1_LSB  4
#define DAC1_MSB  5
#define DAC2_LSB  6
#define DAC2_MSB  7
#define DAC3_LSB  8
#define DAC3_MSB  9
#define DAC4_LSB  10
#define DAC4_MSB  11
#define DAC5_LSB  12
#define DAC5_MSB  13
#define DAC6_LSB  14
#define DAC6_MSB  15
#define DAC7_LSB  16
#define DAC7_MSB  17
#define DAC8_LSB  18
#define DAC8_MSB  19
#define DAC_UPDATE  20
#define IRQ_STATUS 23
#define CLEAR_INT 23

#define INPUT  1
#define OUTPUT  0

#define UNIPOLAR 0
#define BIPOLAR 1

/***** Custom defines for Space robot******/
#define PORTA  0
#define PORTB  1
#define PORTC  2



class DM6604Device{
	
  private:
  unsigned int BaseAddress, DACOffset;

  float DACSlope;

  int HighmV, LowmV;

  public:
  /***********
   Constructor
  ***********/
  
  DM6604Device(unsigned base_address);

  
  /*********** 
   Read DigitalIO
  ************
   The ReadDigitalIO function returns the value of the specified digital
   input port.  Each digital input line is represented by a bit of the
   return value. Digital in 0 is bit 0, digital in 1 is bit 1, and so on. 
  ***********/

  unsigned char readDigitalPort(unsigned char InputPort);

  
  /**************
   WriteDigitalIO
  ***************
   The WriteDigitalIO function sets the value of the digital output port to
   equal the value passed as parameter v.  Each digital output line is
   represented by a bit of v.  Digital out 0 is bit 0, digital out 1 is bit 1,
   and so on.
  ***********/
  
  void writeDigitalPort(unsigned char OutputPort, unsigned char v);




  bool readDigitalPin(unsigned char InputPort, unsigned char pin);

  void setDigitalPin(unsigned char OutputPort, unsigned char v);

  void clearDigitalPin(unsigned char OutputPort, unsigned char v);

  /****************
   ConfigureIOPorts
  *****************
   The ConfigureIOPorts procedure is used to configure the ports A and C on
   the 8255 PPI for either input or output.  A value of 1 means input, a value
   of 0 is for output.  It is advisable to use the INPUT and OUTPUT constants
   defined in this file.
  *****************/

  void configureIOPorts(unsigned char PortA, unsigned char PortB, unsigned char PortC);


  /********
  UpdateDAC
  *********
  The UpdateDAC function outputs the specified voltage to the specifed
  DAC.  The DACSlope and DACOffset variables must be set to the values
  required for the output range of the DACs. 
  ********/

  void updateDAC(unsigned char DAC, float Volts);


  /***********
   SetDACRange
  ************
   This procedure is used for calculating the constants for
   converting between voltages and bits.  The range passed to this procedure
   is the difference between the high limit and the low limit of the DAC that
   was set using the on board jumpers.  A 0 to 5 volt DAC has a range of 5
   volts, a 0 to 10 volt DAC has a range of 10 volts, a -5 to 5 volt DAC has
   a range of 10 volts, and a -10 to 10 volt DAC has a range of 20 volts.
   Polarity is also based on the selected DAC range.  Ranges of 0 to 5 volts
   or 0 to 10 volts are UNIPOLAR. Ranges of -5 to +5 volts or -10 to +10 volts
   are BIPOLAR. 
  ************/
  
  void setDACRange(int Range, unsigned char Polarity);

};