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