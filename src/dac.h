/*
** dac.h
**
** Version 06/05/08
**
** This should contain stuff ONLY needed in dac.c
*/

#ifndef DAC_H
#define DAC_H


#define DAC_LSB 4      /* DAC channel LSB */
#define DAC_MSB 5      /* DAC channel MSB */
#define DAC_UPDATE 20  /* Used for updating simultaneously all eight DAC channels */


#define DAC_HIGH 10000  /* DAC highest voltage in mV */
#define DAC_LOW -10000  /* DAC lowest voltage in mV */


/*
** DACOffset and DACSlope at UpadateDAC are used for converting volts to bits
**
** low is the lowest voltage of the specified range and high is the highest,
** units are mV and the acceptable values are:
**   [0,+5000], [0,+10000], [-5000,+5000], [-10000,+10000]
*/

#define DACOffset(low) (((low)==0)?0.0:2048.0) 
#define DACSlope(low, high) (4095.0/(((high)-(low))/1000.0))


#endif   /* #define DAC_H  */
