/*
** hardware.c
** Compile with: gcc -c hardware.c -O2
** Version 17/06/11
**
** Functions that deal with typical hardware stuff.
*/

#include <stdlib.h>
#include <stdio.h>

#include "common.h"


/* inb() and outb() are defined in linux,
** but not in Windows, so I define them here
** to compile and use the program.
**
** In linux you must #define PC104 before this line.
*/

#ifndef PC104

BYTE inb(MEMPORT m) {
  MEMPORT *a = NULL;
  a = m;               /* the compiler will issue a warning about this line */
//  return(*a);
}

void outb(BYTE b, MEMPORT m) {
  MEMPORT *a = NULL;
  a = m;               /* the compiler will issue a warning about this line */
//  *a=b;
}

int ioperm(MEMPORT m, unsigned int n, unsigned int i) {
  return(0);
}

#endif

#ifdef PC104
#include <sys/io.h>
#endif


/* Fundamentals: read and write memory locations.
**
** In essence what we do is rename inb() & outb(), so
** only inbyte() & outbyte() need to know about inb() & outb().
**
** inbyte(m) returns contents of memory location m
** outbyte(b,m) puts byte b to memory location m
*/


void outbyte(BYTE b, MEMPORT m) {
#ifdef DEBUG
  fprintf(stderr,"outb(%d,%d)\n",b,m);
#endif
  outb(b,m);
}


BYTE inbyte(MEMPORT m) {
#ifdef DEBUG
  fprintf(stderr,"inb(%d)\n",m);
#endif
  return(inb(m));
}

void outword(unsigned int v, MEMPORT m) {
  BYTE lsb, msb;
  lsb = v % 256;
  msb = v / 256;

  outbyte(lsb, m);
  outbyte(msb, m+1);
}

unsigned int inword(MEMPORT m) {
  BYTE lsb, msb;
  msb=inbyte(m+1);
  lsb=inbyte(m);
  return(lsb+256*msb);
}


/* Bit manipulation functions.
**
** Handy C operators:
**
** |   is bitwise OR
** &   is bitwise AND
** ~   is "one's complement", i.e. inverts bits (00101 becomes 11010)
** <<  is bitwise shift to the left, so:
**          1<<0 is binary 00000001 = decimal 2^0 = decimal 1
**          1<<1 is binary 00000010 = decimal 2^1 = decimal 2
**          1<<2 is binary 00000100 = decimal 2^2 = decimal 4
**          1<<3 is binary 00001000 = decimal 2^3 = decimal 8
**          1<<4 is binary 00010000 = decimal 2^4 = decimal 16 etc
*/

/* Read bit b of memory location m */
BOOLEAN bitread(BITPOS b, MEMPORT m) {
  return ( (inbyte(m) & (1<<b) ) ? TRUE : FALSE );
}

/* Set bit b of memory location m and return new contents */
BYTE bitset(BITPOS b, MEMPORT m) {
  BYTE a;
  a = inbyte(m) | (1<<b);
  outbyte(a, m);
  return(a);
}

/* Clear bit b of memory location m and return new contents */
BYTE bitclear(BITPOS b, MEMPORT m) {
  BYTE a;
  a = inbyte(m) & ( ~(1<<b) );
  outbyte(a, m);
  return(a);
}
