#
#	FILE NAME: Makefile
#
#	FILE DESCRIPTION: Make description file for building DM6814 library
#
#	PROJECT NAME: Linux DM6814 Driver, Library, and Example Programs
#
#	PROJECT VERSION: (Defined in README.TXT)
#
#	Copyright 2005 RTD Embedded Technologies, Inc.  All Rights Reserved.
#

CC=g++
DEBUG_FLAGS=
INCLUDE_FLAGS=-I../include
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall
CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

all:			librtd-dm6814.a

librtd-dm6814.a:	librtd-dm6814.o
	ar -cru librtd-dm6814.a librtd-dm6814.o

librtd-dm6814.o:	librtd-dm6814.cpp
	$(CC) $(CFLAGS) -c librtd-dm6814.cpp

clean:
	rm -f *.o *.a
