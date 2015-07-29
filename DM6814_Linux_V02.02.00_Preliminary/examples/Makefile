#
#	FILE NAME: Makefile
#
#	FILE DESCRIPTION: Make description file for building example programs
#
#	PROJECT NAME: Linux DM6814 Driver, Library, and Example Programs
#
#	PROJECT VERSION: (Defined in README.TXT)
#
#	Copyright 2005 RTD Embedded Technologies, Inc.  All Rights Reserved.
#

CC=g++
DEBUG_FLAGS=-g
INCLUDE_FLAGS=-I../include
LIBRARY_FLAGS=-L../lib -lrtd-dm6814
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall

CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

EXAMPLES= \
	digital-io \
	encoder-int \
	encoders \
	external-int \
	int-wait-via-read \
	int-wait-via-select \
	timer-int \
	timers

TESTS= \
	basic-test \
	dio-test \
	interrupt-test \
	timer-test

all:	$(EXAMPLES) $(TESTS)

basic-test:	basic-test.cpp
	$(CC) $(CFLAGS) -o basic-test basic-test.cpp $(LIBRARY_FLAGS)

digital-io:	digital-io.cpp
	$(CC) $(CFLAGS) -o digital-io digital-io.cpp $(LIBRARY_FLAGS)

dio-test:	dio-test.cpp
	$(CC) $(CFLAGS) -o dio-test dio-test.cpp $(LIBRARY_FLAGS)

encoder-int:	encoder-int.cpp
	$(CC) $(CFLAGS) -o encoder-int encoder-int.cpp $(LIBRARY_FLAGS)

encoders:	encoders.cpp
	$(CC) $(CFLAGS) -o encoders encoders.cpp $(LIBRARY_FLAGS)

external-int:	external-int.cpp
	$(CC) $(CFLAGS) -o external-int external-int.cpp $(LIBRARY_FLAGS)

interrupt-test:		interrupt-test.cpp
	$(CC) $(CFLAGS) -o interrupt-test interrupt-test.cpp $(LIBRARY_FLAGS)

int-wait-via-read:		int-wait-via-read.cpp
	$(CC) $(CFLAGS) -o int-wait-via-read int-wait-via-read.cpp \
		$(LIBRARY_FLAGS)

int-wait-via-select:		int-wait-via-select.cpp
	$(CC) $(CFLAGS) -o int-wait-via-select int-wait-via-select.cpp \
		$(LIBRARY_FLAGS)

timer-int:	timer-int.cpp
	$(CC) $(CFLAGS) -o timer-int timer-int.cpp $(LIBRARY_FLAGS)

timers:		timers.cpp
	$(CC) $(CFLAGS) -o timers timers.cpp $(LIBRARY_FLAGS)

timer-test:	timer-test.cpp
	$(CC) $(CFLAGS) -o timer-test timer-test.cpp $(LIBRARY_FLAGS)

clean:
	rm -f *.o *~ $(EXAMPLES) $(TESTS)
