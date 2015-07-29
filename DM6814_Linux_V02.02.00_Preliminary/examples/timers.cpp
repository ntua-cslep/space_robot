/*
	FILE NAME: timers.cpp

	FILE DESCRIPTION:

		Example program which demonstrates using the 8254
		timer/counters in rate generator and event count modes.

		This program assumes that the DM6814 device being used has all
		jumpers in their factory default positions.

		Timer/counter 0 is put into rate generator mode and set to run
		at 200 Hz.  Timer/counter 1 is put into event count mode and
		initially loaded with the value 0xFFFF.  Timer/counter 1 is
		decremented every tick of timer/counter 0, i.e. every 5
		milliseconds.  Thus, timer/counter 1 indicates the length of
		time that timer/counter 0 has been running.

	PROJECT NAME: Linux DM6814 Driver, Library, and Example Programs

	PROJECT VERSION: (Defined in README.TXT)

	Copyright 2005 RTD Embedded Technologies, Inc.  All Rights Reserved.
*/


#include <errno.h>
#include <error.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <dm6814_library.h>


static char *program_name_p;
static DM6814Device DM6814;
static uint8_t exit_program = 0x00;


static void
control_c_handler(int signal_num) {
    
    /*
     * Make sure only SIGINT is received
     */

    if (signal_num != SIGINT) {
        exit(EXIT_FAILURE);
    }

    exit_program = 0xFF;
}



static void
usage(void) {
    fprintf(stderr, "\n");
    fprintf(stderr, "%s\n", program_name_p);
    fprintf(stderr, "\n");
    fprintf(stderr, "Usage: %s MinorNumber\n", program_name_p);
    fprintf(stderr, "\n");
    fprintf(
	stderr,
	"    MinorNumber:    DM6814 device file minor number (0 - 3).\n"
    );
    fprintf(stderr, "\n");
    exit(EXIT_FAILURE);
}


int
main(int argument_count, char **arguments_p_p) {
    struct sigaction	sig_action;
    uint32_t           minor_number;

    program_name_p = arguments_p_p[0];

    fprintf(stderr, "\n");
    fprintf(stderr, "\tDM6814 Timer Example Program\n");
    fprintf(stderr, "\n");

    /*
     * Set up a way for user to interrupt the program via CONTROL-C
     */

    sig_action.sa_handler = control_c_handler;
    sigfillset(&sig_action.sa_mask);
    sig_action.sa_flags = 0;

    if (sigaction(SIGINT, &sig_action, NULL) < 0) {
	error(EXIT_FAILURE, errno, "sigaction() FAILED");
    }

    /*
     * Process command line options
     */

    if (argument_count != 2) {
	fprintf(stderr, "ERROR: Invalid number of options given!\n");
	usage();
    }

    if (sscanf(arguments_p_p[1], "%u", &minor_number) == 0) {
	fprintf(stderr, "ERROR: Minor number must be an integer!\n");
	usage();
    }

    if (minor_number > 3) {
	fprintf(stderr, "ERROR: Minor number must be between 0 and 3!\n");
	usage();
    }

    if (! DM6814.OpenBoard6814(minor_number)) {
	error(EXIT_FAILURE, errno, "ERROR: OpenBoard6814() FAILED");
    }

    if (! DM6814.InitBoard6814()) {
	error(EXIT_FAILURE, errno, "ERROR: InitBoard6814() FAILED");
    }

    /*
     * Put timer/counter 0 into rate generator mode
     */

    if (! DM6814.ClockMode6814(0, 2)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockMode6814(0) FAILED");
    }

    /*
     * Put timer/counter 1 into event count mode
     */

    if (! DM6814.ClockMode6814(1, 0)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockMode6814(1) FAILED");
    }

    /*
     * Set timer/counter 0 to 200 Hz
     */

    if (! DM6814.ClockDivisor6814(0, 40000)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockDivisor6814(0) FAILED");
    }

    /*
     * Set initial value of timer/counter 1
     */

    if (! DM6814.ClockDivisor6814(1, 0xFFFF)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockDivisor6814(2) FAILED");
    }

    fprintf(stdout, "Press CONTROL-C to exit.\n");
    fprintf(stdout, "\n");
    fprintf(stdout, "Elapsed time (in seconds)\n");

    /*
     * Loop reading timer/counter 1 value
     */

    while (!exit_program) {
	uint16_t	timer_1_value;

	if (! DM6814.ReadTimerCounter6814(1, &timer_1_value)) {
	    error(EXIT_FAILURE, errno, "ERROR: ReadTimerCounter6814() FAILED");
	}

	/*
	 * Elapsed time is number of timer 1 ticks divided by timer 0 frequency
	 */

	fprintf(
	    stdout,
	    "%8.2f\r",
	    (float) (0xFFFF - timer_1_value) / 200.0
	);

	fflush(stdout);
    }
    
    (void) DM6814.CloseBoard6814();

    fprintf(stdout, "\n");
    fprintf(stdout, "Successful end of example program.\n");

    exit(EXIT_SUCCESS);
}
