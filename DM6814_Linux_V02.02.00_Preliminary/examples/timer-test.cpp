/*
	FILE NAME: timer-test.cpp

	FILE DESCRIPTION:

		Program which tests the basic functionality of the 8254
		timer/counter library functions.

		This test makes the following assumptions about the
		timer/counters:
		    * The source of timer/counter 0 is the onboard 8 MHz clock
		    * The source of timer/counter 1 is the output of
		      timer/counter 0
		    * The source of timer/counter 2 is the output of
		      timer/counter 1
		Therefore, the DM6814 device being used must have all jumpers
		in their factory default positions.

	PROJECT NAME: Linux DM6814 Driver, Library, and Example Programs

	PROJECT VERSION: (Defined in README.TXT)

	Copyright 2005 RTD Embedded Technologies, Inc.  All Rights Reserved.
*/


#include <errno.h>
#include <error.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <dm6814_library.h>


static char *program_name_p;

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


static void
verify_frequency(float calculated_freq, float actual_freq) {
    float	difference;

    difference = fabs(calculated_freq - actual_freq);

    /*
     * If calculated frequency differs from actual frequency by one percent or
     * more, then fail
     */

    if (difference >= (actual_freq * .01)) {
	error(
	    EXIT_FAILURE,
	    0,
	    "ERROR: Calculated frequency differs from actual by at least 1 pct"
	);
    }
}


int
main(int argument_count, char **arguments_p_p) {
    DM6814Device	DM6814;
    float		calculated_freq;
    float		clock_0_rate;
    float		clock_1_rate;
    struct timespec	sleep_time;
    uint16_t		clock_1_value;
    uint16_t		clock_2_value;
    uint32_t		minor_number;
    uint8_t		loop_count;
    uint8_t		mode;
    uint8_t		timer;

    program_name_p = arguments_p_p[0];

    fprintf(stderr, "\n");
    fprintf(stderr, "\tDM6814 8254 Timer Test\n");
    fprintf(stderr, "\n");

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
	error(EXIT_FAILURE, errno, "OpenBoard6814() FAILED");
    }

    if (! DM6814.InitBoard6814()) {
	error(EXIT_FAILURE, errno, "InitBoard6814() FAILED");
    }

    /*=========================================================================
    Verify basic functionality of ClockDivisor6814()
     =========================================================================*/

    fprintf(stdout, "## Testing ClockDivisor6814() ...\n");

    fprintf(stdout, "    On invalid timer ...\n");

    if (DM6814.ClockDivisor6814(3, 0xFF)) {
	fprintf(
	    stderr, "FAILURE: ClockDivisor6814() unexpectedly succeeded!\n"
	);
	exit(EXIT_FAILURE);
    }

    fprintf(stdout, "    On all valid timers ...\n");

    for (timer = 0; timer < 3; timer++) {
	fprintf(stdout, "        Timer %d\n", timer);

	if (! DM6814.ClockDivisor6814(timer, 0xFF)) {
	    fprintf(
		stderr, "FAILURE: ClockDivisor6814() unexpectedly failed!\n"
	    );
	    exit(EXIT_FAILURE);
	}
    }

    /*=========================================================================
    Verify basic functionality of ClockMode6814()
     =========================================================================*/

    fprintf(stdout, "## Testing ClockMode6814() ...\n");

    fprintf(stdout, "    On invalid timer and valid mode ...\n");

    if (DM6814.ClockMode6814(3, 2)) {
	fprintf(
	    stderr, "FAILURE: ClockMode6814() unexpectedly succeeded!\n"
	);
	exit(EXIT_FAILURE);
    }

    fprintf(stdout, "    On valid timer and invalid mode ...\n");

    if (DM6814.ClockMode6814(0, 6)) {
	fprintf(
	    stderr, "FAILURE: ClockMode6814() unexpectedly succeeded!\n"
	);
	exit(EXIT_FAILURE);
    }

    fprintf(stdout, "    On all valid timers and modes ...\n");

    for (timer = 0; timer < 3; timer++) {
	fprintf(stdout, "        Timer %d\n", timer);

	for (mode = 0; mode < 6; mode++) {
	    fprintf(stdout, "            Mode %d\n", mode);

	    if (! DM6814.ClockMode6814(timer, mode)) {
		fprintf(
		    stderr, "FAILURE: ClockMode6814() unexpectedly failed!\n"
		);
		exit(EXIT_FAILURE);
	    }
	}
    }

    fprintf(stdout, "## Testing timer rate programming ...\n");

    /*=========================================================================
    Verify that ClockDivisor6814() and ClockMode6814() operate on the proper
    timer/counter by programming distinct rates into each
     =========================================================================*/

    fprintf(stdout, "    Using ClockMode6814()/ClockDivisor6814() ...\n");

    fprintf(stdout, "        Checking timers 0 and 1 ...\n");

    fprintf(stdout, "            Set timer 0 to 160 Hz ...\n");

    if (! DM6814.ClockMode6814(0, 3)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockMode6814() FAILED");
    }

    if (! DM6814.ClockDivisor6814(0, 50000)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockDivisor6814() FAILED");
    }

    fprintf(stdout, "            Load timer 1 initial value ...\n");

    if (! DM6814.ClockMode6814(1, 0)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockMode6814() FAILED");
    }

    if (! DM6814.ClockDivisor6814(1, 0xFFFF)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockDivisor6814() FAILED");
    }

    sleep_time.tv_nsec = 0;
    sleep_time.tv_sec = 5;

    for (loop_count = 1; loop_count < 7; loop_count++) {
	if (nanosleep(&sleep_time, NULL) == -1) {
	    error(EXIT_FAILURE, errno, "ERROR: nanosleep() FAILED");
	}

	if (! DM6814.ReadTimerCounter6814(1, &clock_1_value)) {
	    error(EXIT_FAILURE, errno, "ERROR: ReadTimerCounter6814(1) FAILED");
	}

	/*
	 * Calculate the clock frequency to verify it is ticking at the desired
	 * rate.  The value 5.0 below is the number of seconds the process was
	 * put to sleep by nanosleep().  This can vary based upon system
	 * activity but will be at least 5 seconds.  The frequency is the
	 * number of clock ticks divided by the number of seconds the clock has
	 * been running.
	 */

	calculated_freq = (
	    (float) (0xFFFF - clock_1_value) / (5.0 * loop_count)
	);

	fprintf(
	    stdout,
	    "            Calculated timer 0 frequency: %8.2f Hz\n",
	    calculated_freq
	);

	verify_frequency(calculated_freq, 160.0);
    }

    fprintf(stdout, "        Checking timers 1 and 2 ...\n");

    fprintf(stdout, "            Set timer 0 to 20 KHz ...\n");

    if (! DM6814.ClockMode6814(0, 3)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockMode6814() FAILED");
    }

    if (! DM6814.ClockDivisor6814(0, 400)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockDivisor6814() FAILED");
    }

    fprintf(stdout, "            Set timer 1 to 200 Hz ...\n");

    if (! DM6814.ClockMode6814(1, 3)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockMode6814() FAILED");
    }

    if (! DM6814.ClockDivisor6814(1, 100)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockDivisor6814() FAILED");
    }

    fprintf(stdout, "            Load timer 2 initial value ...\n");

    if (! DM6814.ClockMode6814(2, 0)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockMode6814() FAILED");
    }

    if (! DM6814.ClockDivisor6814(2, 0xFFFF)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockDivisor6814() FAILED");
    }

    sleep_time.tv_nsec = 0;
    sleep_time.tv_sec = 5;

    for (loop_count = 1; loop_count < 7; loop_count++) {
	if (nanosleep(&sleep_time, NULL) == -1) {
	    error(EXIT_FAILURE, errno, "ERROR: nanosleep() FAILED");
	}

	if (! DM6814.ReadTimerCounter6814(2, &clock_2_value)) {
	    error(EXIT_FAILURE, errno, "ERROR: ReadTimerCounter6814(2) FAILED");
	}

	/*
	 * Calculate the clock frequency to verify it is ticking at the desired
	 * rate.  The value 5.0 below is the number of seconds the process was
	 * put to sleep by nanosleep().  This can vary based upon system
	 * activity but will be at least 5 seconds.  The frequency is the
	 * number of clock ticks divided by the number of seconds the clock has
	 * been running.
	 */

	calculated_freq = (
	    (float) (0xFFFF - clock_2_value) / (5.0 * loop_count)
	);

	fprintf(
	    stdout,
	    "            Calculated timer 1 frequency: %8.2f Hz\n",
	    calculated_freq
	);

	verify_frequency(calculated_freq, 200.0);
    }

    /*=========================================================================
    Verify that SetUserClock6814() operates on the proper timer/counter by
    programming distinct rates into each
     =========================================================================*/

    fprintf(stdout, "    Using SetUserClock6814() ...\n");

    fprintf(stdout, "        Checking timers 0 and 1 ...\n");

    if (! DM6814.SetUserClock6814(0, 8000000, 320, &clock_0_rate)) {
	error(EXIT_FAILURE, errno, "ERROR: SetUserClock6814(0) FAILED");
    }

    fprintf(stdout, "            Timer 0\n");
    fprintf(stdout, "                Requested: %8.2f Hz\n", 320.0);
    fprintf(stdout, "                Actual:    %8.2f Hz\n", clock_0_rate);

    fprintf(stdout, "            Load timer 1 initial value ...\n");

    if (! DM6814.ClockMode6814(1, 0)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockMode6814() FAILED");
    }

    if (! DM6814.ClockDivisor6814(1, 0xFFFF)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockDivisor6814() FAILED");
    }

    sleep_time.tv_nsec = 0;
    sleep_time.tv_sec = 5;

    for (loop_count = 1; loop_count < 7; loop_count++) {
	if (nanosleep(&sleep_time, NULL) == -1) {
	    error(EXIT_FAILURE, errno, "ERROR: nanosleep() FAILED");
	}

	if (! DM6814.ReadTimerCounter6814(1, &clock_1_value)) {
	    error(EXIT_FAILURE, errno, "ERROR: ReadTimerCounter6814(1) FAILED");
	}

	/*
	 * Calculate the clock frequency to verify it is ticking at the desired
	 * rate.  The value 5.0 below is the number of seconds the process was
	 * put to sleep by nanosleep().  This can vary based upon system
	 * activity but will be at least 5 seconds.  The frequency is the
	 * number of clock ticks divided by the number of seconds the clock has
	 * been running.
	 */

	calculated_freq = (
	    (float) (0xFFFF - clock_1_value) / (5.0 * loop_count)
	);

	fprintf(
	    stdout,
	    "            Calculated timer 0 frequency: %8.2f Hz\n",
	    calculated_freq
	);

	verify_frequency(calculated_freq, 320.0);
    }

    fprintf(stdout, "        Checking timers 1 and 2 ...\n");

    if (! DM6814.SetUserClock6814(0, 8000000, 10000, &clock_0_rate)) {
	error(EXIT_FAILURE, errno, "ERROR: SetUserClock6814(0) FAILED");
    }

    fprintf(stdout, "            Timer 0\n");
    fprintf(stdout, "                Requested: %8.2f Hz\n", 10000.0);
    fprintf(stdout, "                Actual:    %8.2f Hz\n", clock_0_rate);

    if (! DM6814.SetUserClock6814(1, 10000, 1000, &clock_1_rate)) {
	error(EXIT_FAILURE, errno, "ERROR: SetUserClock6814(1) FAILED");
    }

    fprintf(stdout, "            Timer 1\n");
    fprintf(stdout, "                Requested: %8.2f Hz\n", 1000.0);
    fprintf(stdout, "                Actual:    %8.2f Hz\n", clock_1_rate);

    fprintf(stdout, "            Load timer 2 initial value ...\n");

    if (! DM6814.ClockMode6814(2, 0)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockMode6814() FAILED");
    }

    if (! DM6814.ClockDivisor6814(2, 0xFFFF)) {
	error(EXIT_FAILURE, errno, "ERROR: ClockDivisor6814() FAILED");
    }

    sleep_time.tv_nsec = 0;
    sleep_time.tv_sec = 5;

    for (loop_count = 1; loop_count < 7; loop_count++) {
	if (nanosleep(&sleep_time, NULL) == -1) {
	    error(EXIT_FAILURE, errno, "ERROR: nanosleep() FAILED");
	}

	if (! DM6814.ReadTimerCounter6814(2, &clock_2_value)) {
	    error(EXIT_FAILURE, errno, "ERROR: ReadTimerCounter6814(2) FAILED");
	}

	/*
	 * Calculate the clock frequency to verify it is ticking at the desired
	 * rate.  The value 5.0 below is the number of seconds the process was
	 * put to sleep by nanosleep().  This can vary based upon system
	 * activity but will be at least 5 seconds.  The frequency is the
	 * number of clock ticks divided by the number of seconds the clock has
	 * been running.
	 */

	calculated_freq = (
	    (float) (0xFFFF - clock_2_value) / (5.0 * loop_count)
	);

	fprintf(
	    stdout,
	    "            Calculated timer 1 frequency: %8.2f Hz\n",
	    calculated_freq
	);

	verify_frequency(calculated_freq, 1000.0);
    }

    fprintf(stdout, "\n");
    fprintf(stdout, "Success.  All tests passed.\n");

    exit(EXIT_SUCCESS);
}
