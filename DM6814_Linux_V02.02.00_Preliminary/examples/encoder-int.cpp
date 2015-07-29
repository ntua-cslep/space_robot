/*
	FILE NAME: encoder-int.cpp

	FILE DESCRIPTION:

		Example program which demonstrates how to use incremental
		encoder interrupts.

		This program assumes that the DM6814 device being used has all
		jumpers in their factory default positions.

		An external incremental encoder should be connected to the
		device as follows:
		    * encoder pin "-" to P2 pin 50
		    * encoder pin "A" to P2 pins 39, 23, and 7
		    * encoder pin "+" to P2 pin 49
		    * encoder pin "B" to P2 pins 37, 21, and 5
		These connections allow the single external encoder to function
		as a single update source for each of the 3 encoder counters.

		In addition, the following P2 connections should be made:
		    * pin 35 to pin 33
		    * pin 19 to pin 17
		    * pin 3 to pin 1
		These connections feed the overflow output signal for each
		encoder into its interrupt input so that an encoder interrupt
		can be generated when a counter overflows/underflows.

		This program uses busy-wait to repeatedly check for interrupt
		occurrence.

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


static void
cleanup(void) {

    /*
     * Disable all incremental encoder interrupts
     */

    (void) DM6814.EnableEncoderIrq6814(1, false);
    (void) DM6814.EnableEncoderIrq6814(2, false);
    (void) DM6814.EnableEncoderIrq6814(3, false);

    (void) DM6814.CloseBoard6814();
}


static void
fail(
    int exit_status,
    int error_number,
    const char *message_p
) {
    cleanup();
    error(exit_status, error_number, message_p);
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
    uint32_t		int_count;
    uint32_t		last_int_count;
    uint32_t		minor_number;
    uint32_t		start_int_count;
    uint8_t		status_reg;

    program_name_p = arguments_p_p[0];

    fprintf(stderr, "\n");
    fprintf(stderr, "\tDM6814 Incremental Encoder Interrupt Example Program\n");
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
	error(EXIT_FAILURE, errno, "ERROR: OpenBoard6814() FAILED");
    }

    if (! DM6814.InitBoard6814()) {
	error(EXIT_FAILURE, errno, "ERROR: InitBoard6814() FAILED");
    }

    /*=========================================================================
    Incremental encoder 1 initialization
     =========================================================================*/

    /*
     * Disable the encoder
     */

    if (! DM6814.EnableEncoder6814(1, false)) {
	error(EXIT_FAILURE, errno, "ERROR: EnableEncoder6814(1, false) FAILED");
    }

    /*
     * Disable encoder interrupt
     */

    if (! DM6814.EnableEncoderIrq6814(1, false)) {
	error(EXIT_FAILURE, errno, "ERROR: EnableEncoderIrq6814(1) FAILED");
    }

    /*
     * Load a value into the encoder.  The encoder should be disabled before
     * loading a value into it.
     */

    if (! DM6814.LoadEncoder6814(1, 0xC000)) {
	error(EXIT_FAILURE, errno, "ERROR: LoadEncoder6814(1) FAILED");
    }

    /*
     * Enable the encoder
     */

    if (! DM6814.EnableEncoder6814(1, true)) {
	error(EXIT_FAILURE, errno, "ERROR: EnableEncoder6814(1, true) FAILED");
    }

    /*=========================================================================
    Incremental encoder 2 initialization
     =========================================================================*/

    /*
     * Disable the encoder
     */

    if (! DM6814.EnableEncoder6814(2, false)) {
	error(EXIT_FAILURE, errno, "ERROR: EnableEncoder6814(2, false) FAILED");
    }

    /*
     * Disable encoder interrupt
     */

    if (! DM6814.EnableEncoderIrq6814(2, false)) {
	error(EXIT_FAILURE, errno, "ERROR: EnableEncoderIrq6814(2) FAILED");
    }

    /*
     * Load a value into the encoder.  The encoder should be disabled before
     * loading a value into it.
     */

    if (! DM6814.LoadEncoder6814(2, 0x8000)) {
	error(EXIT_FAILURE, errno, "ERROR: LoadEncoder6814(2) FAILED");
    }

    /*
     * Enable the encoder
     */

    if (! DM6814.EnableEncoder6814(2, true)) {
	error(EXIT_FAILURE, errno, "ERROR: EnableEncoder6814(2, true) FAILED");
    }

    /*=========================================================================
    Incremental encoder 3 initialization
     =========================================================================*/

    /*
     * Disable the encoder
     */

    if (! DM6814.EnableEncoder6814(3, false)) {
	error(EXIT_FAILURE, errno, "ERROR: EnableEncoder6814(3) FAILED");
    }

    /*
     * Disable encoder interrupt
     */

    if (! DM6814.EnableEncoderIrq6814(3, false)) {
	error(EXIT_FAILURE, errno, "ERROR: EnableEncoderIrq6814(3) FAILED");
    }

    /*
     * Load a value into the encoder.  The encoder should be disabled before
     * loading a value into it.
     */

    if (! DM6814.LoadEncoder6814(3, 0x4000)) {
	error(EXIT_FAILURE, errno, "ERROR: LoadEncoder6814(3) FAILED");
    }

    /*
     * Enable the encoder
     */

    if (! DM6814.EnableEncoder6814(3, true)) {
	error(EXIT_FAILURE, errno, "ERROR: EnableEncoder6814(3, true) FAILED");
    }

    /*=========================================================================
    Interrupt count initialization
     =========================================================================*/

    /*
     * Get driver's interrupt count before interrupts are enabled
     */

    if (! DM6814.GetIntStatus6814(&start_int_count, &status_reg)) {
	error(EXIT_FAILURE, errno, "ERROR: GetIntStatus6814(4) FAILED");
    }

    /*
     * Most recent interrupt count is the initial count
     */

    last_int_count = start_int_count;

    /*=========================================================================
    Get the interrupts going
     =========================================================================*/

    /*
     * Enable incremental encoder 1 interrupt
     */

    if (! DM6814.EnableEncoderIrq6814(1, true)) {
	error(EXIT_FAILURE, errno, "ERROR: Encoder 1 interrupt enable FAILED");
    }

    /*
     * Enable incremental encoder 2 interrupt
     */

    if (! DM6814.EnableEncoderIrq6814(2, true)) {
	error(EXIT_FAILURE, errno, "ERROR: Encoder 2 interrupt enable FAILED");
    }

    /*
     * Enable incremental encoder 3 interrupt
     */

    if (! DM6814.EnableEncoderIrq6814(3, true)) {
	error(EXIT_FAILURE, errno, "ERROR: Encoder 3 interrupt enable FAILED");
    }

    /*=========================================================================
    Main processing loop
     =========================================================================*/

    fprintf(stdout, "\n");
    fprintf(stdout, "Busy-waiting for interrupt ...\n");

    while (true) {
	uint32_t	int_difference;

	/*=====================================================================
	Loop getting interrupt status until driver says an interrupt occurred
	 =====================================================================*/

	/*
	 * Get driver's interrupt count and cached IRQ Status Register
	 */

	if (! DM6814.GetIntStatus6814(&int_count, &status_reg)) {
	    fail(EXIT_FAILURE, errno, "ERROR: GetIntStatus6814() FAILED");
	}

	/*
	 * Figure out how many interrupts occurred since last status read
	 */

	int_difference = (int_count - last_int_count);

	/*
	 * If no interrupts occurred, we are reading status faster than
	 * interrupts are being generated.  Just go back and read status again.
	 */

	if (int_difference == 0) {
	    continue;
	}

	/*
	 * If more than one interrupt occurred, this program is not reading
	 * status fast enough.  In other words, it is missing interrupts.
	 */

	if (int_difference != 1) {
	    fprintf(stderr, "\n");
	    fprintf(
		stderr,
		">> %d interrupts occurred since last status\n",
		int_difference
	    );
	    fail(EXIT_FAILURE, 0, "ERROR: Application is losing interrupts");
	}

	fprintf(stdout, "\n");
	fprintf(stdout, "Interrupt occurred\n");

	/*=====================================================================
	Make sure IRQ status Register value is correct
	 =====================================================================*/

	fprintf(stdout, "\n");
	fprintf(stdout, "Verifying IRQ Status Register value ...\n");

	/*
	 * No P14 interrupt should have occurred
	 */

	if (P14_INT_OCCURRED(status_reg)) {
	    fail(EXIT_FAILURE, 0, "ERROR: P14 interrupt occurred");
	}

	/*
	 * At least one of the encoders should have interrupted
	 */

	if (
	    (! ENC3_INT_OCCURRED(status_reg))
	    &&
	    (! ENC2_INT_OCCURRED(status_reg))
	    &&
	    (! ENC1_INT_OCCURRED(status_reg))
	) {
	    fail(EXIT_FAILURE, 0, "ERROR: No encoder interrupt occurred");
	}

	if (ENC3_INT_OCCURRED(status_reg)) {
	    fprintf(stdout, "    Incremental encoder 3 interrupt occurred\n");
	}

	if (ENC2_INT_OCCURRED(status_reg)) {
	    fprintf(stdout, "    Incremental encoder 2 interrupt occurred\n");
	}

	if (ENC1_INT_OCCURRED(status_reg)) {
	    fprintf(stdout, "    Incremental encoder 1 interrupt occurred\n");
	}

	/*
	 * Interrupt occurred, time to exit
	 */

	break;
    }

    /*=========================================================================
    Final processing before exit
     =========================================================================*/

    /*
     * Disable incremental encoder 1 interrupt
     */

    if (! DM6814.EnableEncoderIrq6814(1, false)) {
	error(EXIT_FAILURE, errno, "ERROR: Encoder 1 interrupt disable FAILED");
    }

    /*
     * Disable incremental encoder 2 interrupt
     */

    if (! DM6814.EnableEncoderIrq6814(2, false)) {
	error(EXIT_FAILURE, errno, "ERROR: Encoder 2 interrupt disable FAILED");
    }

    /*
     * Disable incremental encoder 3 interrupt
     */

    if (! DM6814.EnableEncoderIrq6814(3, false)) {
	error(EXIT_FAILURE, errno, "ERROR: Encoder 3 interrupt disable FAILED");
    }

    /*
     * Interrupts are disabled again, so get ending driver interrupt count
     */

    if (! DM6814.GetIntStatus6814(&int_count, &status_reg)) {
	error(EXIT_FAILURE, errno, "ERROR: GetIntStatus6814() FAILED");
    }

    /*
     * Verify that driver saw one interrupt
     */

    if ((int_count - start_int_count) != 1) {
	fprintf(
	    stderr,
	    "Driver saw % interrupts, which is incorrect\n",
	    (int_count - start_int_count)
	);
	error(EXIT_FAILURE, 0, "ERROR: Interrupt problem");
    }

    fprintf(stdout, "\n");
    fprintf(stdout, "Successful end of example program.\n");

    exit(EXIT_SUCCESS);
}
