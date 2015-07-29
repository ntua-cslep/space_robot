/*
	FILE NAME: encoders.cpp

	FILE DESCRIPTION:

		Example program which demonstrates reading the incremental
		encoders.

		This program assumes that the DM6814 device being used has all
		jumpers in their factory default positions.

		An external incremental encoder should be connected to P2 as
		follows:

		    Encoder Pin		P2 Pin
		    ==========================
			-		50
			A		39, 23, and 7
			+		49
			B		37, 21, and 5

		The way the incremental encoder is connected, it serves as a
		single update source for each of the 3 encoder counters.

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
    uint32_t		minor_number;

    program_name_p = arguments_p_p[0];

    fprintf(stderr, "\n");
    fprintf(stderr, "\tDM6814 Incremental Encoder Read Example Program\n");
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
    Main processing loop
     =========================================================================*/

    fprintf(stdout, "\n");

    /*
     * Loop reading incremental encoders
     */

    while (!exit_program) {
	uint16_t	encoder_1_val;
	uint16_t	encoder_2_val;
	uint16_t	encoder_3_val;

	fprintf(
	    stdout,
	    "Press ENTER to read encoders.  Press CONTROL-C to exit.\n"
	);
	(void) fgetc(stdin);

	/*=====================================================================
	Read the incremental encoders
	 =====================================================================*/

	if (! DM6814.ReadEncoder6814(1, &encoder_1_val)) {
	    error(EXIT_FAILURE, errno, "ERROR: ReadEncoder6814(1) FAILED");
	}

	if (! DM6814.ReadEncoder6814(2, &encoder_2_val)) {
	    error(EXIT_FAILURE, errno, "ERROR: ReadEncoder6814(2) FAILED");
	}

	if (! DM6814.ReadEncoder6814(3, &encoder_3_val)) {
	    error(EXIT_FAILURE, errno, "ERROR: ReadEncoder6814(3) FAILED");
	}

	/*=====================================================================
	Print encoder values
         =====================================================================*/

	fprintf(stdout, "Incremental encoder values\n");

	fprintf(stdout, "    Encoder 1: 0x%4X\n", encoder_1_val);
	fprintf(stdout, "    Encoder 2: 0x%4X\n", encoder_2_val);
	fprintf(stdout, "    Encoder 3: 0x%4X\n", encoder_3_val);
    }
    
    (void) DM6814.CloseBoard6814();

    fprintf(stdout, "\n");
    fprintf(stdout, "Successful end of example program.\n");

    exit(EXIT_SUCCESS);
}
