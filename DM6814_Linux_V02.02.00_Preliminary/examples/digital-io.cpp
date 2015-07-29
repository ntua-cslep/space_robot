/*
	FILE NAME: digital-io.cpp

	FILE DESCRIPTION:

		Example program which demonstrates reading from and writing to
		the digital I/O ports.

		This program assumes that the DM6814 device being used has all
		jumpers in their factory default positions.  Additionally on
		P7, P8, P9, P10, P11, and P12 jumpers should be installed
		between all V and common pins

		Port 0 bits 0 and 1 are set to output.  Port 0 bits 2 and 3 are
		used to receive the value written to port 0 bits 0 and 1.
		Therefore, P2 pin 47 should be connected to P2 pin 43 and P2
		pin 45 should be connected to P2 pin 41.

		Port 2 bits 0 and 1 are set to output.  Port 1 bits 0 and 1 are
		used to receive the value written to port 2 bits 0 and 1.
		Therefore, P2 pin 31 should be connected to P3 pin 47 and P2
		pin 29 should be connected to P3 pin 45.

		Port 4 bits 0 and 1 are set to output.  Port 1 bits 2 and 3 are
		used to receive the value written to port 4 bits 0 and 1.
		Therefore, P2 pin 15 should be connected to P3 pin 43 and P2
		pin 13 should be connected to P3 pin 41.

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
	fprintf(stderr, "CONTROL-C handler received signal %d\n", signal_num);
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
    uint8_t		port_0_output = 0x00;
    uint8_t		port_2_output = 0x01;
    uint8_t		port_4_output = 0x02;

    program_name_p = arguments_p_p[0];

    fprintf(stderr, "\n");
    fprintf(stderr, "\tDM6814 Digital I/O Example Program\n");
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
     * Set port 0 bits 0 and 1 to output
     */

    if (! DM6814.SetEncoderDIODirection6814(1, true, true)) {
	error(
	    EXIT_FAILURE, errno, "ERROR: SetEncoderDIODirection6814(1) FAILED"
	);
    }

    /*=========================================================================
    Incremental encoder 2 initialization
     =========================================================================*/

    /*
     * Set port 2 bits 0 and 1 to output
     */

    if (! DM6814.SetEncoderDIODirection6814(2, true, true)) {
	error(
	    EXIT_FAILURE, errno, "ERROR: SetEncoderDIODirection6814(2) FAILED"
	);
    }

    /*=========================================================================
    Incremental encoder 3 initialization
     =========================================================================*/

    /*
     * Set port 4 bits 0 and 1 to output
     */

    if (! DM6814.SetEncoderDIODirection6814(3, true, true)) {
	error(
	    EXIT_FAILURE, errno, "ERROR: SetEncoderDIODirection6814(3) FAILED"
	);
    }

    /*=========================================================================
    Main processing loop
     =========================================================================*/

    fprintf(stdout, "\n");

    /*
     * Loop reading and writing digital I/O ports
     */

    while (!exit_program) {
	uint8_t	input_data;
	uint8_t	p0_bits_2_3;
	uint8_t	p1_bits_0_1;
	uint8_t	p1_bits_2_3;

	fprintf(
	    stdout,
	    "Press ENTER to read/write digital I/O.  Press CONTROL-C to exit.\n"
	);
	(void) fgetc(stdin);

	/*=====================================================================
	Write to digital I/O ports
	 =====================================================================*/

	/*
	 * Write a value to digital I/O port 0 bits 0 and 1
	 */

	if (! DM6814.WriteEncoderDIO6814(1, port_0_output)) {
	    error(EXIT_FAILURE, errno, "ERROR: WriteEncoderDIO6814(1) FAILED");
	}

	/*
	 * Write a value to digital I/O port 2 bits 0 and 1
	 */

	if (! DM6814.WriteEncoderDIO6814(2, port_2_output)) {
	    error(EXIT_FAILURE, errno, "ERROR: WriteEncoderDIO6814(2) FAILED");
	}

	/*
	 * Write a value to digital I/O port 4 bits 0 and 1
	 */

	if (! DM6814.WriteEncoderDIO6814(3, port_4_output)) {
	    error(EXIT_FAILURE, errno, "ERROR: WriteEncoderDIO6814(3) FAILED");
	}

	/*=====================================================================
	Read digital I/O ports
	 =====================================================================*/

	/*
	 * Read values on digital I/O port 0 bits 2 and 3 and port 1 bits 0, 1,
	 * 2, and 3
	 */

	if (! DM6814.ReadEncoderDIO6814(1, &input_data)) {
	    error(EXIT_FAILURE, errno, "ERROR: ReadEncoderDIO6814(1) FAILED");
	}

	/*=====================================================================
	Extract input data fields
	 =====================================================================*/

	/*
	 * Get port 0 bits 2 and 3 from input value bits 2 and 3
	 */

	p0_bits_2_3 = ((input_data >> 2) & 0x03);

	/*
	 * Get port 1 bits 0 and 1 from input value bits 4 and 5
	 */

	p1_bits_0_1 = ((input_data >> 4) & 0x03);

	/*
	 * Get port 1 bits 2 and 3 from input value bits 6 and 7
	 */

	p1_bits_2_3 = ((input_data >> 6) & 0x03);

	/*=====================================================================
	Print port data
	 =====================================================================*/

	fprintf(stdout, "Port data\n");

	/*
	 * Since port 0 bits 0 and 1 are connected to port 0 bits 2 and 3,
	 * print this information together
	 */

	fprintf(
	    stdout, "    Port 0 bits 0/1 output:    0x%1x\n", port_0_output
	);
	fprintf(
	    stdout, "    Port 0 bits 2/3 input:     0x%1x\n", p0_bits_2_3
	);

	/*
	 * Since port 2 bits 0 and 1 are connected to port 1 bits 0 and 1,
	 * print this information together
	 */

	fprintf(
	    stdout, "    Port 2 bits 0/1 output:    0x%1x\n", port_2_output
	);
	fprintf(
	    stdout, "    Port 1 bits 0/1 input:     0x%1x\n", p1_bits_0_1
	);

	/*
	 * Since port 4 bits 0 and 1 are connected to port 1 bits 2 and 3,
	 * print this information together
	 */

	fprintf(
	    stdout, "    Port 4 bits 0/1 output:    0x%1x\n", port_4_output
	);
	fprintf(
	    stdout, "    Port 1 bits 2/3 input:     0x%1x\n", p1_bits_2_3
	);

	/*=====================================================================
	Update output values
	 =====================================================================*/

	/*
	 * Limit port 0 output values to [0 .. 3] because only 2 bits are
	 * available
	 */

	port_0_output = ((port_0_output + 1) % 4);

	/*
	 * Limit port 2 output values to [0 .. 3] because only 2 bits are
	 * available
	 */

	port_2_output = ((port_2_output + 1) % 4);

	/*
	 * Limit port 4 output values to [0 .. 3] because only 2 bits are
	 * available
	 */

	port_4_output = ((port_4_output + 1) % 4);
    }
    
    (void) DM6814.CloseBoard6814();

    fprintf(stdout, "\n");
    fprintf(stdout, "Successful end of example program.\n");

    exit(EXIT_SUCCESS);
}
