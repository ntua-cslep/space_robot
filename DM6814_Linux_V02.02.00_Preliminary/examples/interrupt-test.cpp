/*
	FILE NAME: interrupt-test.cpp

	FILE DESCRIPTION:

		Program which tests the library interrupt-related functions.

		This program assumes that the DM6814 device being used has all
		jumpers in their factory default positions.

		The incremental encoder 1 IRQ input signal is used as the
		interrupt source.  Port 0 bit 0 provides the incremental
		encoder interrupt signal.  P2 pin 47 should be connected to P2
		pin 33.

	PROJECT NAME: Linux DM6814 Driver, Library, and Example Programs

	PROJECT VERSION: (Defined in README.TXT)

	Copyright 2005 RTD Embedded Technologies, Inc.  All Rights Reserved.
*/


#include <errno.h>
#include <error.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <dm6814_library.h>


static char *program_name_p;
static DM6814Device DM6814;


static void
expect_success(bool status) {
    if (! status) {
	fprintf(stderr, "FAILED: Expected success but failure occurred.\n");
	fprintf(stderr, "    errno is %d\n", errno);
	exit(EXIT_FAILURE);
    }
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
    bool		status;
    struct timespec	sleep_time;
    uint32_t		int_count;
    uint32_t		last_int_count;
    uint32_t		minor_number;
    uint32_t		start_int_count;
    uint8_t		status_reg;

    program_name_p = arguments_p_p[0];

    fprintf(stderr, "\n");
    fprintf(stderr, "\tDM6814 Interrupt Test\n");
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

    fprintf(stdout, "## Testing interrupts ...\n");

    fprintf(stdout, "    Open DM6814 device ...\n");

    status = DM6814.OpenBoard6814(minor_number);
    expect_success(status);

    fprintf(stdout, "    Initialize DM6814 device ...\n");

    status = DM6814.InitBoard6814();
    expect_success(status);

    /*=========================================================================
    Incremental encoder 1 initialization
     =========================================================================*/

    /*
     * Disable encoder interrupt
     */

    fprintf(stdout, "    Disable encoder 1 interrupt ...\n");

    status = DM6814.EnableEncoderIrq6814(1, false);
    expect_success(status);

    /*
     * Set port 0 bit 0 to output
     */

    fprintf(stdout, "    Set port 0 bit 0 to output ...\n");

    status = DM6814.SetEncoderDIODirection6814(1, true, false);
    expect_success(status);

    /*
     * Write a 0 to port 0 bit 0 to pull incremental encoder 1 IRQ input signal
     * low
     */

    fprintf(stdout, "    Write 0 to port 0 bit 0 ...\n");

    status = DM6814.WriteEncoderDIO6814(1, 0x00);
    expect_success(status);

    /*=========================================================================
    Incremental encoder 2 initialization
     =========================================================================*/

    /*
     * Disable encoder interrupt
     */

    fprintf(stdout, "    Disable encoder 2 interrupt ...\n");

    status = DM6814.EnableEncoderIrq6814(2, false);
    expect_success(status);

    /*=========================================================================
    Incremental encoder 3 initialization
     =========================================================================*/

    /*
     * Disable encoder interrupt
     */

    fprintf(stdout, "    Disable encoder 3 interrupt ...\n");

    status = DM6814.EnableEncoderIrq6814(3, false);
    expect_success(status);

    /*=========================================================================
    Verify that when interrupts are enabled but incremental encoder 1 IRQ input
    signal is low, no interrupts occur
     =========================================================================*/

    /*
     * Get initial interrupt status from driver
     */

    fprintf(stdout, "    Get initial interrupt status ...\n");

    status = DM6814.GetIntStatus6814(&int_count, &status_reg);
    expect_success(status);

    last_int_count = int_count;
    start_int_count = int_count;

    /*
     * Enable incremental encoder 1 interrupt
     */

    fprintf(stdout, "    Enable encoder 1 interrupt ...\n");

    status = DM6814.EnableEncoderIrq6814(1, true);
    expect_success(status);

    /*
     * Sleep a bit then see if any unexpected interrupts occur
     */

    fprintf(stdout, "    Sleep 5 seconds to check interrupt status ...\n");

    sleep_time.tv_sec = 5;
    sleep_time.tv_nsec = 0;

    if (nanosleep(&sleep_time, NULL) == -1) {
	error(EXIT_FAILURE, errno, "ERROR: nanosleep() FAILED");
    }

    /*
     * Get interrupt status from driver
     */

    fprintf(stdout, "    Get interrupt status ...\n");

    status = DM6814.GetIntStatus6814(&int_count, &status_reg);
    expect_success(status);

    if (int_count != last_int_count) {
	error(EXIT_FAILURE, 0, "ERROR: Unexpected interrupts occurred");
    }

    /*=========================================================================
    Cause an incremental encoder 1 interrupt and verify interrupt status
     =========================================================================*/

    /*
     * Write a 1 to port 0 bit 0 to pull incremental encoder 1 IRQ input signal
     * high and cause an interrupt
     */

    fprintf(stdout, "    Write a 1 to port 0 bit 0 ...\n");

    status = DM6814.WriteEncoderDIO6814(1, 0x01);
    expect_success(status);

    /*
     * Sleep a bit then see how many interrupts occurred
     */

    fprintf(stdout, "    Sleep 5 seconds to check interrupt status ...\n");

    sleep_time.tv_sec = 5;
    sleep_time.tv_nsec = 0;

    if (nanosleep(&sleep_time, NULL) == -1) {
	error(EXIT_FAILURE, errno, "ERROR: nanosleep() FAILED");
    }

    /*
     * Get interrupt status from driver
     */

    fprintf(stdout, "    Get interrupt status ...\n");

    status = DM6814.GetIntStatus6814(&int_count, &status_reg);
    expect_success(status);

    /*
     * Verify interrupt count and status
     */

    if ((int_count - last_int_count) == 0) {
	error(EXIT_FAILURE, 0, "ERROR: No interrupts occurred");
    }

    if ((int_count - last_int_count) != 1) {
	error(EXIT_FAILURE, 0, "ERROR: Wrong number of interrupts occurred");
    }

    if (P14_INT_OCCURRED(status_reg)) {
	error(EXIT_FAILURE, 0, "ERROR: P14 interrupt occurred");
    }

    if (ENC3_INT_OCCURRED(status_reg)) {
	error(EXIT_FAILURE, 0, "ERROR: Encoder 3 interrupt occurred");
    }

    if (ENC2_INT_OCCURRED(status_reg)) {
	error(EXIT_FAILURE, 0, "ERROR: Encoder 2 interrupt occurred");
    }

    if (! ENC1_INT_OCCURRED(status_reg)) {
	error(EXIT_FAILURE, 0, "ERROR: No encoder 1 interrupt occurred");
    }

    last_int_count = int_count;

    /*=========================================================================
    Pull incremental encoder 1 IRQ input signal low and verify that no
    interrupts occur
     =========================================================================*/

    /*
     * Write a 0 to port 0 bit 0 to pull incremental encoder 1 IRQ input signal
     * low
     */

    fprintf(stdout, "    Write 0 to port 0 bit 0 ...\n");

    status = DM6814.WriteEncoderDIO6814(1, 0x00);
    expect_success(status);

    /*
     * Sleep a bit then see if any unexpected interrupts occur
     */

    fprintf(stdout, "    Sleep 5 seconds to check interrupt status ...\n");

    sleep_time.tv_sec = 5;
    sleep_time.tv_nsec = 0;

    if (nanosleep(&sleep_time, NULL) == -1) {
	error(EXIT_FAILURE, errno, "ERROR: nanosleep() FAILED");
    }

    /*
     * Get interrupt status from driver
     */

    fprintf(stdout, "    Get interrupt status ...\n");

    status = DM6814.GetIntStatus6814(&int_count, &status_reg);
    expect_success(status);

    if (int_count != last_int_count) {
	error(EXIT_FAILURE, 0, "ERROR: Unexpected interrupts occurred");
    }

    /*=========================================================================
    Cause an incremental encoder 1 interrupt and verify interrupt status
     =========================================================================*/

    /*
     * Write a 1 to port 0 bit 0 to pull incremental encoder 1 IRQ input signal
     * high and cause an interrupt
     */

    fprintf(stdout, "    Write a 1 to port 0 bit 0 ...\n");

    status = DM6814.WriteEncoderDIO6814(1, 0x01);
    expect_success(status);

    /*
     * Sleep a bit then see how many interrupts occurred
     */

    fprintf(stdout, "    Sleep 5 seconds to check interrupt status ...\n");

    sleep_time.tv_sec = 5;
    sleep_time.tv_nsec = 0;

    if (nanosleep(&sleep_time, NULL) == -1) {
	error(EXIT_FAILURE, errno, "ERROR: nanosleep() FAILED");
    }

    /*
     * Get interrupt status from driver
     */

    fprintf(stdout, "    Get interrupt status ...\n");

    status = DM6814.GetIntStatus6814(&int_count, &status_reg);
    expect_success(status);

    /*
     * Verify interrupt count and status
     */

    if ((int_count - last_int_count) == 0) {
	error(EXIT_FAILURE, 0, "ERROR: No interrupts occurred");
    }

    if ((int_count - last_int_count) != 1) {
	error(EXIT_FAILURE, 0, "ERROR: Wrong number of interrupts occurred");
    }

    if (P14_INT_OCCURRED(status_reg)) {
	error(EXIT_FAILURE, 0, "ERROR: P14 interrupt occurred");
    }

    if (ENC3_INT_OCCURRED(status_reg)) {
	error(EXIT_FAILURE, 0, "ERROR: Encoder 3 interrupt occurred");
    }

    if (ENC2_INT_OCCURRED(status_reg)) {
	error(EXIT_FAILURE, 0, "ERROR: Encoder 2 interrupt occurred");
    }

    if (! ENC1_INT_OCCURRED(status_reg)) {
	error(EXIT_FAILURE, 0, "ERROR: No encoder 1 interrupt occurred");
    }

    last_int_count = int_count;

    /*=========================================================================
    Disable incremental encoder 1 interrupt and check final interrupt status
     =========================================================================*/

    /*
     * Disable incremental encoder 1 interrupt
     */

    fprintf(stdout, "    Disable encoder 1 interrupt ...\n");

    status = DM6814.EnableEncoderIrq6814(1, false);
    expect_success(status);

    /*
     * Sleep a bit then see how many interrupts occurred
     */

    fprintf(stdout, "    Sleep 5 seconds to check interrupt status ...\n");

    sleep_time.tv_sec = 5;
    sleep_time.tv_nsec = 0;

    if (nanosleep(&sleep_time, NULL) == -1) {
	error(EXIT_FAILURE, errno, "ERROR: nanosleep() FAILED");
    }

    /*
     * Get interrupt status from driver
     */

    fprintf(stdout, "    Get interrupt status ...\n");

    status = DM6814.GetIntStatus6814(&int_count, &status_reg);
    expect_success(status);

    if (int_count != last_int_count) {
	error(EXIT_FAILURE, 0, "ERROR: Unexpected interrupts occurred");
    }

    if ((int_count - start_int_count) != 2) {
	error(
	    EXIT_FAILURE,
	    errno,
	    "ERROR: Expected interrupt total of 2 not received"
	);
    }

    fprintf(stdout, "\n");
    fprintf(stdout, "Success.  All tests passed.\n");

    exit(EXIT_SUCCESS);
}
