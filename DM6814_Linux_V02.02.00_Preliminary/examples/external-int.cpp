/*
	FILE NAME: external-int.cpp

	FILE DESCRIPTION:

		Example program which demonstrates how to use P14 external
		interrupts.

		This program assumes that the DM6814 device being used has all
		jumpers in their factory default positions except for P14.  On
		P14, the jumper should be removed from "OT2" and placed across
		"EI1".  Additionally on P7, P8, P9, P10, P11, and P12 jumpers
		should be installed between all V and common pins

		P2 pin 2 is used to notify the board of an external interrupt.
		Port 0 bit 0 provides the external interrupt signal.  P2 pin 47
		should be connected to P2 pin 2.

		Before P14 interrupts are enabled, a value of 0 is written to
		port 0 bit 0 to pull the external interrupt 1 line low.  A
		SIGALRM signal is delivered to the program.  When that happens,
		a value of 1 is written to port 0 bit 0 to pull the external
		interrupt 1 line high and cause an interrupt.

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
#include <unistd.h>

#include <dm6814_library.h>


static char *program_name_p;
static DM6814Device DM6814;
static uint8_t alarm_sig = 0x00;


static int
setup_p14_interrupt(
    bool enable_irq,
    bool positive_polarity,
    bool share_irq
) {
    uint8_t	irq_enable = 0x00;

    if (enable_irq) {
	ENABLE_P14_IRQ(irq_enable);
    } else {
	DISABLE_P14_IRQ(irq_enable);
    }

    if (positive_polarity) {
	POSITIVE_IRQ_POLARITY(irq_enable);
    } else {
	NEGATIVE_IRQ_POLARITY(irq_enable);
    }

    if (share_irq) {
	ENABLE_IRQ_SHARING(irq_enable);
    } else {
	DISABLE_IRQ_SHARING(irq_enable);
    }

    if (! DM6814.LoadIRQRegister6814(irq_enable)) {
	return -1;
    }

    return 0;
}


static void
cleanup(void) {

    /*
     * Cancel any outstanding wakeup request
     */

    alarm(0);

    /*
     * Disable P14 interrupt
     */

    (void) setup_p14_interrupt(false, true, false);

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
sigalrm_handler(int signal_num) {
    if (signal_num != SIGALRM) {
        fail(EXIT_FAILURE, 0, "ERROR: Signal problem");
    }
    alarm_sig = 0xFF;    
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
	"    MinorNumber:         DM6814 device file minor number (0 - 3).\n"
    );
    fprintf(stderr, "\n");
    exit(EXIT_FAILURE);
}


int
main(int argument_count, char **arguments_p_p) {
    struct sigaction	sig_action;
    uint32_t		int_count;
    uint32_t		minor_number;
    uint32_t		start_int_count;
    uint8_t		status_reg;

    program_name_p = arguments_p_p[0];

    fprintf(stdout, "\n");
    fprintf(stdout, "\tDM6814 P14 External Interrupt Example Program\n");
    fprintf(stdout, "\n");

    /*
     * Set up signal used to toggle port 0 bit 0
     */

    sig_action.sa_handler = sigalrm_handler;
    sigfillset(&sig_action.sa_mask);
    sig_action.sa_flags = 0;

    if (sigaction(SIGALRM, &sig_action, NULL) < 0) {
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
     * Set digital I/O port 0 bit 0 to output
     */

    if (! DM6814.SetEncoderDIODirection6814(1, true, false)) {
	error(
	    EXIT_FAILURE, errno, "ERROR: SetEncoderDIODirection6814() FAILED"
	);
    }

    /*
     * Write a zero to port 0 bit 0 to pull external interrupt 1 pin low
     */

    if (!DM6814.WriteEncoderDIO6814(1, 0x00)) {
	error(EXIT_FAILURE, errno, "ERROR: WriteEncoderDIO6814() FAILED");
    }

    /*
     * Get driver interrupt count before interrupts are enabled
     */

    if (! DM6814.GetIntStatus6814(&start_int_count, &status_reg)) {
	error(EXIT_FAILURE, errno, "ERROR: GetIntStatus6814() FAILED");
    }

    /*
     * Enable P14 interrupt
     */

    if (setup_p14_interrupt(true, true, false) == -1) {
	error(EXIT_FAILURE, errno, "ERROR: Could not enable P14 interrupt");
    }

    /*
     * Set up a SIGALRM signal to be delivered in 3 seconds so that port 0 bit
     * 0 can be toggled
     */

    alarm(3);

    /*
     * Busy-wait for the external interrupt
     */

    fprintf(stdout, "Busy-waiting for interrupt ...\n");

    while (true) {

	/*
	 * Get driver's interrupt count and cachedIRQ Status Register
	 */

	if (! DM6814.GetIntStatus6814(&int_count, &status_reg)) {
	    fail(EXIT_FAILURE, errno, "ERROR: GetIntStatus6814() FAILED");
	}

	if ((int_count - start_int_count) != 0) {

	    /*
	     * Disable P14 interrupt
	     */

	    if (setup_p14_interrupt(false, true, false) == -1) {
		error(
		    EXIT_FAILURE,
		    errno,
		    "ERROR: Could not disable P14 interrupt"
		);
	    }

	    fprintf(stdout, "    Interrupt occurred.\n");
	    break;
	}
	
	if(alarm_sig) {
	    fprintf(stdout, "    <SIGALRM: Writing 1 to port 0 bit 0>\n");

        /*
         * Write a 1 to port 0 bit 0, thus causing an external interrupt
         */
    
        if (! DM6814.WriteEncoderDIO6814(1, 0x01)) {
            fail(EXIT_FAILURE, errno, "ERROR: WriteEncoderDIO6814() FAILED");
        }
        alarm_sig = 0x00;
	}
	
    }

    fprintf(stdout, "Checking interrupt status ...\n");
    fprintf(
	stdout,
	"    Driver saw %d interrupt(s)\n",
	(int_count - start_int_count)
    );

    /*
     * Only one interrupt should have occurred
     */

    if ((int_count - start_int_count) != 1) {
	error(EXIT_FAILURE, 0, "ERROR: Expecting only a single interrupt");
    }

    /*
     * Verify IRQ Status Register value
     */

    if (! P14_INT_OCCURRED(status_reg)) {
	error(EXIT_FAILURE, 0, "ERROR: No P14 interrrupt occurred");
    }

    if (ENC3_INT_OCCURRED(status_reg)) {
	error(EXIT_FAILURE, 0, "ERROR: Encoder 3 interrrupt occurred");
    }

    if (ENC2_INT_OCCURRED(status_reg)) {
	error(EXIT_FAILURE, 0, "ERROR: Encoder 2 interrrupt occurred");
    }

    if (ENC1_INT_OCCURRED(status_reg)) {
	error(EXIT_FAILURE, 0, "ERROR: Encoder 1 interrrupt occurred");
    }

    fprintf(stdout, "One interrupt occurred with correct IRQ Status value.\n");

    if (! DM6814.CloseBoard6814()) {
	error(EXIT_FAILURE, errno, "ERROR: CloseBoard6814() FAILED");
    }

    fprintf(stdout, "\n");
    fprintf(stdout, "Successful end of example program.\n");

    exit(EXIT_SUCCESS);
}
