/*
	FILE NAME: timer-int.cpp

	FILE DESCRIPTION:

		Example program which demonstrates how to use P14 timer
		interrupts.

		This program assumes that the DM6814 device being used has all
		jumpers in their factory default positions.  Additionally on
		P7, P8, P9, P10, P11, and P12 jumpers should be installed
		between all V and common pins

		Timer/counters 0, 1, and 2 are put into rate generator mode.
		Timer/counter 0 is set to 20 KHz.  Timer/counter 1 is set to
		2 KHz.  Timer/counter 2 is set to 5 Hz.  Timer/counter 2 is the
		source of P14 interrupts.  Therefore, interrupts occur every
		200 milliseconds.

		During each execution of the main loop, a value (between 0 and
		3) is written to port 0 bits 0 and 1 and a value is read from
		port 2 bits 2 and 3.  P2 pin 47 should be connected to P2 pin
		43 and P2 pin 45 should be connected to P2 pin 41.

		Aproximately every five seconds, signal SIGALRM is delivered to
		the program.  The signal handler displays the following
		information:
		    * number of interrupts seen by the program so far
		    * the most recent data written to port 0
		    * the most recent data read from port 2

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
#include <sys/select.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <dm6814_library.h>


static char *program_name_p;
static DM6814Device DM6814;
static volatile uint8_t output_data = 0;
static volatile uint8_t input_data;
static volatile uint32_t int_total = 0;
static uint8_t exit_program = 0x00;


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
sigalrm_handler(int signal_num) {
    if (signal_num != SIGALRM) {
	fprintf(
	    stderr,
	    "ERROR: SIGALRM handler received incorrect signal %d\n",
	    signal_num
	);
	fail(EXIT_FAILURE, 0, "ERROR: Signal problem");
    }

    fprintf(stdout, "Port status at interrupt %d\n", int_total);
    fprintf(stdout, "    Port 0 bits 0/1 output:    0x%1x\n", output_data);
    fprintf(stdout, "    Port 0 bits 2/3 input:     0x%1x\n", input_data);

    /*
     * Set another wakeup call for 5 seconds from now
     */

    alarm(5);
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
    float		clock_0_rate;
    float		clock_1_rate;
    float		clock_2_rate;
    struct sigaction	sig_action;
    uint32_t		last_int_count = 0;
    uint32_t		minor_number;

    program_name_p = arguments_p_p[0];

    fprintf(stderr, "\n");
    fprintf(stderr, "\tDM6814 P14 Timer Interrupt Example Program\n");
    fprintf(stderr, "\n");

    /*
     * Set up signal used to display port input data
     */

    sig_action.sa_handler = sigalrm_handler;
    sigfillset(&sig_action.sa_mask);
    sig_action.sa_flags = 0;

    if (sigaction(SIGALRM, &sig_action, NULL) < 0) {
	error(EXIT_FAILURE, errno, "sigaction(SIGALRM) FAILED");
    }

    /*
     * Set up a way for user to interrupt the program via CONTROL-C
     */

    sig_action.sa_handler = control_c_handler;
    sigfillset(&sig_action.sa_mask);
    sig_action.sa_flags = 0;

    if (sigaction(SIGINT, &sig_action, NULL) < 0) {
        error(EXIT_FAILURE, errno, "sigaction(SIGINT) FAILED");
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
     * Set digital I/O port 0 bits 0 and 1 to output
     */

    if (! DM6814.SetEncoderDIODirection6814(1, true, true)) {
	error(
	    EXIT_FAILURE, errno, "ERROR: SetEncoderDIODirection6814(1) FAILED"
	);
    }

    /*=========================================================================
    Timer/counter initialization
     =========================================================================*/

    /*
     * Set timer/counter 0 to 20 KHz
     */

    if (! DM6814.SetUserClock6814(0, 8000000.0, 20000.0, &clock_0_rate)) {
	error(EXIT_FAILURE, errno, "ERROR: SetUserClock6814(0) FAILED");
    }

    fprintf(stdout, "Timer/counter 0 set to %8.2f Hz\n", clock_0_rate);

    /*
     * Set timer/counter 1 to 2 KHz
     */

    if (! DM6814.SetUserClock6814(1, clock_0_rate, 2000.0, &clock_1_rate)) {
	error(EXIT_FAILURE, errno, "ERROR: SetUserClock6814(1) FAILED");
    }

    fprintf(stdout, "Timer/counter 1 set to %8.2f Hz\n", clock_1_rate);

    /*
     * Set timer/counter 2 to 5 Hz
     */

    if (! DM6814.SetUserClock6814(2, clock_1_rate, 5.0, &clock_2_rate)) {
	error(EXIT_FAILURE, errno, "ERROR: SetUserClock6814(2) FAILED");
    }

    fprintf(stdout, "Timer/counter 2 set to %8.2f Hz\n", clock_2_rate);

    /*=========================================================================
    Set up SIGALRM wakeup to display data
     =========================================================================*/

    /*
     * Set up a SIGALRM signal to be delivered in 5 seconds so that port input
     * data can be displayed
     */

    alarm(5);

    fprintf(stdout, "\n");
    fprintf(
	stdout,
	"Waiting for interrupts.  Port input data will be displayed about\n"
    );
    fprintf(
	stdout,
	"every 5 seconds.  Press CONTROL-C to exit.\n"
    );

    /*=========================================================================
    Get the interrupts going
     =========================================================================*/

    /*
     * Enable P14 interrupt
     */

    if (setup_p14_interrupt(true, true, false) == -1) {
	error(EXIT_FAILURE, errno, "ERROR: Could not enable P14 interrupt");
    }

    /*=========================================================================
    Main interrupt processing loop
     =========================================================================*/

    /*
     * Loop checking interrupt status
     */

    while (!exit_program) {
	uint32_t	int_count;
	uint32_t	int_difference;
	uint8_t		data;
	uint8_t		status_reg;

	/*
	 * Get driver's interrupt count and cached IRQ Status Register
	 */

	if (! DM6814.GetIntStatus6814(&int_count, &status_reg)) {
	    fail(EXIT_FAILURE, errno, "ERROR: GetIntStatus6814() FAILED");
	}

	/*
	 * If this is the first interrupt, save interrupt count and go back to
	 * get interrupt status again
	 */

	if (last_int_count == 0) {
	    last_int_count = int_count;
	    continue;
	}

	/*
	 * Figure out how many interrupts occurred since last status was read
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

	/*
	 * Verify IRQ Status Register value
	 */

	if (! P14_INT_OCCURRED(status_reg)) {
	    fail(EXIT_FAILURE, 0, "ERROR: No P14 interrupt occurred");
	}

	if (ENC3_INT_OCCURRED(status_reg)) {
	    fail(EXIT_FAILURE, 0, "ERROR: Encoder 3 interrupt occurred");
	}

	if (ENC2_INT_OCCURRED(status_reg)) {
	    fail(EXIT_FAILURE, 0, "ERROR: Encoder 2 interrupt occurred");
	}

	if (ENC1_INT_OCCURRED(status_reg)) {
	    fail(EXIT_FAILURE, 0, "ERROR: Encoder 1 interrupt occurred");
	}

	/*
	 * Save interrupt count and count this interrupt
	 */

	last_int_count = int_count;
	int_total++;

	/*
	 * Write to digital I/O port 0 bits 0 and 1
	 */

	if (! DM6814.WriteEncoderDIO6814(1, output_data)) {
	    fail(EXIT_FAILURE, errno, "ERROR: WriteEncoderDIO6814(1) FAILED");
	}

	/*
	 * Read digital I/O port 0 bits 2 and 3
	 */

	if (! DM6814.ReadEncoderDIO6814(1, &data)) {
	    fail(EXIT_FAILURE, errno, "ERROR: ReadEncoderDIO6814(1) FAILED");
	}

	/*
	 * Bits 2 and 3 in value contain port 0 bits 2 and 3.  Move these 2
	 * bits into the least significant 2 bits and mask out all other bits.
	 */

	input_data = ((data >> 2) & 0x03);

	/*
	 * Limit output values to [0 .. 3] because only 2 bits are available
	 */

	output_data = ((output_data + 1) % 4);
    }
    
    cleanup();

    fprintf(stdout, "\n");
    fprintf(stdout, "Successful end of example program.\n");

    exit(EXIT_SUCCESS);
}
