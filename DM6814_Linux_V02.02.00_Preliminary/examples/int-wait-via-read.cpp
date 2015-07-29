/*
	FILE NAME: int-wait-via-read.cpp

	FILE DESCRIPTION:

		Example program which demonstrates waiting for interrupt
		notification.  In this particular example, the read(2) system
		call is used to perform the wait.  To do this, blocking I/O
		must be set for the device.

		P14 timer interrupts are used as the interrupt source.  In
		particular, timer/counter 2 generates the interrupts.

		This program assumes that the DM6814 device being used has all
		jumpers in their factory default positions.

		Timer/counter 0 is put into rate generator mode and is set to
		20 KHz.  Timer/counter 1 is put into rate generator mode and is
		set to 2 KHz.  Timer/counter 2 is put into rate generator mode
		and is set to 1 Hz.  Since timer/counter 2 generates the
		interrupts, the program receives interrupts once per second.

		A process waiting for interrupt occurrence can be awoken
		prematurely by receipt of a signal.  The application bears the
		responsibility for handling this situation in an appropriate
		manner.  Therefore, this program also demonstrates how to deal
		with early wakeup due to signal delivery.

		Even though signals are used herein, they are not required to
		wait for interrupts to occur.  Rather the signals function to
		prematurely wake up an interrupt wait so that the proper way to
		handle this situation can be shown.

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
#include <sys/time.h>
#include <unistd.h>

#include <dm6814_library.h>


static char *program_name_p;
static DM6814Device DM6814;


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


static int
cancel_interval_timer(void) {
    struct itimerval	timer_cancel;

    timer_cancel.it_interval.tv_sec = 0;
    timer_cancel.it_interval.tv_usec = 0;
    timer_cancel.it_value.tv_sec = 0;
    timer_cancel.it_value.tv_usec = 0;

    return setitimer(ITIMER_REAL, &timer_cancel, NULL);
}


static void
cleanup(void) {

    /*
     * Cancel any outstanding interval timer expiration
     */

    (void) cancel_interval_timer();

    /*
     * Disable P14 interrupts
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
	fprintf(
	    stderr,
	    "ERROR: SIGALRM handler received incorrect signal %d\n",
	    signal_num
	);
	fail(EXIT_FAILURE, 0, "ERROR: Signal problem");
    }
}


static void
usage(void) {
    fprintf(stderr, "\n");
    fprintf(stderr, "%s\n", program_name_p);
    fprintf(stderr, "\n");
    fprintf(
	stderr,
	"Usage: %s MinorNumber NumberInterrupts\n",
	program_name_p
    );
    fprintf(stderr, "\n");
    fprintf(
	stderr,
	"    MinorNumber:         DM6814 device file minor number (0 - 3).\n"
    );
    fprintf(stderr, "\n");
    fprintf(
	stderr,
	"    NumberInterrupts:    Number of interrupts to handle.\n"
    );
    fprintf(stderr, "\n");
    exit(EXIT_FAILURE);
}


int
main(int argument_count, char **arguments_p_p) {
    float		clock_0_rate;
    float		clock_1_rate;
    float		clock_2_rate;
    struct itimerval	timer_setup;
    struct sigaction	sig_action;
    uint32_t		int_count;
    uint32_t		int_total = 0;
    uint32_t		last_int_count;
    uint32_t		minor_number;
    uint32_t		num_interrupts;
    uint32_t		start_int_count;
    uint8_t		status_reg;

    fprintf(stderr, "\n");
    fprintf(stderr, "\tDM6814 Interrupt Wait Via read(2) Example Program\n");
    fprintf(stderr, "\n");

    program_name_p = arguments_p_p[0];

    /*
     * Set up SIGALRM handler.  SIGALRM will be delivered twice a second so
     * that waiting for interrupt occurrence will prematurely awaken and the
     * proper technique for retrying the wait can be demonstrated.
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

    if (argument_count != 3) {
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

    if (sscanf(arguments_p_p[2], "%u", &num_interrupts) == 0) {
	fprintf(stderr, "ERROR: Number of interrupts must be an integer!\n");
	usage();
    }

    if (num_interrupts == 0) {
	fprintf(
	    stderr, "ERROR: Number of interrupts must be greater than 0!\n"
	);
	usage();
    }

    /*=========================================================================
    Device initialization
     =========================================================================*/

    /*
     * Open the device file
     */

    fprintf(stdout, "Open DM6814 device file ...\n");

    if (! DM6814.OpenBoard6814(minor_number)) {
	error(EXIT_FAILURE, errno, "OpenBoard6814() FAILED");
    }

    /*
     * Initialize the board
     */

    fprintf(stdout, "Initialize DM6814 device ...\n");

    if (! DM6814.InitBoard6814()) {
	error(EXIT_FAILURE, errno, "InitBoard6814() FAILED");
    }

    /*=========================================================================
    8254 timer/counter initialization
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
     * Set timer/counter 2 to 1 Hz
     */

    if (! DM6814.SetUserClock6814(2, clock_1_rate, 1.0, &clock_2_rate)) {
	error(EXIT_FAILURE, errno, "ERROR: SetUserClock6814(2) FAILED");
    }

    fprintf(stdout, "Timer/counter 2 set to %8.2f Hz\n", clock_2_rate);

    /*=========================================================================
    Interrupt count initialization
     =========================================================================*/

    /*
     * Get driver interrupt count before interrupts are enabled and blocking
     * I/O is turned on
     */

    if (! DM6814.GetIntStatus6814(&start_int_count, &status_reg)) {
	error(EXIT_FAILURE, errno, "ERROR: GetIntStatus6814() FAILED");
    }

    /*
     * Most recent interrupt count is the initial count
     */

    last_int_count = start_int_count;

    /*
     * Set blocking I/O for the device file.  This enables GetIntStatus6814()
     * to block in the kernel until an interrupt occurs.  Do this after getting
     * the initial status so that call does not block forever.
     */

    if (! DM6814.EnableGetIntStatusWait6814(true)) {
	error(EXIT_FAILURE, errno, "EnableGetIntStatusWait6814(true) FAILED");
    }

    /*
     * Set up interval timer to deliver SIGALRM to process twice per second.
     * This provides signal interference so that proper technique for retrying
     * interrupt wait can be demonstrated
     */

    timer_setup.it_interval.tv_sec = 0;
    timer_setup.it_interval.tv_usec = 500000;
    timer_setup.it_value.tv_sec = 0;
    timer_setup.it_value.tv_usec = 500000;

    if (setitimer(ITIMER_REAL, &timer_setup, NULL) != 0) {
	error(EXIT_FAILURE, errno, "ERROR: setitimer(setup) FAILED");
    }

    /*=========================================================================
    Get the interrupts going
     =========================================================================*/

    /*
     * Turn on P14 interrupts
     */

    if (setup_p14_interrupt(true, true, false) == -1) {
	error(EXIT_FAILURE, errno, "ERROR: Could not enable P14 interrupt");
    }

    /*=========================================================================
    Wait for and process interrupt notifications
     =========================================================================*/

    fprintf(stdout, "Sleeping for interrupts ...\n");

    while (int_total < num_interrupts) {
	uint32_t	int_difference;

	/*
	 * Put the process to sleep waiting for an interrupt
	 */

	if (! DM6814.GetIntStatus6814(&int_count, &status_reg)) {

	    /*
	     * See if the function was interrupted by a signal
	     */

	    if (errno == EINTR) {

		/*
		 * Signal interrupted the wait, so wait again because an
		 * interrupt did not occur
		 */

		continue;
	    }

	    /*
	     * Unrecoverable error occurred
	     */

	    fail(EXIT_FAILURE, errno, "ERROR: GetIntStatus6814() FAILED");
	}

	/*
	 * Figure out how many interrupts occurred since last status read
	 */

        int_difference = (int_count - last_int_count);

	/*
	 * If no interrupts occurred, something is very wrong
	 */

	if (int_difference == 0) {
	    fail(EXIT_FAILURE, 0, "ERROR: Notification without interrupt");
	}

	/*
	 * If more than one interrupt occurred, something is very wrong
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
	 * Verify IRQ Status Register Value
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
	 * Count this interrupt
	 */

	int_total++;

	fprintf(stdout, "    Interrupt number %d occurred.\n", int_total);

	/*
	 * Save current interrupt count as most recent count
	 */

	last_int_count = int_count;
    }

    /*=========================================================================
    Desired number of interrupts have occurred so turn interrupts off and
    verify results
     =========================================================================*/

    fprintf(stdout, "Interrupt count reached, exiting ...\n");

    /*
     * Disable P14 interrupts
     */

    if (setup_p14_interrupt(false, true, false) == -1) {
	error(EXIT_FAILURE, errno, "ERROR: Could not disable P14 interrupt");
    }

    /*
     * Cancel any outstanding interval timer expiration
     */

    if (cancel_interval_timer() != 0) {
	error(EXIT_FAILURE, errno, "ERROR: Interval timer cancel FAILED");
    }

    /*
     * Set non-blocking I/O for the device file.  This disables
     * GetIntStatus6814() from blocking in the kernel until an interrupt
     * occurs.  Do this before getting final status otherwise that call will
     * block forever.
     */

    if (! DM6814.EnableGetIntStatusWait6814(false)) {
	error(EXIT_FAILURE, errno, "EnableGetIntStatusWait6814(false) FAILED");
    }

    /*
     * Interrupts are disabled again, so get ending driver interrupt count
     */

    if (! DM6814.GetIntStatus6814(&int_count, &status_reg)) {
	error(EXIT_FAILURE, errno, "ERROR: GetIntStatus6814() FAILED");
    }

    /*
     * Verify that driver saw correct number of interrupts
     */

    if ((int_count - start_int_count) != num_interrupts) {
	fprintf(
	    stderr,
	    "Driver saw % interrupts, which is incorrect\n",
	    (int_count - start_int_count)
	);
	error(EXIT_FAILURE, 0, "ERROR: Interrupt problem");
    }

    /*
     * Verify that program saw correct number of interrupts
     */

    if (int_total != num_interrupts) {
	fprintf(
	    stderr,
	    "Program saw % interrupts, which is incorrect\n",
	    int_total
	);
	error(EXIT_FAILURE, 0, "ERROR: Interrupt problem");
    }

    if (! DM6814.CloseBoard6814()) {
	error(EXIT_FAILURE, errno, "ERROR: CloseBoard6814() FAILED");
    }

    fprintf(stdout, "\n");
    fprintf(stdout, "Successful end of example program.\n");

    exit(EXIT_SUCCESS);
}
