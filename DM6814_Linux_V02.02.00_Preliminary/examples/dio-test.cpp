/*
	FILE NAME: dio-test.cpp

	FILE DESCRIPTION:

		Program which tests the library functions related to digital
		I/O.

		This program assumes that the DM6814 device being used has all
		jumpers in their factory default positions.

	PROJECT NAME: Linux DM6814 Driver, Library, and Example Programs

	PROJECT VERSION: (Defined in README.TXT)

	Copyright 2005 RTD Embedded Technologies, Inc.  All Rights Reserved.
*/


#include <errno.h>
#include <error.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <dm6814_library.h>
#include <dm6814_registers.h>


bool status;
static char *program_name_p;
uint8_t status_reg;
DM6814Device DM6814;


/*
 * The I/O space offsets for each incremental encoder's control register
 */

static uint8_t control_offsets[3] = {
    INCREMENTAL_ENCODER_1_CLEAR,
    INCREMENTAL_ENCODER_2_CLEAR,
    INCREMENTAL_ENCODER_3_CLEAR
};


/*
 * The I/O space offsets for each incremental encoder's value LSB
 */

static uint8_t lsb_offsets[3] = {
    INCREMENTAL_ENCODER_1_LSB,
    INCREMENTAL_ENCODER_2_LSB,
    INCREMENTAL_ENCODER_3_LSB
};


/*
 * The I/O space offsets for each incremental encoder's mode register
 */

static uint8_t mode_offsets[3] = {
    INCREMENTAL_ENCODER_1_MODE,
    INCREMENTAL_ENCODER_2_MODE,
    INCREMENTAL_ENCODER_3_MODE
};


/*
 * The I/O space offsets for each incremental encoder's value MSB
 */

static uint8_t msb_offsets[3] = {
    INCREMENTAL_ENCODER_1_MSB,
    INCREMENTAL_ENCODER_2_MSB,
    INCREMENTAL_ENCODER_3_MSB
};


static void
expect_success(bool status) {
    if (! status) {
	fprintf(stderr, "FAILED: Expected success but failure occurred.\n");
	fprintf(stderr, "    errno is %d\n", errno);
	exit(EXIT_FAILURE);
    }
}


static void
expect_failure_and_check(bool status, int expected_errno) {
    if (status) {
	fprintf(stderr, "FAILED: Expected failure but success occurred.\n");
	exit(EXIT_FAILURE);
    }

    if (errno != expected_errno) {
	fprintf(
	    stderr,
	    "FAILED: Expected errno %d, got %d.\n",
	    expected_errno,
	    errno
	);
	exit(EXIT_FAILURE);
    }
}


static void
read_encoder_value_lsb_then_msb(uint8_t encoder, uint16_t *encoder_value_p) {
    bool	status;
    uint8_t	encoder_lsb;
    uint8_t	encoder_msb;

    /*
     * Select Hold Register for encoder
     */

    status = DM6814.WriteByte6814(mode_offsets[(encoder - 1)], 0x01);
    expect_success(status);

    /*
     * Latch encoder value
     */

    status = DM6814.WriteByte6814(control_offsets[(encoder - 1)], 0x00);
    expect_success(status);

    /*
     * Read encoder LSB
     */

    status = DM6814.ReadByte6814(lsb_offsets[(encoder - 1)], &encoder_lsb);
    expect_success(status);

    /*
     * Read encoder MSB
     */

    status = DM6814.ReadByte6814(msb_offsets[(encoder - 1)], &encoder_msb);
    expect_success(status);

    *encoder_value_p = ((encoder_msb << 8) | encoder_lsb);
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
    uint16_t		encoder_value;
    uint32_t		minor_number;
    uint8_t		dio_value;
    uint8_t		encoder;
    uint8_t		mode_reg;

    program_name_p = arguments_p_p[0];

    fprintf(stderr, "\n");
    fprintf(stderr, "\tDM6814 Digital I/O Test\n");
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

    status = DM6814.OpenBoard6814(minor_number);
    expect_success(status);

    status = DM6814.InitBoard6814();
    expect_success(status);

    /*
     * Verify basic functionality of ClearEncoderChip6814()
     */

    fprintf(stdout, "## Testing ClearEncoderChip6814() ...\n");

    fprintf(stdout, "    On invalid encoder ...\n");

    status = DM6814.ClearEncoderChip6814(0);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On another invalid encoder ...\n");

    status = DM6814.ClearEncoderChip6814(4);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On all valid encoders ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ClearEncoderChip6814(encoder);
	expect_success(status);
    }

    /*
     * Verify basic functionality of EnableEncoder6814()
     */

    fprintf(stdout, "## Testing EnableEncoder6814() ...\n");

    fprintf(stdout, "    On invalid encoder ...\n");

    status = DM6814.EnableEncoder6814(0, false);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On another invalid encoder ...\n");

    status = DM6814.EnableEncoder6814(4, false);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On all valid encoders ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.EnableEncoder6814(encoder, false);
	expect_success(status);
    }

    /*
     * Verify basic functionality of EnableEncoderClear6814()
     */

    fprintf(stdout, "## Testing EnableEncoderClear6814() ...\n");

    fprintf(stdout, "    On invalid encoder ...\n");

    status = DM6814.EnableEncoderClear6814(0, false);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On another invalid encoder ...\n");

    status = DM6814.EnableEncoderClear6814(4, false);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On all valid encoders ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.EnableEncoderClear6814(encoder, false);
	expect_success(status);
    }

    /*
     * Verify basic functionality of EnableEncoderIrq6814()
     */

    fprintf(stdout, "## Testing EnableEncoderIrq6814() ...\n");

    fprintf(stdout, "    On invalid encoder ...\n");

    status = DM6814.EnableEncoderIrq6814(0, false);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On another invalid encoder ...\n");

    status = DM6814.EnableEncoderIrq6814(4, false);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On all valid encoders ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.EnableEncoderIrq6814(encoder, false);
	expect_success(status);
    }

    /*
     * Verify basic functionality of LoadEncoder6814()
     */

    fprintf(stdout, "## Testing LoadEncoder6814() ...\n");

    fprintf(stdout, "    On invalid encoder ...\n");

    status = DM6814.LoadEncoder6814(0, 0x0000);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On another invalid encoder ...\n");

    status = DM6814.LoadEncoder6814(4, 0x0000);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On all valid encoders ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.LoadEncoder6814(encoder, 0x0000);
	expect_success(status);
    }

    /*
     * Verify basic functionality of ReadEncoder6814()
     */

    fprintf(stdout, "## Testing ReadEncoder6814() ...\n");

    fprintf(stdout, "    On invalid encoder ...\n");

    status = DM6814.ReadEncoder6814(0, &encoder_value);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On another invalid encoder ...\n");

    status = DM6814.ReadEncoder6814(4, &encoder_value);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On all valid encoders ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadEncoder6814(encoder, &encoder_value);
	expect_success(status);
    }

    /*
     * Verify basic functionality of ReadEncoderDIO6814()
     */

    fprintf(stdout, "## Testing ReadEncoderDIO6814() ...\n");

    fprintf(stdout, "    On invalid encoder ...\n");

    status = DM6814.ReadEncoderDIO6814(0, &dio_value);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On another invalid encoder ...\n");

    status = DM6814.ReadEncoderDIO6814(4, &dio_value);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On all valid encoders ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadEncoderDIO6814(encoder, &dio_value);
	expect_success(status);
    }

    /*
     * Verify basic functionality of SetEncoderDIODirection6814()
     */

    fprintf(stdout, "## Testing SetEncoderDIODirection6814() ...\n");

    fprintf(stdout, "    On invalid encoder ...\n");

    status = DM6814.SetEncoderDIODirection6814(0, false, false);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On another invalid encoder ...\n");

    status = DM6814.SetEncoderDIODirection6814(4, false, false);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On all valid encoders ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.SetEncoderDIODirection6814(encoder, false, false);
	expect_success(status);
    }

    /*
     * Verify basic functionality of WriteEncoderDIO6814()
     */

    fprintf(stdout, "## Testing WriteEncoderDIO6814() ...\n");

    fprintf(stdout, "    On invalid encoder ...\n");

    status = DM6814.WriteEncoderDIO6814(0, 0x00);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On another invalid encoder ...\n");

    status = DM6814.WriteEncoderDIO6814(4, 0x00);
    expect_failure_and_check(status, EINVAL);

    fprintf(stdout, "    On all valid encoders ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.WriteEncoderDIO6814(encoder, 0x00);
	expect_success(status);
    }

    /*=========================================================================
    Verify board state set by ClearEncoderChip6814()
     =========================================================================*/

    fprintf(stdout, "## Testing ClearEncoderChip6814() ...\n");

    /*
     * Set board to a known state
     */

    fprintf(stdout, "    Set board to a known state ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	fprintf(stdout, "            Set port bits 0/1 to output\n");

	status = DM6814.SetEncoderDIODirection6814(encoder, true, true);
	expect_success(status);

	fprintf(stdout, "            Set encoder value ...\n");

	status = DM6814.LoadEncoder6814(encoder, (encoder * 1000));
	expect_success(status);
    }

    fprintf(stdout, "    Verify board state ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	fprintf(stdout, "            Port bits 0/1 direction\n");

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if ((mode_reg & 0x0C) != 0x0C) {
	    error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	}

	fprintf(stdout, "            Encoder value\n");

	status = DM6814.ReadEncoder6814(encoder, &encoder_value);
	expect_success(status);

	if (encoder_value != (encoder * 1000)) {
	    error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	}
    }

    /*
     * Clear the incremental encoder 1 chip and then verify that port 0 bits 0
     * and 1 are set to input and that port 2 and 4 bits 0 and 1 are set to
     * output
     */

    fprintf(stdout, "    Clear incremental encoder 1 ...\n");

    status = DM6814.ClearEncoderChip6814(1);
    expect_success(status);

    fprintf(stdout, "    Verify board state ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	fprintf(stdout, "            Port bits 0/1 direction\n");

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if (encoder == 1) {
	    if ((mode_reg & 0x0C) != 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	    }
	} else {
	    if ((mode_reg & 0x0C) != 0x0C) {
		error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	    }
	}

	fprintf(stdout, "            Encoder value\n");

	status = DM6814.ReadEncoder6814(encoder, &encoder_value);
	expect_success(status);

	if (encoder == 1) {
	    if (encoder_value != 0x0000) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	    }
	} else {
	    if (encoder_value != (encoder * 1000)) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	    }
	}
    }

    /*
     * Clear the incremental encoder 2 chip and then verify that port 0 and 2
     * bits 0 and 1 are set to input and that port 4 bits 0 and 1 are set to
     * output
     */

    fprintf(stdout, "    Clear incremental encoder 2 ...\n");

    status = DM6814.ClearEncoderChip6814(2);
    expect_success(status);

    fprintf(stdout, "    Verify board state ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	fprintf(stdout, "            Port bits 0/1 direction\n");

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if (encoder != 3) {
	    if ((mode_reg & 0x0C) != 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	    }
	} else {
	    if ((mode_reg & 0x0C) != 0x0C) {
		error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	    }
	}

	fprintf(stdout, "            Encoder value\n");

	status = DM6814.ReadEncoder6814(encoder, &encoder_value);
	expect_success(status);

	if (encoder != 3) {
	    if (encoder_value != 0x0000) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	    }
	} else {
	    if (encoder_value != (encoder * 1000)) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	    }
	}
    }

    /*
     * Clear the incremental encoder 3 chip and then verify that port 0, 2, and
     * 4 bits 0 and 1 are set to input
     */

    fprintf(stdout, "    Clear incremental encoder 3 ...\n");

    status = DM6814.ClearEncoderChip6814(3);
    expect_success(status);

    fprintf(stdout, "    Verify board state ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	fprintf(stdout, "            Port bits 0/1 direction\n");

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if ((mode_reg & 0x0C) != 0x00) {
	    error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	}

	fprintf(stdout, "            Encoder value\n");

	status = DM6814.ReadEncoder6814(encoder, &encoder_value);
	expect_success(status);

	if (encoder_value != 0x0000) {
	    error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	}
    }

    /*=========================================================================
    Verify board state set by EnableEncoder6814()
     =========================================================================*/

    fprintf(stdout, "## Testing EnableEncoder6814() ...\n");

    /*
     * Set board to a known state
     */

    fprintf(stdout, "    Disable encoder inputs ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.EnableEncoder6814(encoder, false);
	expect_success(status);
    }

    fprintf(stdout, "    Verify all encoder inputs disabled ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if ((mode_reg & 0x60) != 0x00) {
	    error(EXIT_FAILURE, 0, "ERROR: Encoder input state incorrect");
	}
    }

    /*
     * Enable incremental encoder 1 input and verify that encoder 1 input is
     * enabled and encoder 2 and 3 inputs are disabled
     */

    fprintf(stdout, "    Enable encoder 1 input ...\n");

    status = DM6814.EnableEncoder6814(1, true);
    expect_success(status);

    fprintf(stdout, "    Verify encoder input states ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if (encoder == 1) {
	    if ((mode_reg & 0x60) != 0x60) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder input state incorrect");
	    }
	} else {
	    if ((mode_reg & 0x60) != 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder input state incorrect");
	    }
	}
    }

    /*
     * Enable incremental encoder 2 input and verify that encoder 1 and 2 inputs
     * are enabled and encoder 3 input is disabled
     */

    fprintf(stdout, "    Enable encoder 2 input ...\n");

    status = DM6814.EnableEncoder6814(2, true);
    expect_success(status);

    fprintf(stdout, "    Verify encoder input states ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if (encoder != 3) {
	    if ((mode_reg & 0x60) != 0x60) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder input state incorrect");
	    }
	} else {
	    if ((mode_reg & 0x60) != 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder input state incorrect");
	    }
	}
    }

    /*
     * Enable incremental encoder 3 input and verify that encoder 1,2, and 3
     * inputs are enabled
     */

    fprintf(stdout, "    Enable encoder 3 input ...\n");

    status = DM6814.EnableEncoder6814(3, true);
    expect_success(status);

    fprintf(stdout, "    Verify encoder input states ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if ((mode_reg & 0x60) != 0x60) {
	    error(EXIT_FAILURE, 0, "ERROR: Encoder input state incorrect");
	}
    }

    /*
     * Disable the encoder inputs to prevent side effects
     */

    fprintf(stdout, "    Disable encoder inputs ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.EnableEncoder6814(encoder, false);
	expect_success(status);
    }

    /*=========================================================================
    Verify board state set by EnableEncoderClear6814()
     =========================================================================*/

    fprintf(stdout, "## Testing EnableEncoderClear6814() ...\n");

    /*
     * Set board to a known state
     */

    fprintf(stdout, "    Disable encoder clears ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.EnableEncoderClear6814(encoder, false);
	expect_success(status);
    }

    fprintf(stdout, "    Verify all encoder clears disabled ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if ((mode_reg & 0x80) != 0x00) {
	    error(EXIT_FAILURE, 0, "ERROR: Encoder clear state incorrect");
	}
    }

    /*
     * Enable incremental encoder 1 clear and verify that encoder clear is
     * enabled on encoder 1 and disabled on encoders 2 and 3
     */

    fprintf(stdout, "    Enable encoder 1 clear ...\n");

    status = DM6814.EnableEncoderClear6814(1, true);
    expect_success(status);

    fprintf(stdout, "    Verify encoder clear states ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if (encoder == 1) {
	    if ((mode_reg & 0x80) != 0x80) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder clear state incorrect");
	    }
	} else {
	    if ((mode_reg & 0x80) != 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder clear state incorrect");
	    }
	}
    }

    /*
     * Enable incremental encoder 2 clear and verify that encoder clear is
     * enabled on encoders 1 and 2 and disabled on encoder and 3
     */

    fprintf(stdout, "    Enable encoder 2 clear ...\n");

    status = DM6814.EnableEncoderClear6814(2, true);
    expect_success(status);

    fprintf(stdout, "    Verify encoder clear states ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if (encoder != 3) {
	    if ((mode_reg & 0x80) != 0x80) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder clear state incorrect");
	    }
	} else {
	    if ((mode_reg & 0x80) != 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder clear state incorrect");
	    }
	}
    }

    /*
     * Enable incremental encoder 3 clear and verify that encoder clear is
     * enabled on encoders 1, 2, and 3
     */

    fprintf(stdout, "    Enable encoder 3 clear ...\n");

    status = DM6814.EnableEncoderClear6814(3, true);
    expect_success(status);

    fprintf(stdout, "    Verify encoder clear states ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if ((mode_reg & 0x80) != 0x80) {
	    error(EXIT_FAILURE, 0, "ERROR: Encoder clear state incorrect");
	}
    }

    /*
     * Disable the encoder clears to prevent side effects
     */

    fprintf(stdout, "    Disable encoder clears ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.EnableEncoderClear6814(encoder, false);
	expect_success(status);
    }

    /*=========================================================================
    Verify board state set by EnableEncoderIrq6814()
     =========================================================================*/

    fprintf(stdout, "## Testing EnableEncoderIrq6814() ...\n");

    /*
     * Set board to a known state
     */

    fprintf(stdout, "    Disable encoder interrupts ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.EnableEncoderIrq6814(encoder, false);
	expect_success(status);
    }

    fprintf(stdout, "    Verify all encoder interrupts disabled ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if ((mode_reg & 0x10) != 0x00) {
	    error(EXIT_FAILURE, 0, "ERROR: Encoder interrupt state incorrect");
	}
    }

    /*
     * Enable incremental encoder 1 interrupts and verify that interrupts are
     * enabled on encoder 1 and disabled on encoders 2 and 3
     */

    fprintf(stdout, "    Enable encoder 1 interrupts ...\n");

    status = DM6814.EnableEncoderIrq6814(1, true);
    expect_success(status);

    fprintf(stdout, "    Verify encoder interrupt states ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if (encoder == 1) {
	    if ((mode_reg & 0x10) != 0x10) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder clear state incorrect");
	    }
	} else {
	    if ((mode_reg & 0x10) != 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder clear state incorrect");
	    }
	}
    }

    /*
     * Enable incremental encoder 2 interrupts and verify that interrupts are
     * enabled on encoders 1 and 2 and disabled on encoder 3
     */

    fprintf(stdout, "    Enable encoder 2 interrupts ...\n");

    status = DM6814.EnableEncoderIrq6814(2, true);
    expect_success(status);

    fprintf(stdout, "    Verify encoder interrupt states ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if (encoder != 3) {
	    if ((mode_reg & 0x10) != 0x10) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder clear state incorrect");
	    }
	} else {
	    if ((mode_reg & 0x10) != 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder clear state incorrect");
	    }
	}
    }

    /*
     * Enable incremental encoder 3 interrupts and verify that interrupts are
     * enabled on encoders 1, 2, and 3
     */

    fprintf(stdout, "    Enable encoder 3 interrupts ...\n");

    status = DM6814.EnableEncoderIrq6814(3, true);
    expect_success(status);

    fprintf(stdout, "    Verify encoder interrupt states ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if ((mode_reg & 0x10) != 0x10) {
	    error(EXIT_FAILURE, 0, "ERROR: Encoder clear state incorrect");
	}
    }

    /*
     * Disable encoder interrupts to prevent side effects
     */

    fprintf(stdout, "    Disable encoder interrupts ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.EnableEncoderIrq6814(encoder, false);
	expect_success(status);
    }

    /*=========================================================================
    Verify board state set by LoadEncoder6814()

    For this test, the encoders will eventually be loaded with a value derived
    from the formula: value = ((encoder + 1) << 8) | encoder)
     =========================================================================*/

    fprintf(stdout, "## Testing LoadEncoder6814() ...\n");

    /*
     * Set board to a known state by clearing all encoder chips which also
     * clears encoder values
     */

    fprintf(stdout, "    Clear encoder chips ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ClearEncoderChip6814(encoder);
	expect_success(status);
    }

    fprintf(stdout, "    Verify all encoder values cleared ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	read_encoder_value_lsb_then_msb(encoder, &encoder_value);

	if (encoder_value != 0x0000) {
	    error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	}
    }

    /*
     * Load incremental encoder 1 value and verify that only encoder 1's value
     * changed
     */

    fprintf(stdout, "    Load incremental encoder 1 value ...\n");

    status = DM6814.LoadEncoder6814(1, ((2 << 8) | 1));
    expect_success(status);

    fprintf(stdout, "    Verify all encoder values ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	read_encoder_value_lsb_then_msb(encoder, &encoder_value);

	if (encoder == 1) {
	    if (encoder_value != ((2 << 8) | 1)) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	    }
	} else {
	    if (encoder_value != 0x0000) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	    }
	}
    }

    /*
     * Load incremental encoder 2 value and verify that only encoder 2's value
     * changed
     */

    fprintf(stdout, "    Load incremental encoder 2 value ...\n");

    status = DM6814.LoadEncoder6814(2, ((3 << 8) | 2));
    expect_success(status);

    fprintf(stdout, "    Verify all encoder values ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	read_encoder_value_lsb_then_msb(encoder, &encoder_value);

	if (encoder != 3) {
	    if (encoder_value != (((encoder + 1) << 8) | encoder)) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	    }
	} else {
	    if (encoder_value != 0x0000) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	    }
	}
    }

    /*
     * Load incremental encoder 3 value and verify that only encoder 3's value
     * changed
     */

    fprintf(stdout, "    Load incremental encoder 3 value ...\n");

    status = DM6814.LoadEncoder6814(3, ((4 << 8) | 3));
    expect_success(status);

    fprintf(stdout, "    Verify all encoder values ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	read_encoder_value_lsb_then_msb(encoder, &encoder_value);

	if (encoder_value != (((encoder + 1) << 8) | encoder)) {
	    error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	}
    }

    /*
     * Clear all encoder chips to prevent side effects
     */

    fprintf(stdout, "    Clear encoder chips ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ClearEncoderChip6814(encoder);
	expect_success(status);
    }

    /*=========================================================================
    Verify board access by ReadEncoder6814()

    For this test, the encoders will eventually be loaded with a value derived
    from the formula: value = ((encoder + 1) << 8) | encoder)
     =========================================================================*/

    fprintf(stdout, "## Testing ReadEncoder6814() ...\n");

    /*
     * Set board to a known state by clearing all encoder chips which also
     * clears encoder values
     */

    fprintf(stdout, "    Clear encoder chips ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ClearEncoderChip6814(encoder);
	expect_success(status);
    }

    fprintf(stdout, "    Verify all encoder values cleared ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadEncoder6814(encoder, &encoder_value);
	expect_success(status);

	if (encoder_value != 0x0000) {
	    error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	}
    }

    /*
     * Load incremental encoder 1 value and verify each encoder's value
     */

    fprintf(stdout, "    Load incremental encoder 1 value ...\n");

    status = DM6814.LoadEncoder6814(1, ((2 << 8) | 1));
    expect_success(status);

    fprintf(stdout, "    Verify all encoder values ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadEncoder6814(encoder, &encoder_value);
	expect_success(status);

	if (encoder == 1) {
	    if (encoder_value != ((2 << 8) | 1)) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	    }
	} else {
	    if (encoder_value != 0x0000) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	    }
	}
    }

    /*
     * Load incremental encoder 2 value and verify each encoder's value
     */

    fprintf(stdout, "    Load incremental encoder 2 value ...\n");

    status = DM6814.LoadEncoder6814(2, ((3 << 8) | 2));
    expect_success(status);

    fprintf(stdout, "    Verify all encoder values ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadEncoder6814(encoder, &encoder_value);
	expect_success(status);

	if (encoder != 3) {
	    if (encoder_value != (((encoder + 1) << 8) | encoder)) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	    }
	} else {
	    if (encoder_value != 0x0000) {
		error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	    }
	}
    }

    /*
     * Load incremental encoder 3 value and verify each encoder's value
     */

    fprintf(stdout, "    Load incremental encoder 3 value ...\n");

    status = DM6814.LoadEncoder6814(3, ((4 << 8) | 3));
    expect_success(status);

    fprintf(stdout, "    Verify all encoder values ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadEncoder6814(encoder, &encoder_value);
	expect_success(status);

	if (encoder_value != (((encoder + 1) << 8) | encoder)) {
	    error(EXIT_FAILURE, 0, "ERROR: Encoder value incorrect");
	}
    }

    /*
     * Clear all encoder chips to prevent side effects
     */

    fprintf(stdout, "    Clear encoder chips ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ClearEncoderChip6814(encoder);
	expect_success(status);
    }

    /*=========================================================================
    Verify board state set by SetEncoderDIODirection6814()
     =========================================================================*/

    fprintf(stdout, "## Testing SetEncoderDIODirection6814() ...\n");

    /*
     * Set board to a known state by setting port bits 0 and 1 to input
     */

    fprintf(stdout, "    Set port bits 0/1 to input ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.SetEncoderDIODirection6814(encoder, false, false);
	expect_success(status);
    }

    fprintf(stdout, "    Verify all bits set to input ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if ((mode_reg & 0x0C) != 0x00) {
	    error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	}
    }

    /*
     * Set incremental encoder 1 bit 0 to output and bit 1 to input and verify
     * bit directions for all encoders
     */

    fprintf(
	stdout, "    Set encoder 1 bit 0 to output and bit 1 to input ...\n"
    );

    status = DM6814.SetEncoderDIODirection6814(1, true, false);
    expect_success(status);

    fprintf(stdout, "    Verify bit directions ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if (encoder == 1) {
	    if ((mode_reg & 0x0C) != 0x04) {
		error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	    }
	} else {
	    if ((mode_reg & 0x0C) != 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	    }
	}
    }

    /*
     * Set incremental encoder 2 bit 0 to input and bit 1 to output and verify
     * bit directions for all encoders
     */

    fprintf(
	stdout, "    Set encoder 2 bit 0 to input and bit 1 to output ...\n"
    );

    status = DM6814.SetEncoderDIODirection6814(2, false, true);
    expect_success(status);

    fprintf(stdout, "    Verify bit directions ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if (encoder == 1) {
	    if ((mode_reg & 0x0C) != 0x04) {
		error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	    }
	} else if (encoder == 2) {
	    if ((mode_reg & 0x0C) != 0x08) {
		error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	    }
	} else {
	    if ((mode_reg & 0x0C) != 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	    }
	}
    }

    /*
     * Set incremental encoder 3 bits 0 and 1 to output and verify bit
     * directions for all encoders
     */

    fprintf(
	stdout, "    Set encoder 3 bits 0 and 1 to output ...\n"
    );

    status = DM6814.SetEncoderDIODirection6814(3, true, true);
    expect_success(status);

    fprintf(stdout, "    Verify bit directions ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	status = DM6814.ReadByte6814(mode_offsets[(encoder - 1)], &mode_reg);
	expect_success(status);

	if (encoder == 1) {
	    if ((mode_reg & 0x0C) != 0x04) {
		error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	    }
	} else if (encoder == 2) {
	    if ((mode_reg & 0x0C) != 0x08) {
		error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	    }
	} else {
	    if ((mode_reg & 0x0C) != 0x0C) {
		error(EXIT_FAILURE, 0, "ERROR: Bit direction incorrect");
	    }
	}
    }

    /*=========================================================================
    Verify that ReadEncoderDIO6814() and WriteEncoderDIO6814() operate on the
    proper ports
     =========================================================================*/

    fprintf(
	stdout,
	"## Testing ReadEncoderDIO6814()/WriteEncoderDIO6814() port access "
	"...\n"
    );

    /*
     * Set board into a known state
     */

    fprintf(stdout, "    Set board into a known state ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	fprintf(stdout, "        Encoder %d\n", encoder);

	fprintf(stdout, "            Clear encoder chip\n");

	status = DM6814.ClearEncoderChip6814(encoder);
	expect_success(status);

	fprintf(stdout, "            Disable encoder interrupt\n");

	status = DM6814.EnableEncoderIrq6814(encoder, false);
	expect_success(status);

	fprintf(stdout, "            Disable encoder\n");

	status = DM6814.EnableEncoder6814(encoder, false);
	expect_success(status);

	fprintf(stdout, "            Disable encoder clear\n");

	status = DM6814.EnableEncoderClear6814(encoder, false);
	expect_success(status);
    }

    fprintf(stdout, "    Checking port access ...\n");

    for (encoder = 1; encoder < 4; encoder++) {
	uint8_t	input_encoder;
	uint8_t	output_encoder;

	output_encoder = encoder;
	input_encoder = ((output_encoder + 1) % 4);
	if (input_encoder == 0) {
	    input_encoder = 1;
	}

	fprintf(
	    stdout,
	    "        Encoders [output/input]: %1u/%1u\n",
	    output_encoder,
	    input_encoder
	);

	fprintf(stdout, "        Set output encoder direction\n");

	status = DM6814.SetEncoderDIODirection6814(output_encoder, true, true);
	expect_success(status);

	fprintf(stdout, "        Set input encoder direction\n");

	status = DM6814.SetEncoderDIODirection6814(input_encoder, false, false);
	expect_success(status);

	fprintf(stdout, "\n");
	fprintf(
	    stdout,
	    "        Connect port %1u to port %1u.\n",
	    ((output_encoder - 1) * 2),
	    ((input_encoder - 1) * 2)
	);

	fprintf(stdout, "        Press ENTER to start test:");
	(void) fgetc(stdin);

	/*
	 * Write the output encoder number to the output port and make sure it
	 * was read on the input port
	 */

	status = DM6814.WriteEncoderDIO6814(output_encoder, output_encoder);
	expect_success(status);

	status = DM6814.ReadEncoderDIO6814(input_encoder, &dio_value);
	expect_success(status);

	if ((dio_value & 0x03) != output_encoder) {
	    fprintf(
		stderr,
		"Read %x from input, expecting %x\n",
		(dio_value & 0x03),
		output_encoder
	    );
	    error(EXIT_FAILURE, 0, "ERROR: Wrong value read from input port");
	}

	/*
	 * Write the input encoder number to the output port and make sure it
	 * was read on the input port
	 */

	status = DM6814.WriteEncoderDIO6814(output_encoder, input_encoder);
	expect_success(status);

	status = DM6814.ReadEncoderDIO6814(input_encoder, &dio_value);
	expect_success(status);

	if ((dio_value & 0x03) != input_encoder) {
	    fprintf(
		stderr,
		"Read %x from input, expecting %x\n",
		(dio_value & 0x03),
		input_encoder
	    );
	    error(EXIT_FAILURE, 0, "ERROR: Wrong value read from input port");
	}
    }

    status = DM6814.CloseBoard6814();
    expect_success(status);

    fprintf(stdout, "\n");
    fprintf(stdout, "Success.  All tests passed.\n");

    exit(EXIT_SUCCESS);
}
