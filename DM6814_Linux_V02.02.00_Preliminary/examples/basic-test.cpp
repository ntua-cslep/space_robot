/*
	FILE NAME: basic-test.cpp

	FILE DESCRIPTION:

		Program which tests the basic functionality of the driver and
		library.

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
#include <dm6814_version.h>


#define MAX_DM6814_DEVICES	4

/*
 * Macros for driver version information
 */

static char *program_name_p;


static void
usage(void) {
    fprintf(stderr, "\n");
    fprintf(stderr, "%s\n", program_name_p);
    fprintf(stderr, "\n");
    fprintf(stderr, "Usage: %s\n", program_name_p);
    fprintf(stderr, "\n");
    exit(EXIT_FAILURE);
}


int
main(int argument_count, char **arguments_p_p) {
    DM6814Device	DM6814_Devices[MAX_DM6814_DEVICES];
    uint32_t		version;
    uint8_t		data;
    uint8_t		device_mask = 0x00;
    uint8_t		first_device = 0;
    uint8_t		major_version;
    uint8_t		minor_number;
    uint8_t		minor_version;
    uint8_t		patch_level;

    program_name_p = arguments_p_p[0];

    fprintf(stderr, "\n");
    fprintf(stderr, "\tDM6814 Basic Functionality Test\n");
    fprintf(stderr, "\n");

    /*
     * Process command line options
     */

    if (argument_count != 1) {
	fprintf(stderr, "ERROR: Invalid number of options given!\n");
	usage();
    }

    /*
     * First, figure out which actual devices of the possible ones are present
     * and create a bit mask representing them.  This bit mask is used in later
     * tests to indicate which boards are subject to testing.
     */

    fprintf(stdout, "## Determining status of all boards ...\n");

    for (minor_number = 0; minor_number < MAX_DM6814_DEVICES; minor_number++) {
	fprintf(stdout, "    Minor number %d\n", minor_number);
	if (DM6814_Devices[minor_number].OpenBoard6814(minor_number)) {
	    fprintf(stdout, "        Present\n");
	    device_mask |= (1 << minor_number);
	    (void) DM6814_Devices[minor_number].CloseBoard6814();
	} else {
	    fprintf(stdout, "        Not present\n");
	}
    }

    fprintf(stdout, "    Device mask: %x\n", device_mask);

    if (device_mask == 0x00) {
	error(EXIT_FAILURE, 0, "FAILURE: No boards present!");
    }

    /*
     * Verify that all boards which are present can be opened again
     */

    fprintf(stdout, "## Testing OpenBoard6814() ...\n");

    for (minor_number = 0; minor_number < MAX_DM6814_DEVICES; minor_number++) {
	fprintf(stdout, "    Minor number %d\n", minor_number);
	if ((1 << minor_number) & device_mask) {
	    if (! DM6814_Devices[minor_number].OpenBoard6814(minor_number)) {
		perror("OpenBoard6814() FAILED");
		exit(EXIT_FAILURE);
	    }
	} else {
	    fprintf(stdout, "        Not present, skipping.\n");
	}
    }

    /*
     * Verify that all boards which were opened can be closed
     */

    fprintf(stdout, "## Testing CloseBoard6814() ...\n");

    for (minor_number = 0; minor_number < MAX_DM6814_DEVICES; minor_number++) {
	fprintf(stdout, "    Minor number %d\n", minor_number);
	if ((1 << minor_number) & device_mask) {
	    if (! DM6814_Devices[minor_number].CloseBoard6814()) {
		perror("CloseBoard6814() FAILED");
		exit(EXIT_FAILURE);
	    }
	} else {
	    fprintf(stdout, "        Not present, skipping.\n");
	}
    }

    /*
     * Determine the lowest minor number for which a device is present.  This
     * minor number will be used for the rest of the tests herein.
     */

    for (minor_number = 0; minor_number < MAX_DM6814_DEVICES; minor_number++) {
	if ((1 << minor_number) & device_mask) {
	    first_device = minor_number;
	    break;
	}
    }

    if (! DM6814_Devices[first_device].OpenBoard6814(first_device)) {
	error(EXIT_FAILURE, errno, "OpenBoard6814() FAILED");
    }

    /*
     * Verify that we can get the driver version
     */

    fprintf(stdout, "## Testing GetDriverVersion6814() ...\n");

    if (! DM6814_Devices[first_device].GetDriverVersion6814(&version)) {
	error(EXIT_FAILURE, errno, "GetDriverVersion6814() FAILED");
    }

    major_version = ((version >> 16) & 0xF);
    minor_version = ((version >> 8) & 0xF);
    patch_level = (version & 0xF);

    fprintf(stdout, "        Major version: %d\n", major_version);
    fprintf(stdout, "        Minor version: %d\n", minor_version);
    fprintf(stdout, "        Patch level:   %d\n", patch_level);

    if (major_version != DRIVER_MAJOR_VERSION) {
	error(EXIT_FAILURE, 0, "ERROR: Major version incorrect");
    }

    if (minor_version != DRIVER_MINOR_VERSION) {
	error(EXIT_FAILURE, 0, "ERROR: Minor version incorrect");
    }

    if (patch_level != DRIVER_PATCH_LEVEL) {
	error(EXIT_FAILURE, 0, "ERROR: Patch level incorrect");
    }

    /*
     * Verify ReadByte6814() error checking
     */

    fprintf(stdout, "## Testing ReadByte6814() ...\n");

    fprintf(stdout, "    On invalid offset ...\n");

    if (DM6814_Devices[first_device].ReadByte6814(0x12, &data)) {
	error(EXIT_FAILURE, 0, "FAILURE: ReadByte6814() succeeded");
    }

    if (errno != EINVAL) {
	error(
	    EXIT_FAILURE,
	    errno,
	    "FAILURE: ReadByte6814() failed with wrong errno"
	);
    }

    fprintf(stdout, "    On valid, write-only offset  ...\n");

    if (DM6814_Devices[first_device].ReadByte6814(0x0F, &data)) {
	error(EXIT_FAILURE, 0, "FAILURE: ReadByte6814() succeeded");
    }

    if (errno != EOPNOTSUPP) {
	error(
	    EXIT_FAILURE,
	    errno,
	    "FAILURE: ReadByte6814() failed with wrong errno"
	);
    }

    /*
     * Verify WriteByte6814() error checking
     */

    fprintf(stdout, "## Testing WriteByte6814() ...\n");

    fprintf(stdout, "    On invalid offset ...\n");

    if (DM6814_Devices[first_device].WriteByte6814(0x12, data)) {
	error(EXIT_FAILURE, 0, "FAILURE: WriteByte6814() succeeded");
    }

    if (errno != EINVAL) {
	error(
	    EXIT_FAILURE,
	    errno,
	    "FAILURE: WriteByte6814() failed with wrong errno"
	);
    }

    fprintf(stdout, "    On valid, read-only offset  ...\n");

    if (DM6814_Devices[first_device].WriteByte6814(0x11, data)) {
	error(EXIT_FAILURE, 0, "FAILURE: WriteByte6814() succeeded");
    }

    if (errno != EOPNOTSUPP) {
	error(
	    EXIT_FAILURE,
	    errno,
	    "FAILURE: WriteByte6814() failed with wrong errno"
	);
    }

    if (! DM6814_Devices[first_device].CloseBoard6814()) {
	error(EXIT_FAILURE, errno, "CloseBoard6814() FAILED");
    }

    fprintf(stdout, "\n");
    fprintf(stdout, "Success.  All tests passed.\n");

    exit(EXIT_SUCCESS);
}
