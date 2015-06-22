/**
    @file

    @brief
        DM6814 library source code

    @verbatim
    --------------------------------------------------------------------------
    This file and its contents are copyright (C) RTD Embedded Technologies,
    Inc.  All Rights Reserved.

    This software is licensed as described in the RTD End-User Software License
    Agreement.  For a copy of this agreement, refer to the file LICENSE.TXT
    (which should be included with this software) or contact RTD Embedded
    Technologies, Inc.
    --------------------------------------------------------------------------
    @endverbatim

    $Id: librtd-dm6814.cpp 45950 2010-05-05 15:53:22Z wtate $
*/


#include <cstdio>
#include <errno.h>
#include <error.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <dm6814_library.h>
#include <dm6814_registers.h>
#include <dm6814_types.h>


/*
 * Indicator value for DM6814 device file descriptor when it is not open
 */

#define INVALID_HANDLE_VALUE -1


/*=============================================================================
Private member functions
 =============================================================================*/


bool
DM6814Device::inb(uint8_t offset, uint8_t *data_p) {
    dm6814_ioctl_argument_t	ioctl_request;

    ioctl_request.access_8.offset = offset;
    ioctl_request.access_8.data = 0;
    if (ioctl(hDevice, DM6814_IOCTL_READ_8_BITS, &ioctl_request) == -1) {
	fprintf(stderr, "Failed 8 bit read, I/O offset: %x\n", offset);
	perror("ioctl(DM6814_IOCTL_READ_8_BITS) FAILED");
	return false;
    }

    *data_p = ioctl_request.access_8.data;
    return true;
}


bool
DM6814Device::outb(uint8_t offset, uint8_t data) {
    dm6814_ioctl_argument_t	ioctl_request;

    ioctl_request.access_8.offset = offset;
    ioctl_request.access_8.data = data;
    if (ioctl(hDevice, DM6814_IOCTL_WRITE_8_BITS, &ioctl_request) == -1) {
	fprintf(
	    stderr,
	    "Failed 8 bit write, I/O offset: %x, data %x\n",
	    offset,
	    data
	);
	perror("ioctl(DM6814_IOCTL_WRITE_8_BITS) FAILED");
	return false;
    }

    return true;
}


int
DM6814Device::select_encoder_control_register(uint8_t offset, uint8_t select) {
    uint8_t	mode_value;

    if (! inb(offset, &mode_value)) {
	return -1;
    }

    /*
     * Clear bits 0 and 1 in Encoder Mode Register
     */

    mode_value &= 0xFC;

    mode_value |= select;

    if (! outb(offset, mode_value)) {
	return -1;
    }

    return 0;
}


int
DM6814Device::validate_8254_timer(uint8_t timer, uint8_t *offset_p) {
    switch (timer) {
	case 0:
	    *offset_p = TIMER_COUNTER_0;
	    break;

	case 1:
	    *offset_p = TIMER_COUNTER_1;
	    break;

	case 2:
	    *offset_p = TIMER_COUNTER_2;
	    break;

	default:
	    errno = EINVAL;
	    return -1;
	    break;
    }

    return 0;
}


int
DM6814Device::validate_incr_encoder(
    uint8_t encoder,
    incr_encoder_info_t *encoder_info_p
) {
    switch (encoder) {
	case 1:
	    encoder_info_p->lsb_offset = INCREMENTAL_ENCODER_1_LSB;
	    encoder_info_p->msb_offset = INCREMENTAL_ENCODER_1_MSB;
	    encoder_info_p->control_offset = INCREMENTAL_ENCODER_1_CLEAR;
	    encoder_info_p->mode_offset = INCREMENTAL_ENCODER_1_MODE;
	    break;

	case 2:
	    encoder_info_p->lsb_offset = INCREMENTAL_ENCODER_2_LSB;
	    encoder_info_p->msb_offset = INCREMENTAL_ENCODER_2_MSB;
	    encoder_info_p->control_offset = INCREMENTAL_ENCODER_2_CLEAR;
	    encoder_info_p->mode_offset = INCREMENTAL_ENCODER_2_MODE;
	    break;

	case 3:
	    encoder_info_p->lsb_offset = INCREMENTAL_ENCODER_3_LSB;
	    encoder_info_p->msb_offset = INCREMENTAL_ENCODER_3_MSB;
	    encoder_info_p->control_offset = INCREMENTAL_ENCODER_3_CLEAR;
	    encoder_info_p->mode_offset = INCREMENTAL_ENCODER_3_MODE;
	    break;

	default:
	    errno = EINVAL;
	    return -1;
	    break;
    }

    return 0;
}


/*=============================================================================
End of private member functions
 =============================================================================*/


/*=============================================================================
Public member functions
 =============================================================================*/


bool
DM6814Device::ClearEncoderChip6814(uint8_t Encoder) {
    incr_encoder_info_t	encoder_info;

    if (validate_incr_encoder(Encoder, &encoder_info) == -1) {
	return false;
    }

    if (
	select_encoder_control_register(
	    encoder_info.mode_offset, SELECT_CLEAR_REGISTER
	)
	==
	-1
    ) {
	return false;
    }

    return outb(encoder_info.control_offset, 0);
}


bool
DM6814Device::ClockDivisor6814(uint8_t Timer, uint16_t Divisor) {
    uint8_t	MSB, LSB;
    uint8_t	offset;

    if (validate_8254_timer(Timer, &offset) == -1) {
	return false;
    }

    LSB = Divisor & 0xff;
    MSB = (Divisor & 0xff00) >> 8;

    if (! outb(offset, LSB)) {
	return false;
    }

    return outb(offset, MSB);
}


bool
DM6814Device::ClockMode6814(uint8_t Timer, uint8_t Mode) {
    uint8_t	dummy;
    uint8_t	timer_control;

    if (validate_8254_timer(Timer, &dummy) == -1) {
	return false;
    }

    switch (Mode) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	    break;

	default:
	    errno = EINVAL;
	    return false;
	    break;
    }

    /*
     * Read/load LSB then MSB and set to binary
     */

    timer_control = 0x30;

    /*
     * Select the timer/counter
     */

    timer_control |= (Timer << 6);

    /*
     * Set counter mode
     */

    timer_control |= (Mode << 1);

    return outb(TIMER_COUNTER_CONTROL, timer_control);
}


bool
DM6814Device::CloseBoard6814(void) {
    if (hDevice != INVALID_HANDLE_VALUE) {
	if (close(hDevice) == -1) {
	    perror("close() FAILED");
	    hDevice = INVALID_HANDLE_VALUE;
	    return false;
	}
    }

    hDevice = INVALID_HANDLE_VALUE;

    return true;
}


DM6814Device::DM6814Device(void) :
    hDevice(INVALID_HANDLE_VALUE)
{
}


DM6814Device::~DM6814Device(void) {
    CloseBoard6814();
}


bool
DM6814Device::EnableEncoder6814(uint8_t Encoder, bool Enable) {
    incr_encoder_info_t		encoder_info;
    uint8_t			mode_reg;

    if (validate_incr_encoder(Encoder, &encoder_info) == -1) {
	return false;
    }

    if (! inb(encoder_info.mode_offset, &mode_reg)) {
	return false;
    }

    if (Enable) {
	mode_reg |= 0x60;
    } else {
	mode_reg &= ~0x60;
    }

    return outb(encoder_info.mode_offset, mode_reg);
}


bool
DM6814Device::EnableEncoderClear6814(uint8_t Encoder, bool Enable) {
    incr_encoder_info_t		encoder_info;
    uint8_t			mode_reg;

    if (validate_incr_encoder(Encoder, &encoder_info) == -1) {
	return false;
    }

    if (! inb(encoder_info.mode_offset, &mode_reg)) {
	return false;
    }

    if (Enable) {
	mode_reg |= 0x80;
    } else {
	mode_reg &= ~0x80;
    }

    return outb(encoder_info.mode_offset, mode_reg);
}


bool
DM6814Device::EnableEncoderIrq6814(uint8_t Encoder, bool Enable) {
    incr_encoder_info_t	encoder_info;
    uint8_t		mode_value;

    if (validate_incr_encoder(Encoder, &encoder_info) == -1) {
	return false;
    }

    if (! inb(encoder_info.mode_offset, &mode_value)) {
	return false;
    }

    if (Enable) {
	mode_value |= 0x10;
    } else {
	mode_value &= ~0x10;
    }

    return outb(encoder_info.mode_offset, mode_value);
}


bool
DM6814Device::EnableGetIntStatusWait6814(bool Enable) {
    int		file_flags;

    file_flags = fcntl(hDevice, F_GETFL);
    if (file_flags == -1) {
	return false;
    }

    if (Enable) {
	file_flags &= ~O_NONBLOCK;
    } else {
	file_flags |= O_NONBLOCK;
    }

    if (fcntl(hDevice, F_SETFL, file_flags) == -1) {
	return false;
    }

    return true;
}


bool
DM6814Device::GetDriverVersion6814(uint32_t *version_p) {
    dm6814_ioctl_argument_t	ioctl_request;

    if (ioctl(hDevice, DM6814_IOCTL_GET_DRIVER_VERSION, &ioctl_request) == -1) {
	perror("ioctl(DM6814_IOCTL_GET_DRIVER_VERSION) FAILED");
	return false;
    }

    *version_p = ioctl_request.version.driver_version;
    return true;
}


bool
DM6814Device::GetIntStatus6814(
    uint32_t	*int_count_p,
    uint8_t	*status_reg_p
) {
    dm6814_int_status_t		int_status;
    ssize_t			bytes_read;

    bytes_read = read(hDevice, &int_status, sizeof(dm6814_int_status_t));

    /*
     * Check read() error status
     */

    if (bytes_read == -1) {

	/*
	 * Some error occurred, let the application sort it out
	 */

	return false;
    }

    /*
     * No error occurred, check number of bytes that were read
     */

    if (bytes_read != sizeof(dm6814_int_status_t)) {

	/*
	 * We were expecting a number of bytes equal to the interrupt status
	 * structure size but did not.  Something is wrong with the driver.
	 */

	errno = EBADMSG;
	return false;
    }

    *int_count_p = int_status.int_count;
    *status_reg_p = int_status.status_reg;

    return true;
}


bool
DM6814Device::InitBoard6814(void) {
    uint8_t	encoder;
    uint8_t	irq_enable = 0x00;

    /*
     * P14 interrupt
     */

    DISABLE_IRQ_SHARING(irq_enable);
    POSITIVE_IRQ_POLARITY(irq_enable);
    DISABLE_P14_IRQ(irq_enable);

    if (! LoadIRQRegister6814(irq_enable)) {
	return false;
    }

    /*
     * Incremental encoders
     */

    for (encoder = 1; encoder < 4; encoder++) {
	if (! ClearEncoderChip6814(encoder)) {
	    return false;
	}

	if (! EnableEncoderIrq6814(encoder, false)) {
	    return false;
	}
    }

    return true;
}


bool
DM6814Device::LoadEncoder6814(uint8_t Encoder, uint16_t Value) {
    dm6814_ioctl_argument_t	ioctl_request;
    incr_encoder_info_t		encoder_info;

    if (validate_incr_encoder(Encoder, &encoder_info) == -1) {
	return false;
    }

    ioctl_request.access_16.data = Value;
    ioctl_request.access_16.offset = encoder_info.lsb_offset;

    if (ioctl(hDevice, DM6814_IOCTL_WRITE_16_BITS, &ioctl_request) == -1) {
	return false;
    }

    return true;
}


bool
DM6814Device::LoadIRQRegister6814(uint8_t Value) {
    return outb(IRQ_CONTROL, Value);
}


bool
DM6814Device::OpenBoard6814(uint32_t nDevice) {
    char devName[1024];

    (void) CloseBoard6814();
    snprintf(devName, sizeof(devName), "/dev/rtd-dm6814-%u", nDevice);
    hDevice = open(devName, (O_RDWR | O_NONBLOCK));
    if (hDevice == -1) {
	return false;
    }

    return true;
}


bool
DM6814Device::ReadByte6814(uint8_t offset, uint8_t *data_p) {
    dm6814_ioctl_argument_t	ioctl_request;
    
    ioctl_request.access_8.data = 0;
    ioctl_request.access_8.offset = offset;

    if (ioctl(hDevice, DM6814_IOCTL_READ_8_BITS, &ioctl_request) == -1) {
	return false;
    }

    *data_p = ioctl_request.access_8.data;
    return true;
}


bool
DM6814Device::ReadEncoder6814(uint8_t Encoder, uint16_t *value_p) {
    dm6814_ioctl_argument_t	ioctl_request;
    incr_encoder_info_t		encoder_info;

    if (validate_incr_encoder(Encoder, &encoder_info) == -1) {
	return false;
    }

    if (
	select_encoder_control_register(
	    encoder_info.mode_offset,
	    SELECT_HOLD_REGISTER
	)
	==
	-1
    ) {
	return false;
    }

    if (! outb(encoder_info.control_offset, 0x00)) {
	return false;
    }

    ioctl_request.access_16.data = 0;
    ioctl_request.access_16.offset = encoder_info.lsb_offset;

    if (ioctl(hDevice, DM6814_IOCTL_READ_16_BITS, &ioctl_request) == -1) {
	return false;
    }

    *value_p = ioctl_request.access_16.data;
    return true;
}


bool
DM6814Device::ReadEncoderDIO6814(uint8_t Encoder, uint8_t *value_p) {
    incr_encoder_info_t		encoder_info;
    uint8_t			value;

    if (validate_incr_encoder(Encoder, &encoder_info) == -1) {
	return false;
    }

    if (
	select_encoder_control_register(
	    encoder_info.mode_offset,
	    SELECT_DIO_REGISTER
	)
	==
	-1
    ) {
	return false;
    }

    if (! inb(encoder_info.control_offset, &value)) {
	return false;
    }

    *value_p = value;
    return true;
}


bool
DM6814Device::ReadTimerCounter6814(uint8_t Timer, uint16_t *count_p) {
    uint8_t	MSB, LSB;
    uint8_t	offset;

    if (validate_8254_timer(Timer, &offset) == -1) {
	return false;
    }

    /*
     * Because bits 4 and 5 in the 8254 Timer/Counter Control Word Register are
     * set to zero, this latches the counter value in a special register until
     * it is read below
     */

    if (! outb(TIMER_COUNTER_CONTROL, (Timer << 6))) {
	return false;
    }

    if (! inb(offset, &LSB)) {
	return false;
    }

    if (! inb(offset, &MSB)) {
	return false;
    }

    *count_p = ((256 * MSB) + LSB);
    return true;
}


bool
DM6814Device::SetEncoderDIODirection6814(
    uint8_t Encoder,
    bool Bit0Output,
    bool Bit1Output
) {
    incr_encoder_info_t		encoder_info;
    uint8_t			mode_reg;

    if (validate_incr_encoder(Encoder, &encoder_info) == -1) {
	return false;
    }

    if (! inb(encoder_info.mode_offset, &mode_reg)) {
	return false;
    }

    mode_reg &= 0xF3;

    if (Bit0Output) {
	mode_reg |= 0x04;
    }

    if (Bit1Output) {
	mode_reg |= 0x08;
    }

    return outb(encoder_info.mode_offset, mode_reg);
}


bool
DM6814Device::SetUserClock6814(
    uint8_t Timer,
    float InputRate,
    float OutputRate,
    float *actual_rate_p
) {
    if (! ClockMode6814(Timer, 2)) {
	return false;
    }

    if (! ClockDivisor6814(Timer, (uint16_t) (InputRate / OutputRate))) {
	return false;
    }

    *actual_rate_p = (InputRate / (int) (InputRate / OutputRate));
    return true;
}


bool
DM6814Device::WaitForInterrupt6814(void) {
    fd_set	exception_fds;
    fd_set	read_fds;
    int		status;

    /*
     * Set up the set of file descriptors that will be watched for input
     * activity.  Only the DM6814 device file descriptor is of interest.
     */

    FD_ZERO(&read_fds);
    FD_SET(hDevice, &read_fds);

    /*
     * Set up the set of file descriptors that will be watched for exception
     * activity.  Only the DM6814 device file descriptor is of interest.
     */

    FD_ZERO(&exception_fds);
    FD_SET(hDevice, &exception_fds);

    /*
     * Wait for the interrupt to happen.  No timeout is given, which means
     * the process will not be woken up until either an interrupt occurs or
     * a signal is delivered.
     */

    status = select((hDevice + 1), &read_fds, NULL, &exception_fds, NULL);

    /*
     * Check select() error status
     */

    if (status == -1) {

	/*
	 * Some error occurred, let the application sort it out
	 */

	return false;
    }

    /*
     * No error occurred and no timeout expired, therefore an event supposedly
     * happened.  Check the number of file descriptors select() says have data
     * available.
     */

    if (status == 0) {

	/*
	 * No file descriptors have data available.  This means something is
         * broken in the driver
	 */

	errno = ENODATA;
	return false;
    }

    /*
     * There is data available.  Check to see if an exception occurred
     */

    if (FD_ISSET(hDevice, &exception_fds)) {

	/*
	 * An exception occurred.  This means that no IRQ line was allocated to
	 * the device when the driver was loaded.
	 */

	errno = EIO;
	return false;
    }

    /*
     * At least one file descriptor has data available and no exception
     * occurred.  Check the device file descriptor to see if it is readable.
     */

    if (! FD_ISSET(hDevice, &read_fds)) {

	/*
	 * The device file is not readable.  This means something is broken in
         * the driver.
	 */

	errno = ENODATA;
	return false;
    }

    /*
     * An interrupt occurred
     */

    return true;
}


bool
DM6814Device::WriteByte6814(uint8_t offset, uint8_t data) {
    dm6814_ioctl_argument_t	ioctl_request;
    
    ioctl_request.access_8.data = data;
    ioctl_request.access_8.offset = offset;

    if (ioctl(hDevice, DM6814_IOCTL_WRITE_8_BITS, &ioctl_request) == -1) {
	return false;
    }

    return true;
}


bool
DM6814Device::WriteEncoderDIO6814(uint8_t Encoder, uint8_t Value) {
    incr_encoder_info_t		encoder_info;

    if (validate_incr_encoder(Encoder, &encoder_info) == -1) {
	return false;
    }

    if (
	select_encoder_control_register(
	    encoder_info.mode_offset,
	    SELECT_DIO_REGISTER
	)
	==
	-1
    ) {
	return false;
    }

    return outb(encoder_info.control_offset, Value);
}
