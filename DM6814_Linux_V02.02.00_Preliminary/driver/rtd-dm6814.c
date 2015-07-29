/**
    @file

    @brief
        DM6814 driver source code

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

    $Id: rtd-dm6814.c 62901 2012-09-11 20:35:52Z rgroner $
*/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/ioport.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/stat.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/wait.h>

#include <dm6814_driver.h>
#include <dm6814_registers.h>
#include <dm6814_types.h>
#include <dm6814_version.h>

/*=============================================================================
Differences between 2.4 and 2.6 kernels
 =============================================================================*/

/*
 * Interrupt-specific information (including declarations for free_irq() and
 * request_irq() functions) moved to new header file in 2.6 kernel
 */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))

#include <linux/interrupt.h>
#include <linux/cdev.h>

static struct cdev dm6814_cdev;

#endif

/*
 * Module reference counting is handled automatically in 2.6 kernel but must be
 * done manually in 2.4 kernel.  Set up preprocessor magic to hide this
 * difference.
 */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0))

#define INCREMENT_MODULE_USAGE	MOD_INC_USE_COUNT
#define DECREMENT_MODULE_USAGE	MOD_DEC_USE_COUNT

#else

#define INCREMENT_MODULE_USAGE
#define DECREMENT_MODULE_USAGE

#endif

/*
 * 2.6 kernel interrupt handlers return a value indicating whether or not the
 * interrupt was able to be processed.  2.4 kernel interrupt handlers do not
 * return a value.  Set up preprocessor magic to hide this difference.
 */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0))

#define INTERRUPT_HANDLER_TYPE	static void
#define INTERRUPT_HANDLED
#define INTERRUPT_NOT_HANDLED
typedef void (*dm6814_handler_t) (int, void *, struct pt_regs *);

#else
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
#define INTERRUPT_HANDLER_TYPE	static irqreturn_t
#define INTERRUPT_HANDLED	IRQ_HANDLED
#define INTERRUPT_NOT_HANDLED	IRQ_NONE
typedef irqreturn_t(*dm6814_handler_t) (int, void *, struct pt_regs *);
#else
#define INTERRUPT_HANDLER_TYPE	static irqreturn_t
#define INTERRUPT_HANDLED	IRQ_HANDLED
#define INTERRUPT_NOT_HANDLED	IRQ_NONE
typedef irqreturn_t(*dm6814_handler_t) (int, void *);
#endif
#endif

/*
 * 2.6 kernel by default exports no module symbols whereas 2.4 kernel by default
 * exports module symbols.  Set up preprocessor magic to hide this difference.
 */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0))

#define DONT_EXPORT_SYMBOLS	EXPORT_NO_SYMBOLS

#else

#define DONT_EXPORT_SYMBOLS

#endif

/*=============================================================================
Driver documentation
 =============================================================================*/

#define DRIVER_NAME "rtd-dm6814"

MODULE_AUTHOR(RTD_COPYRIGHT_STRING);
MODULE_DESCRIPTION(VERSION_STRING);
MODULE_LICENSE("Proprietary");

/*=============================================================================
Module parameters
 =============================================================================*/

/*
 * Base I/O addresses
 */

static uint16_t io[DM6814_MAX_DEVS];

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
module_param_array(io, ushort, NULL, S_IRUSR | S_IRGRP | S_IROTH);
#else
MODULE_PARM(io, "1-" __MODULE_STRING(DM6814_MAX_DEVS) "h");
#endif
MODULE_PARM_DESC(io, "I/O port base address");
/*
 * IRQ line numbers
 */

static uint8_t irq[DM6814_MAX_DEVS];
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0))
module_param_array(irq, byte, NULL, S_IRUSR | S_IRGRP | S_IROTH);
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
module_param_array(irq, int, NULL, S_IRUSR | S_IRGRP | S_IROTH);
#else
MODULE_PARM(irq, "1-" __MODULE_STRING(DM6814_MAX_DEVS) "b");
#endif
#endif
MODULE_PARM_DESC(irq, "IRQ line");

/*=============================================================================
Global variables
 =============================================================================*/

/*
 * Character device major number; dynamically assigned
 */

static int dm6814_major;

/*
 * DM6814 device descriptors
 */

static struct DM6814Device dm6814_devices[DM6814_MAX_DEVS];

/*=============================================================================
Forward declarations
 =============================================================================*/

static struct file_operations dm6814_file_ops;

/*=============================================================================
Driver functions
 =============================================================================*/

/******************************************************************************
@brief
    Initialize the state of all device interrupts.  This consists of 1)
    disabling P14 interrupts, 2) setting positive edge P14 interrupt
    polarity, 3) disabling interrupt sharing, 4) clearing all incremental
    encoder chips, and 5) disabling all incremental encoder interrupts.

@param
    device_p

    Address of device descriptor for device.

 ******************************************************************************/

static void dm6814_init_interrupts(struct DM6814Device *device_p)
{

    /*=========================================================================
    P14 interrupt initialization
     =========================================================================*/

	/*
	 * Set up IRQ Enable Register.  Bit 0 is cleared which disables the P14
	 * interrupt.  Bit 1 is cleared which sets positive IRQ polarity.  Bit 2 is
	 * set which disables IRQ sharing.
	 */

	outb_p(0x04, (device_p->io + IRQ_CONTROL));

	/*
	 * Clear P14 interrupt by reading Clear IRQ Register
	 */

	(void)inb_p(device_p->io + IRQ_CONTROL);

    /*=========================================================================
    Incremental encoder 1 initialization
     =========================================================================*/

	/*
	 * Set up Incremental Encoder 1 Chip Mode Register.  Bits 0 and 1 are
	 * cleared which selects clear mode for addressing base I/O address + 2.
	 * Bit 2 is cleared which sets P0.0 to input.  Bit 3 is cleared which sets
	 * P0.1 to input.  Bit 4 is cleared which disabled the encoder interrupt.
	 * Bits 5 and 6 are cleared which disables the encoder input.  Bit 7 is
	 * cleared which disables counter clear.
	 */

	outb_p(0x00, (device_p->io + INCREMENTAL_ENCODER_1_MODE));

	/*
	 * Clear the encoder chip which also clears the encoder interrupt status
	 * flag.  The value written does not matter.
	 */

	outb_p(0x00, (device_p->io + INCREMENTAL_ENCODER_1_CLEAR));

    /*=========================================================================
    Incremental encoder 2 initialization
     =========================================================================*/

	/*
	 * Set up Incremental Encoder 2 Chip Mode Register.  Bits 0 and 1 are
	 * cleared which selects clear mode for addressing base I/O address + 6.
	 * Bit 2 is cleared which sets P2.0 to input.  Bit 3 is cleared which sets
	 * P2.1 to input.  Bit 4 is cleared which disables the encoder interrupt.
	 * Bits 5 and 6 are cleared which disables the encoder input.  Bit 7 is
	 * cleared which disables counter clear.
	 */

	outb_p(0x00, (device_p->io + INCREMENTAL_ENCODER_2_MODE));

	/*
	 * Clear the encoder chip which also clears the encoder interrupt status
	 * flag.  The value written does not matter.
	 */

	outb_p(0x00, (device_p->io + INCREMENTAL_ENCODER_2_CLEAR));

    /*=========================================================================
    Incremental encoder 3 initialization
     =========================================================================*/

	/*
	 * Set up Incremental Encoder 3 Chip Mode Register.  Bits 0 and 1 are
	 * cleared which selects clear mode for addressing base I/O address + 10.
	 * Bit 2 is cleared which sets P4.0 to input.  Bit 3 is cleared which sets
	 * P4.1 to input.  Bit 4 is cleared which disables the encoder interrupt.
	 * Bits 5 and 6 are cleared which disables the encoder input.  Bit 7 is
	 * cleared which disables counter clear.
	 */

	outb_p(0x00, (device_p->io + INCREMENTAL_ENCODER_3_MODE));

	/*
	 * Clear the encoder chip which also clears the encoder interrupt status
	 * flag.  The value written does not matter.
	 */

	outb_p(0x00, (device_p->io + INCREMENTAL_ENCODER_3_CLEAR));
}

/******************************************************************************
@brief
    Encode and return the driver's major & minor version numbers and the
    patch level number.

@param
    ioctl_param

    Third parameter given on ioctl() call.  This is the user space address of
    the structure used to pass in the arguments.

@retval
    0

    Success.

@retval
     < 0
     Failure.@n@n
     The following values may be returned:
     @arg \c
        -EFAULT     ioctl_param is not a valid user address.
 ******************************************************************************/

static int dm6814_get_driver_version(unsigned long ioctl_param)
{
	dm6814_ioctl_argument_t ioctl_argument;

	ioctl_argument.version.driver_version = ((DRIVER_MAJOR_VERSION << 16)
						 | (DRIVER_MINOR_VERSION << 8)
						 | DRIVER_PATCH_LEVEL);

	if (copy_to_user((dm6814_ioctl_argument_t *) ioctl_param,
			 &ioctl_argument, sizeof(dm6814_ioctl_argument_t)
	    )
	    ) {
		return -EFAULT;
	}

	return 0;
}

/******************************************************************************
@brief
    Write an unsigned 16-bit value into a device's I/O port space.

@param
    device_p

    Address of device descriptor for device.

@param
    ioctl_param

    Third parameter given on ioctl() call.  This is the user space address of
    the structure used to pass in the arguments.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
    @arg \c
        -EFAULT     ioctl_param is not a valid user address.
    @arg \c
        -EINVAL     The I/O port offset is not valid.
 ******************************************************************************/

static int
dm6814_write_16_bits(struct DM6814Device *device_p, unsigned long ioctl_param)
{
	dm6814_ioctl_argument_t ioctl_argument;

	if (copy_from_user(&ioctl_argument,
			   (dm6814_ioctl_argument_t *) ioctl_param,
			   sizeof(dm6814_ioctl_argument_t)
	    )
	    ) {
		return -EFAULT;
	}

	switch (ioctl_argument.access_16.offset) {
	case 0:
	case 4:
	case 8:
		break;

	default:
		return -EINVAL;
		break;
	}

	outw_p(ioctl_argument.access_16.data,
	       (device_p->io + ioctl_argument.access_16.offset)
	    );

	return 0;
}

/******************************************************************************
@brief
    Write an unsigned 8-bit value into a device's I/O port space.

@param
    device_p

    Address of device descriptor for device.

@param
    ioctl_param

    Third parameter given on ioctl() call.  This is the user space address of
    the structure used to pass in the arguments.

@retval
    0

    Success.

@param
    < 0

    Failure.@n@n
    The following values may be returned:
    @arg \c
        -EFAULT     ioctl_param is not a valid user address.
    @arg \c
        -EINVAL     The I/O port offset is not valid.
    @arg \c
        -EOPNOTSUPP The I/O port offset is valid but it represents a read-only
                    register.
 ******************************************************************************/

static int
dm6814_write_8_bits(struct DM6814Device *device_p, unsigned long ioctl_param)
{
	dm6814_ioctl_argument_t ioctl_argument;

	if (copy_from_user(&ioctl_argument,
			   (dm6814_ioctl_argument_t *) ioctl_param,
			   sizeof(dm6814_ioctl_argument_t)
	    )
	    ) {
		return -EFAULT;
	}

	if (ioctl_argument.access_8.offset > 17) {
		return -EINVAL;
	}

	if (ioctl_argument.access_8.offset == 17) {
		return -EOPNOTSUPP;
	}

	outb_p(ioctl_argument.access_8.data,
	       (device_p->io + ioctl_argument.access_8.offset)
	    );

	return 0;
}

/******************************************************************************
@brief
    Read an unsigned 16-bit value from a device's I/O port space.

@param
    device_p

    Address of device descriptor for device.

    ioctl_param

    Third parameter given on ioctl() call.  This is the user space address of
    the structure used to pass in the arguments.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
    @arg \c
        -EFAULT     ioctl_param is not a valid user address.
    @arg \c
        -EINVAL     The I/O port offset is not valid.
 ******************************************************************************/

static int
dm6814_read_16_bits(struct DM6814Device *device_p, unsigned long ioctl_param)
{
	dm6814_ioctl_argument_t ioctl_argument;

	if (copy_from_user(&ioctl_argument,
			   (dm6814_ioctl_argument_t *) ioctl_param,
			   sizeof(dm6814_ioctl_argument_t)
	    )
	    ) {
		return -EFAULT;
	}

	switch (ioctl_argument.access_16.offset) {
	case 0:
	case 4:
	case 8:
		break;

	default:
		return -EINVAL;
		break;
	}

	ioctl_argument.access_16.data =
	    inw_p(device_p->io + ioctl_argument.access_16.offset);

	if (copy_to_user((dm6814_ioctl_argument_t *) ioctl_param,
			 &ioctl_argument, sizeof(dm6814_ioctl_argument_t)
	    )
	    ) {
		return -EFAULT;
	}

	return 0;
}

/******************************************************************************
@brief
    Read an unsigned 8-bit value from a device's I/O port space.

@param
    device_p

    Address of device descriptor for device.

@param
    ioctl_param

    Third parameter given on ioctl() call.  This is the user space address of
    the structure used to pass in the arguments.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
    @arg \c
        -EFAULT     ioctl_param is not a valid user address.
    @arg \c
        -EINVAL     The I/O port offset is not valid.
    @arg \c
        -EOPNOTSUPP The I/O port offset is valid but it represents a write-only
                    register.
 ******************************************************************************/

static int
dm6814_read_8_bits(struct DM6814Device *device_p, unsigned long ioctl_param)
{
	dm6814_ioctl_argument_t ioctl_argument;

	if (copy_from_user(&ioctl_argument,
			   (dm6814_ioctl_argument_t *) ioctl_param,
			   sizeof(dm6814_ioctl_argument_t)
	    )
	    ) {
		return -EFAULT;
	}

	if (ioctl_argument.access_8.offset > 17) {
		return -EINVAL;
	}

	if (ioctl_argument.access_8.offset == 15) {
		return -EOPNOTSUPP;
	}

	ioctl_argument.access_8.data =
	    inb_p(device_p->io + ioctl_argument.access_8.offset);

	if (copy_to_user((dm6814_ioctl_argument_t *) ioctl_param,
			 &ioctl_argument, sizeof(dm6814_ioctl_argument_t)
	    )
	    ) {
		return -EFAULT;
	}

	return 0;
}

/******************************************************************************
@brief
    Prepare an DM6814 device file to be opened and used.

@param
    inode_p

    Address of kernel's inode descriptor for the device file.

@param
    file_p

    Address of kernel's file descriptor for the device file.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
    @arg \c
        -EBUSY      The device file is already open.
    @arg \c
        -ENODEV     The device's minor number is not valid
    @arg \c
        -ENODEV     The device failed its configuration, the device is not
                    present, or the device was not configured.

@note
    This function acquires and releases the DM6814 device's spin lock.
 ******************************************************************************/

static int dm6814_open(struct inode *inode_p, struct file *file_p)
{
	struct DM6814Device *device_p;
	uint32_t minor_number;

	minor_number = MINOR(inode_p->i_rdev);

	if (minor_number >= DM6814_MAX_DEVS) {
		return -ENODEV;
	}

	device_p = &(dm6814_devices[minor_number]);

	if (!device_p->io) {
		return -ENODEV;
	}

	if (device_p->reference_count) {
		return -EBUSY;
	}

	file_p->private_data = device_p;
	device_p->reference_count++;
	device_p->status_reg = 0x00;
	device_p->int_count = 0;
	INCREMENT_MODULE_USAGE;

	dm6814_init_interrupts(device_p);

	return 0;
}

/******************************************************************************
@brief
    Do all processing necessary after the last reference to an DM6814
    device file is released elsewhere in the kernel.

@param
    inode_p

    Address of kernel's inode descriptor for the device file. Unused.

@param
    file_p

    Address of kernel's file descriptor for the device file.

@retval
    0

    Success.  This function always returns zero.

@note
    This function is not necessarily invoked each time a user process calls
    close() on a device file.  When a file structure is shared, for example
    after a call to fork() or dup(), the release method is not called until
    the last reference is released.

@note
    This function acquires and releases the DM6814 device's spin lock.
 ******************************************************************************/

static int dm6814_release(struct inode *inode_p, struct file *file_p)
{
	struct DM6814Device *device_p;

	device_p = (struct DM6814Device *)file_p->private_data;

	dm6814_init_interrupts(device_p);

	spin_lock(&(device_p->lock));

	device_p->reference_count--;
	file_p->private_data = 0;
	DECREMENT_MODULE_USAGE;

	spin_unlock(&(device_p->lock));

	return 0;
}

/******************************************************************************
@brief
    Process ioctl(2) system calls directed toward an DM6814 device file.

@param
    inode_p

    Address of kernel's inode descriptor for the device file.  Unused.

@param
    file_p

    Address of kernel's file descriptor for the device file.

@param
    request_code

    The service being requested.

@param
    ioctl_param

    Third parameter given on ioctl() call.  Depending upon request_code,
    ioctl_param may or may not be used.  Also based upon request code,
    ioctl_param may be an actual value or may be an address.  If the third
    parameter is not given on the ioctl() call, then ioctl_param has some
    undefined value.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
    @arg \c
        -EBADFD     File structure's private data pointer is NULL but it should
                    be the address of the device descriptor for the device
    @arg \c
        -EINVAL     request_code is not valid.

@note
        Please see the descriptions of dm6814_get_driver_version(),
        dm6814_write_8_bits(), and dm6814_read_8_bits() for information on
        other possible values returned in this case.

@note
    This function may indirectly acquire and release the DM6814 device's
    spin lock.  Also, local processor interrupts may indirectly be disabled
    by this function.
 ******************************************************************************/

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static int
dm6814_ioctl(struct inode *inode_p,
	     struct file *file_p,
	     unsigned int request_code, unsigned long ioctl_param)
#else
static long
dm6814_ioctl(struct file *file_p,
	     unsigned int request_code, unsigned long ioctl_param)
#endif
{
	int status;
	struct DM6814Device *device_p = file_p->private_data;

	if (!device_p) {
		return -EBADFD;
	}

	switch (request_code) {
	case DM6814_IOCTL_GET_DRIVER_VERSION:{
			status = dm6814_get_driver_version(ioctl_param);
			break;
		}

	case DM6814_IOCTL_WRITE_8_BITS:
		status = dm6814_write_8_bits(device_p, ioctl_param);
		break;

	case DM6814_IOCTL_READ_8_BITS:
		status = dm6814_read_8_bits(device_p, ioctl_param);
		break;

	case DM6814_IOCTL_WRITE_16_BITS:
		status = dm6814_write_16_bits(device_p, ioctl_param);
		break;

	case DM6814_IOCTL_READ_16_BITS:
		status = dm6814_read_16_bits(device_p, ioctl_param);
		break;

	default:
		status = -EINVAL;
		break;
	}

	return status;
}

/******************************************************************************
@brief
    Determine whether or not a DM6814 device is readable.  This function
    supports the poll(2) and select(2) system calls.

@param
    file_p

    Address of kernel's file descriptor for the device file.

@param
    poll_table_p

    Address of kernel's poll table descriptor.  This keeps track of all event
    queues on which the process can wait.

@retval
    Bit mask describing the status of the device.  The mask assumes one of
    of the following states:
        * the POLLPRI bit will be set if the device has no allocated IRQ
        * the POLLIN and POLLRDNORM bits will be set if a cached Interrupt
          Status Register is available
        * no bits are set if none of the above two conditions are met

@note
    A DM6814 device is readable if and only if an interrupt just occurred
    on the device and a process has not yet obtained the interrupt status
    from it.

@note
    This function is used in the process of waiting until an interrupt
    occurs on a device.

@note
    This function can be executed before an interrupt occurs, which happens
    if something sends a signal to the process.

@note
    This function disables interrupts for the given device and then
    acquires the device's spin lock to read interrupt status maintained by
    the interrupt handler.  Once that information is obtained, the spin
    lock is released and interrupts are enabled for the device.  This time
    interval is short, on the order of a few instructions.
 ******************************************************************************/

static unsigned int
dm6814_poll(struct file *file_p, struct poll_table_struct *poll_table_p)
{
	struct DM6814Device *device_p = file_p->private_data;
	uint8_t int_status;
	unsigned int status_mask = 0;

	/*
	 * If no IRQ line was allocated to the device when the driver was loaded,
	 * no status is available
	 */

	if (device_p->irq == 0) {

		/*
		 * This value causes select(2) to indicate that a file descriptor is
		 * present in its file descriptor sets but it will be in the exception
		 * set rather than in the input set.  The user library will look for
		 * this exception and return EIO.
		 */

		return POLLPRI;
	}

	/*
	 * Register with the file system layer so that it can wait on and check for
	 * DM6814 events
	 */

	poll_wait(file_p, &(device_p->int_wait_queue), poll_table_p);

    /*=========================================================================
     Waiting is done interruptibly, which means that a signal could have been
     delivered.  Thus we might have been woken up by a signal before an
     interrupt occurred.  Therefore, the process needs to examine the device's
     cached IRQ Status Register value.
     =========================================================================*/

	/*
	 * Disable interrupts on this DM6814 device because the interrupt routine
	 * modifies this information and we want to avoid a race condition.  The
	 * critical section is small, which minimizes interrupt interference.  This
	 * particular call waits for any active interrupt to complete which means
	 * we can't be holding any resource the handler needs or we deadlock.
	 * Therefore disable the interrupt then grab the spin lock.
	 */

	disable_irq(device_p->irq);

	/*
	 * Multiprocessor protection
	 */

	spin_lock(&(device_p->lock));

	/*
	 * Make a local copy of the cached IRQ Status Register value
	 */

	int_status = device_p->status_reg;

	/*
	 * Unlock and enable this device's interrupts once again
	 */

	spin_unlock(&(device_p->lock));
	enable_irq(device_p->irq);

    /*=========================================================================
     Interpret cached IRQ Status Register value
     =========================================================================*/

	/*
	 * The cached value is cleared after reading.  See if it is clear or not.
	 */

	if (int_status != 0) {

		/*
		 * Not clear, therefore an interrupt occurred since value was read last
		 */

		status_mask = (POLLIN | POLLRDNORM);
	}

	return status_mask;
}

/******************************************************************************
@brief
    Read data from a DM6814 device.

@param
    file_p

    Address of kernel's file descriptor for the device file.

@param
    buffer_p

    Address of user space buffer in which data will be stored.

@param
    byte_count

    Number of bytes to read.  This MUST be equal to sizeof(dm6814_int_status_t).

@param
    offset_p

    Address of memory containing the offset within the file being accessed.
    Most read operations would update this offset, e.g. when reading from a
    normal file.  However, this offset has no meaning for a DM6814 device and it
    is not updated.

@retval
    sizeof(dm6814_int_status)

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
    @arg \c
        -EFAULT     buffer_p is not a valid user address.
    @arg \c
        -EINVAL     byte_count is not equal to sizeof(dm6814_int_status_t).
    @arg \c
        -EIO        No IRQ line was allocated to the device when the driver was
                    loaded.
    @arg \c
        -ERESTARTSYS    A signal was delivered to the process while it slept.

@note
    The data available to be read from a DM6814 device is the interrupt
    status information kept by the interrupt handler.

@note
    This function disables interrupts for the given device and then
    acquires the device's spin lock to read interrupt status maintained by
    the interrupt handler.  Once that information is obtained, the spin
    lock is released and interrupts are enabled for the device.  This time
    interval is short, on the order of a few instructions.

@note
    If the device file is set to be accessed using blocking I/O, i.e.
    O_NONBLOCK is not set for the file, this function allows the calling
    process to wait for interrupt occurrence and return status in an atomic
    operation.

@note
    A DM6814 device file is readable in the following situations: 1) the
    cached IRQ Status Register value is non-zero regardless of using
    blocking or non-blocking I/O, or 2) non-blocking I/O is used regardless
    of the cached IRQ Status Register value.  The latter represents a
    departure from usual read(2) behavior in that read(2) should return
    EAGAIN when using non-blocking I/O and no data is available.  Since
    read(2) is the only way to retrieve interrupt status information,
    EAGAIN cannot be returned from here.  An application may want to get
    status before interrupts are enabled, usually when busy-waiting for an
    interrupt to occur.  A DM6814 device file is not readable if the cached
    IRQ Status Register value is zero and blocking I/O is used.
 ******************************************************************************/

static ssize_t
dm6814_read(struct file *file_p,
	    char *buffer_p, size_t byte_count, loff_t * offset_p)
{
	dm6814_int_status_t interrupt_status;
	ssize_t return_value = 0;
	struct DM6814Device *device_p = file_p->private_data;
	uint8_t status_available;
	wait_queue_t status_wait;

	/*
	 * If no IRQ line was allocated to device when driver was loaded, no
	 * interrupt status is available
	 */

	if (device_p->irq == 0) {
		return -EIO;
	}

	/*
	 * Make sure byte count to read agrees with size of information to be
	 * returned
	 */

	if (byte_count != sizeof(dm6814_int_status_t)) {
		return -EINVAL;
	}

	/*
	 * Initialize status wait queue entry and add it to the device's interrupt
	 * wait queue.  This prepares the process for going to sleep waiting for an
	 * interrupt.  This is done whether or not the process will ultimately
	 * sleep to avoid a race condition where an interrupt may be missed if it
	 * occurs after reading cached IRQ Status Register value and before the
	 * process inserts itself on the interrupt wait queue.
	 */

	init_waitqueue_entry(&status_wait, current);
	add_wait_queue(&(device_p->int_wait_queue), &status_wait);

	while (1) {

		/*
		 * Set process' state to indicate to rest of system that it is asleep;
		 * more preparation for putting the process to sleep.  This is done
		 * whether or not the process will ultimately sleep to avoid a race
		 * condition where an interrupt may be missed if it occurs after
		 * reading cached IRQ Status Register value and before the process
		 * inserts itself on the interrupt wait queue.
		 */

		set_current_state(TASK_INTERRUPTIBLE);

	/*=====================================================================
	Grab the driver's interrupt count and cached IRQ Status Register value
	 =====================================================================*/

		/*
		 * Disable interrupts on this DM6814 device because the interrupt
		 * routine modifies this information and we want to avoid a race
		 * condition.  The critical section is small, which minimizes interrupt
		 * interference.  This particular call waits for any active interrupt
		 * to complete which means we can't be holding any resource the handler
		 * needs or we deadlock.  Therefore disable the interrupt then grab the
		 * spin lock.
		 */

		disable_irq(device_p->irq);

		/*
		 * Multiprocessor protection
		 */

		spin_lock(&(device_p->lock));

		/*
		 * Make local copies of interrupt count and cached IRQ Status Register
		 */

		interrupt_status.status_reg = device_p->status_reg;
		interrupt_status.int_count = device_p->int_count;

		/*
		 * Clear cached IRQ Status Register value so that dm6814_poll() knows
		 * when there is status available.  This must be done in the critical
		 * section.
		 */

		device_p->status_reg = 0x00;

		/*
		 * Unlock and enable this device's interrupts once again
		 */

		spin_unlock(&(device_p->lock));
		enable_irq(device_p->irq);

	/*=====================================================================
	Local copies of interrupt status obtained, so examine the status
	 =====================================================================*/

		/*
		 * Is IRQ Status Register value available?
		 */

		if (interrupt_status.status_reg != 0x00) {

			/*
			 * Cached IRQ Status Register value is not cleared, so status is
			 * available
			 */

			status_available = 0xFF;
			break;
		}

		/*
		 * An interrupt has not occurred since the last time interrupt status
		 * was read.  Theoretically, no data is available to read.
		 */

		/*
		 * Normally, using non-blocking I/O would mean that we return -EAGAIN
		 * here.  However, there is no other way to get interrupt status and
		 * therefore we must return the driver's interrupt count and cached IRQ
		 * Status Register value whatever they may be when using non-blocking
		 * I/O.  Although we could look at this another way, namely that the
		 * driver's interrupt count is always available.  Non-blocking I/O is
		 * used when: 1) busy-waiting for interrupts to occur, 2) when getting
		 * an initial interrupt status before enabling interrupts and waiting
		 * for them to occur, or 3) when getting a final interrupt status after
		 * disabling interrupts and all processing is complete.
		 */

		if (file_p->f_flags & O_NONBLOCK) {
			status_available = 0xFF;
			break;
		}

		/*
		 * At this point, no status is available and the process wants blocking
		 * I/O
		 */

		status_available = 0x00;

		/*
		 * Is there a signal pending for the process?
		 */

		if (signal_pending(current)) {

			/*
			 * The process has a signal pending
			 */

			/*
			 * Inform file system layer that a signal is pending and let it
			 * decide what to do about it
			 */

			return_value = -ERESTARTSYS;
			break;
		}

		/*
		 * Switch this process away from the CPU, thus finally putting it to
		 * sleep
		 */

		schedule();
	}

	/*
	 * Was there status available?
	 */

	if (status_available) {

		/*
		 * Interrupt status available
		 */

		/*
		 * Copy status information back to user space
		 */

		if (copy_to_user
		    (buffer_p, &interrupt_status, sizeof(dm6814_int_status_t)
		    )
		    == 0) {
			return_value = sizeof(dm6814_int_status_t);
		} else {
			return_value = -EFAULT;
		}
	}

	/*
	 * Mark process as runnable again which undoes previously setting the state
	 * to sleeping
	 */

	set_current_state(TASK_RUNNING);

	/*
	 * Remove entry from device's interrupt wait queue since process is either
	 * not going to sleep or has already slept
	 */

	remove_wait_queue(&(device_p->int_wait_queue), &status_wait);

	return return_value;
}

/******************************************************************************
@brief
    Interrupt handler for DM6814 devices.

@param
    irq_number

    Interrupt number.

@param
    dev_id

    Address of device's DM6814 device descriptor.  This is set on request_irq()
    call.

@param
    registers_p

    Address of processor context at time of interrupt. Unused.

@retval
    On 2.4 kernels, this function does not return a value.

@retval
    On 2.6 kernels, this function returns a value as follows:
    @arg \c
        IRQ_HANDLED     Interrupt successfully processed.
    @arg \c
        IRQ_NONE        Interrupt could not be processed.
 ******************************************************************************/

INTERRUPT_HANDLER_TYPE
dm6814_interrupt(int irq_number, void *dev_id, struct pt_regs * registers_p)
{
	struct DM6814Device *device_p = (struct DM6814Device *)dev_id;
	uint8_t encoder_1_int;
	uint8_t encoder_2_int;
	uint8_t encoder_3_int;
	uint8_t p14_int;
	uint8_t status_reg;

	if (device_p == NULL) {
		printk(KERN_ERR
		       "%s> ERROR: Interrupt handler has NULL device pointer\n",
		       DRIVER_NAME);

		return INTERRUPT_NOT_HANDLED;
	}

	/*
	 * Multiprocessor protection.  Local processor interrupts are not disabled.
	 */

	spin_lock(&(device_p->lock));

	/*
	 * Read IRQ Status Register and see if any interrupts occurred
	 */

	status_reg = (inb_p(device_p->io + IRQ_STATUS) & 0x0F);
	p14_int = (status_reg & (1 << 3));
	encoder_1_int = (status_reg & (1 << 0));
	encoder_2_int = (status_reg & (1 << 1));
	encoder_3_int = (status_reg & (1 << 2));

	if ((p14_int == 0)
	    && (encoder_1_int == 0)
	    && (encoder_2_int == 0)
	    && (encoder_3_int == 0)
	    ) {
		printk(KERN_ERR "%s-%ld> ERROR: Spurious interrupt\n",
		       DRIVER_NAME, (device_p - &(dm6814_devices[0]))
		    );

		spin_unlock(&(device_p->lock));

		return INTERRUPT_NOT_HANDLED;
	}

    /*=========================================================================
     Count the interrupt and cache the IRQ Status Register contents
     =========================================================================*/

	device_p->int_count++;
	device_p->status_reg = status_reg;

    /*=========================================================================
     Now determine which interrupts occurred
     =========================================================================*/

	/*
	 * Examine P14 interrupt status
	 */

	if (p14_int) {

		/*
		 * P14 interrupt occurred, so acknowledge it by reading Clear IRQ
		 * Register at base I/O adddress + 16
		 */

		(void)inb_p(device_p->io + IRQ_CONTROL);
	}

	/*
	 * Examine incremental encoder 3 digital interrupt status
	 */

	if (encoder_3_int) {
		uint8_t port_status;

		/*
		 * Digital interrupt occurred on encoder 3, so acknowledge it
		 */

		/*
		 * Select Incremental Encoder 3 Clear Register at base I/O address + 10
		 */

		port_status = inb_p(device_p->io + INCREMENTAL_ENCODER_3_MODE);
		port_status &= 0xFC;
		outb_p(port_status,
		       (device_p->io + INCREMENTAL_ENCODER_3_MODE));

		/*
		 * Clear the interrupt
		 */

		(void)inb_p(device_p->io + INCREMENTAL_ENCODER_3_CLEAR);
	}

	/*
	 * Examine incremental encoder 2 digital interrupt status
	 */

	if (encoder_2_int) {
		uint8_t port_status;

		/*
		 * Digital interrupt occurred on encoder 2, so acknowledge it
		 */

		/*
		 * Select Incremental Encoder 2 Clear Register at base I/O address + 6
		 */

		port_status = inb_p(device_p->io + INCREMENTAL_ENCODER_2_MODE);
		port_status &= 0xFC;
		outb_p(port_status,
		       (device_p->io + INCREMENTAL_ENCODER_2_MODE));

		/*
		 * Clear the interrupt
		 */

		(void)inb_p(device_p->io + INCREMENTAL_ENCODER_2_CLEAR);
	}

	/*
	 * Examine incremental encoder 1 digital interrupt status
	 */

	if (encoder_1_int) {
		uint8_t port_status;

		/*
		 * Digital interrupt occurred on encoder 2, so acknowledge it
		 */

		/*
		 * Select Incremental Encoder 1 Clear Register at base I/O address + 2
		 */

		port_status = inb_p(device_p->io + INCREMENTAL_ENCODER_1_MODE);
		port_status &= 0xFC;
		outb_p(port_status,
		       (device_p->io + INCREMENTAL_ENCODER_1_MODE));

		/*
		 * Clear the interrupt
		 */

		(void)inb_p(device_p->io + INCREMENTAL_ENCODER_1_CLEAR);
	}

	/*
	 * All done, release the multiprocessor protection lock
	 */

	spin_unlock(&(device_p->lock));

	/*
	 * Wake up any process waiting for an interrupt to occur
	 */

	wake_up_interruptible(&(device_p->int_wait_queue));

	return INTERRUPT_HANDLED;
}

/******************************************************************************
@brief
    Validate any parameters that may have been passed in via insmod or modprobe.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
    @arg \c
        -EINVAL     A base I/O address is not valid
    @arg \c
        -EINVAL     An IRQ line number is not valid.
 ******************************************************************************/

static int dm6814_validate_mod_params(void)
{
	uint8_t minor_num;

	for (minor_num = 0; minor_num < DM6814_MAX_DEVS; minor_num++) {

		/*
		 * Validate base I/O address.  A base I/O address of zero is valid and
		 * means either no device is present or it is to be ignored.
		 */

		switch (io[minor_num]) {
		case 0x000:
		case 0x200:
		case 0x220:
		case 0x240:
		case 0x260:
		case 0x280:
		case 0x2A0:
		case 0x2C0:
		case 0x2E0:
		case 0x300:
		case 0x320:
		case 0x340:
		case 0x360:
		case 0x380:
		case 0x3A0:
		case 0x3C0:
		case 0x3E0:
			break;

		default:
			printk(KERN_ERR
			       "%s> ERROR: Invalid base I/O address 0x%x!\n",
			       DRIVER_NAME, io[minor_num]
			    );
			return -EINVAL;
			break;
		}

		/*
		 * Validate IRQ.  An IRQ of zero is valid and means that the interrupt
		 * functionality will not be used.
		 */

		switch (irq[minor_num]) {
		case 0:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 10:
		case 11:
		case 12:
		case 14:
		case 15:
			break;

		default:
			printk(KERN_ERR "%s> ERROR: Invalid IRQ %d!\n",
			       DRIVER_NAME, irq[minor_num]
			    );
			return -EINVAL;
			break;
		}
	}

	return 0;
}

/******************************************************************************
@brief
    Determine whether or not all devices are unconfigured.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
    @arg \c
        -ENODEV     No devices are configured.
 ******************************************************************************/

static int dm6814_check_all_unconfigured(void)
{
	int minor_num;

	for (minor_num = 0; minor_num < DM6814_MAX_DEVS; minor_num++) {

		/*
		 * If at least one device is configured, load the driver
		 */

		if (io[minor_num] != 0) {
			return 0;
		}
	}

	/*
	 * No devices configured, refuse to load the driver
	 */

	printk(KERN_ERR "%s> ERROR: No devices configured!\n", DRIVER_NAME);

	return -ENODEV;
}

/******************************************************************************
@brief
    Release any resources allocated by the driver.

@note
    This function is called both at module unload time and when the driver is
    cleaning up after some error occurred.
 ******************************************************************************/

static void dm6814_release_resources(void)
{
	uint8_t minor_num;

	for (minor_num = 0; minor_num < DM6814_MAX_DEVS; minor_num++) {
		struct DM6814Device *device_p;

		device_p = &(dm6814_devices[minor_num]);

		if (device_p->io != 0) {
			release_region(device_p->io, DM6814_IO_EXTENT);
			device_p->io = 0;
		}

		if (device_p->irq) {
			free_irq(device_p->irq, device_p);
			device_p->irq = 0;
		}
	}
}

/******************************************************************************
@brief
    Allocate device resources.

@param
    device_p

    Address of device descriptor for device.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
    @arg \c
        -EBUSY      The interrupt line requested is being used by another
                    device; returned by request_irq().
    @arg \c
        -EINVAL     The interrupt line requested is not valid; returned by
                    request_irq().
    @arg \c
        -EINVAL     No interrupt handler is to be associated with the requested
                    interrupt line; returned by request_irq().
    @arg \c
        -ENXIO      I/O port allocation failed.
    @arg \c
        -ENOMEM     Memory for interrupt action descriptor could not be
                    allocated; returned by request_irq().

@note
    This function is called at module load time.

@note
    The resources allocated are an I/O port range and possibly an IRQ line.
 ******************************************************************************/

static int dm6814_allocate_resources(struct DM6814Device *device_p)
{
	int status;
	uint8_t minor_num;

	minor_num = (device_p - &(dm6814_devices[0]));

	if (request_region
	    (io[minor_num], DM6814_IO_EXTENT, device_p->device_name)
	    == NULL) {
		printk(KERN_ERR
		       "%s-%1u> ERROR: I/O port range %#x-%#x allocation FAILED!\n",
		       DRIVER_NAME,
		       minor_num,
		       io[minor_num], (io[minor_num] + DM6814_IO_EXTENT - 1)
		    );
		dm6814_release_resources();
		return -ENXIO;
	}

	/*
	 * I/O port allocation succeeded, save I/O address in device structure so
	 * that dm6814_release_resources() can deallocate it
	 */

	device_p->io = io[minor_num];

	if (irq[minor_num] == 0) {
		printk(KERN_INFO "%s-%1u> WARNING: IRQ disabled.\n",
		       DRIVER_NAME, minor_num);
		return 0;
	}

	status = request_irq(irq[minor_num],
			     (dm6814_handler_t) dm6814_interrupt,
			     0, device_p->device_name, device_p);
	if (status != 0) {
		printk(KERN_ERR "%s-%1u> ERROR: IRQ %d allocation FAILED!\n",
		       DRIVER_NAME, minor_num, irq[minor_num]
		    );
		dm6814_release_resources();
		return status;
	}

	/*
	 * IRQ number allocation succeeded, save IRQ number in device structure so
	 * that dm6814_release_resources() can deallocate it
	 */

	device_p->irq = irq[minor_num];

	printk(KERN_INFO
	       "%s-%1u> Device successfully configured at io=%#x, irq=%d\n",
	       DRIVER_NAME, minor_num, device_p->io, device_p->irq);

	return 0;
}

/******************************************************************************
@brief
    Initialize a DM6814 device.  This consists of:@n
    1) clearing and disabling P14 interrupts@n
    2) clearing and disabling all encoder interrupts@n
    3) setting up the device descriptor.

@param
    device_p

    Address of device descriptor for device.

 ******************************************************************************/

static void dm6814_initialize_device(struct DM6814Device *device_p)
{

    /*=========================================================================
    Interrupt initialization
     =========================================================================*/

	dm6814_init_interrupts(device_p);

    /*=========================================================================
    DM6814 device descriptor initialization
     =========================================================================*/

	spin_lock_init(&(device_p->lock));
	device_p->int_count = 0;
	device_p->status_reg = 0x00;
	device_p->reference_count = 0;
	init_waitqueue_head(&(device_p->int_wait_queue));
}

/******************************************************************************
@brief
    Set up all DM6814 devices.  If set up of any device fails, then all device
    set up fails.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
    @arg \c
        -ENAMETOOLONG   Some error occurred when creating device name string.

@note
    Please see the descriptions of dm6814_validate_mod_params(),
    dm6814_check_all_unconfigured(), and dm6814_allocate_resources()
    for information on other possible values returned in this case.
 ******************************************************************************/

static int dm6814_setup_devices(void)
{
	int status;
	uint8_t minor_num;

	status = dm6814_validate_mod_params();
	if (status != 0) {
		return status;
	}

	status = dm6814_check_all_unconfigured();
	if (status != 0) {
		return status;
	}

	for (minor_num = 0; minor_num < DM6814_MAX_DEVS; minor_num++) {
		struct DM6814Device *device_p;

		device_p = &(dm6814_devices[minor_num]);

		if (io[minor_num] == 0) {
			continue;
		}

		/*
		 * Create the full device name, which should contain 12 characters.
		 * The device name is in the form "rtd-dm6814-x" where x is the device
		 * minor number.  snprintf() actually appends a NUL character '\0', so
		 * the space containing the name should have room for at least 13
		 * characters.  snprintf() returns the number of characters written not
		 * including the NUL character.
		 */

		status = snprintf(device_p->device_name,
				  DEVICE_NAME_LENGTH,
				  "%s-%1d", DRIVER_NAME, minor_num);
		if (status != 12) {
			printk(KERN_ERR
			       "%s-%1u> Failed to create device name!\n",
			       DRIVER_NAME, minor_num);
			dm6814_release_resources();
			return -ENAMETOOLONG;
		}

		status = dm6814_allocate_resources(device_p);
		if (status != 0) {
			return status;
		}

		dm6814_initialize_device(device_p);
	}

	return 0;
}

/******************************************************************************
@brief
    Register this character device with the kernel.

@param
    major

    Address of the major value that will be allocated by the kernel.

@retval
    0

    Success.

@retval
    < 0

    Failure.
 ******************************************************************************/

/******************************************************************************
@brief
    Unregister this character device with the kernel.

@retval
    0

    Success.

@retval
    < 0

    Failure.
 ******************************************************************************/

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))

int dm6814_register_char_device(int *major)
{
	int status;

	status = register_chrdev(0, DRIVER_NAME, &dm6814_file_ops);
	if (status < 0) {
		return status;
	}

	*major = status;
	return 0;
}

static int dm6814_unregister_char_device(void)
{
	return unregister_chrdev(dm6814_major, DRIVER_NAME);
}

#else

int dm6814_register_char_device(int *major)
{
	dev_t device;
	int status;

	status = alloc_chrdev_region(&device, 0, DM6814_MAX_DEVS, DRIVER_NAME);
	if (status < 0) {
		return status;
	}

	cdev_init(&dm6814_cdev, &dm6814_file_ops);
	dm6814_cdev.owner = THIS_MODULE;

	status = cdev_add(&dm6814_cdev, device, DM6814_MAX_DEVS);
	if (status < 0) {
		unregister_chrdev_region(device, DM6814_MAX_DEVS);
	}
	*major = MAJOR(device);
	return 0;
}

static int dm6814_unregister_char_device(void)
{
	cdev_del(&dm6814_cdev);
	unregister_chrdev_region(MKDEV(dm6814_major, 0), DM6814_MAX_DEVS);
	return 0;
}

#endif

/******************************************************************************
@brief
    Perform all actions necessary to initialize the DM6814 driver and devices.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
    @arg \c
        -EBUSY      All character device major numbers are in use; returned by
                    register_chrdev().
@note
    Please see the description of dm6814_setup_devices() for
    information on other possible values returned in this case.
 ******************************************************************************/

int dm6814_load(void)
{
	int status;

	DONT_EXPORT_SYMBOLS;

	printk(KERN_INFO "%s> Initializing module (version %s).\n",
	       DRIVER_NAME, DRIVER_RELEASE);

	printk(KERN_INFO "%s> %s\n", DRIVER_NAME, VERSION_STRING);
	printk(KERN_INFO "%s> %s\n", DRIVER_NAME, RTD_COPYRIGHT_STRING);

	memset(&(dm6814_devices[0]), 0, sizeof(dm6814_devices));
	dm6814_major = 0;

	status = dm6814_setup_devices();
	if (status != 0) {
		return status;
	}

	/*
	 * Register the character device, requesting dynamic major number
	 * allocation
	 */

	status = dm6814_register_char_device(&dm6814_major);
	if (status < 0) {
		printk(KERN_ERR
		       "%s> ERROR: Dynamic character device registration failed (errno = "
		       "%d)\n", DRIVER_NAME, status);
		dm6814_release_resources();
		return status;
	}

	printk(KERN_INFO
	       "%s> Driver registered using character major number %d\n",
	       DRIVER_NAME, dm6814_major);

	return 0;
}

/******************************************************************************
@brief
    Perform all actions necessary to deinitialize the DM6814 driver and devices.
 ******************************************************************************/

void dm6814_unload(void)
{

	dm6814_release_resources();

	dm6814_unregister_char_device();

	printk(KERN_INFO "%s> Character device %d unregistered\n",
	       DRIVER_NAME, dm6814_major);

	printk(KERN_INFO "%s> Module unloaded.\n", DRIVER_NAME);
}

/*=============================================================================
Module entry point definitions
 =============================================================================*/

module_init(dm6814_load);
module_exit(dm6814_unload);

/*=============================================================================
Operations supported on DM6814 device files.

NOTE:
	The individual structures in this array are set up using ANSI C
	standard format initialization (which is the preferred method in 2.6
	kernels) instead of tagged initialization (which is the preferred
	method in 2.4 kernels).
 =============================================================================*/

static struct file_operations dm6814_file_ops = {
	.owner = THIS_MODULE,
	.read = dm6814_read,
	.poll = dm6814_poll,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
	.ioctl = dm6814_ioctl,
#else
	.unlocked_ioctl = dm6814_ioctl,
#endif
	.open = dm6814_open,
	.release = dm6814_release,
};
