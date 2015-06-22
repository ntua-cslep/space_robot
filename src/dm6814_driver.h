/**
    @file

    @brief
        DM6814 driver header

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

    $Id: dm6814_driver.h 45949 2010-05-05 15:45:09Z wtate $
*/


#ifndef __dm6814_driver_h__
#define __dm6814_driver_h__

#include <linux/spinlock.h>
#include <linux/wait.h>

#include <dm6814_ioctl.h>


/**
 * @def
 *      The length in bytes of a DM6814 device's I/O space
 */

#define DM6814_IO_EXTENT	0x20

/**
 * @def
 *      The maximum number of supported DM6814 devices
 */

#define DM6814_MAX_DEVS		4


/**
 * @def
 *      The length in bytes of a DM6814 device file name; this includes the 
 *      trailing NUL character
 */

#define DEVICE_NAME_LENGTH	15


/*=============================================================================
DM6814 device descriptor
 =============================================================================*/

/**
 * @brief
 *      DM6814 device
 */
 
struct DM6814Device {

    /**
     * Number of device interrupts since the device file was opened
     */

    uint32_t			int_count;

    /**
     * Base I/O address.  A value of zero means either no device is present or
     * it is to be ignored.
     */

    uint32_t			io;

    /**
     * IRQ line.  A value of zero means the interrupt functionality will not be
     * used.
     */

    uint32_t			irq;

    /**
     * Concurrency control
     */

    spinlock_t			lock;

    /**
     * Base name of device file.  A string in the form "rtd-dm6814-x" where x
     * is the device minor number.
     */

    char			device_name[DEVICE_NAME_LENGTH];

    /**
     * Cache of board's IRQ Status Register value.  Cleared when interrupt
     * status is read.  Set by interrupt handler.
     */

    uint8_t			status_reg;

    /**
     * Queue of processes waiting to be woken up when an interrupt occurs
     */

    wait_queue_head_t		int_wait_queue;

    /**
     * Number of entities which have the device file open.  Used to enforce
     * single open semantics.
     */

    uint8_t			reference_count;
};


#endif
