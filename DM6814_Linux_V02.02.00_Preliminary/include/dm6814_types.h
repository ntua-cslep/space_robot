/**
    @file

    @brief
        DM6814 type definitions

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

    $Id: dm6814_types.h 45949 2010-05-05 15:45:09Z wtate $
*/


#ifndef __dm6814_types_h__
#define __dm6814_types_h__


/*=============================================================================
Structure which defines the interrupt status information returned on a read(2)
system call
 =============================================================================*/

/**
 * @brief
 *      DM6814 interrupt status
 */
 
struct dm6814_int_status {

    /**
     * Number of interrupts which have occurred since device file was opened
     */

    uint32_t	int_count;

    /**
     * Driver's cached IRQ Status Register value.  Only bits 0 through 3 in the
     * Status Register have meaning and are returned.
     */

    uint8_t	status_reg;
};

/**
 * @brief
 *      DM6814 interrupt status type
 */

typedef struct dm6814_int_status dm6814_int_status_t;


#endif
