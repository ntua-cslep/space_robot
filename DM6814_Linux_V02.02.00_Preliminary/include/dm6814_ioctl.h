/**
    @file

    @brief
        DM6814 ioctl interface header

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

    $Id: dm6814_ioctl.h 45949 2010-05-05 15:45:09Z wtate $
*/


#ifndef __dm6814_ioctl__h_
#define __dm6814_ioctl__h_


#ifdef __cplusplus

extern "C" {

#endif

/**
 * @brief
 *      DM6814 8-bit I/O ioctl structure
 */
 
struct dm6814_ioctl_8_bit_access {

    /**
     * Offset within I/O space to access
     */

    uint8_t offset;

    /**
     * Data to write or the data read
     */

    uint8_t data;
};

/**
 * @brief
 *      DM6814 8-bit I/O ioctl structure type
 */

typedef struct dm6814_ioctl_8_bit_access dm6814_ioctl_8_bit_access_t;

/**
 * @brief
 *      DM6814 16-bit I/O ioctl structure
 */

struct dm6814_ioctl_16_bit_access {

    /**
     * Offset within I/O space to access
     */

    uint8_t offset;

    /**
     * Data to write or the data read
     */

    uint16_t data;
};

/**
 * @brief
 *      DM6814 16-bit I/O ioctl structure type
 */

typedef struct dm6814_ioctl_16_bit_access dm6814_ioctl_16_bit_access_t;

/**
 * @brief
 *      DM6814 driver version ioctl structure
 *
 * @note
 *      The driver version is incoded as follows:
 *     ((MajorVersion << 16) | (MinorVersion << 8) | PatchLevelNumber)
 */
 
struct dm6814_ioctl_driver_version {

    /**
     * Encoded driver version
     */

    uint32_t driver_version;
};

/**
 * @brief
 *      DM6814 driver version ioctl structure type
 */

typedef struct dm6814_ioctl_driver_version dm6814_ioctl_driver_version_t;


/*=============================================================================
ioctl() request structure encapsulating all possible requests.  This is what
gets passed into the kernel from user space on the ioctl() call.
 =============================================================================*/

/**
 * @brief
 *      Union of all possible driver ioctl requests
 */

union dm6814_ioctl_argument {
    dm6814_ioctl_8_bit_access_t		access_8;
    dm6814_ioctl_16_bit_access_t	access_16;
    dm6814_ioctl_driver_version_t	version;
};

/**
 * @brief
 *      Ioctl request union type
 */

typedef union dm6814_ioctl_argument dm6814_ioctl_argument_t;


/*=============================================================================
Some defines to generate the driver's ioctl() request codes
 =============================================================================*/

/**
 * @def
 *      Magic value to generate DM6814 ioctl() request codes
 */
#define DM6814_IOCTL_MAGIC 'k'

/**
 * @def
 *      Value from which to start numbering the ioctl() request codes
 */

#define DM6814_IOCTL_REQUEST_BASE 0x00


/*=============================================================================
Driver ioctl() request codes
 =============================================================================*/


/**
 * @def
 *      Request code for returning driver version
 */

#define DM6814_IOCTL_GET_DRIVER_VERSION \
    _IOR( \
	DM6814_IOCTL_MAGIC, \
	(DM6814_IOCTL_REQUEST_BASE + 1), \
	dm6814_ioctl_argument_t \
    )

/**
 * @def
 *      Request code for reading 8-bits from device I/O space
 */

#define DM6814_IOCTL_READ_8_BITS \
    _IOWR( \
	DM6814_IOCTL_MAGIC, \
	(DM6814_IOCTL_REQUEST_BASE + 2), \
	dm6814_ioctl_argument_t \
    )

/**
 * @def
 *      Request code for writing 8-bits to device I/O space
 */

#define DM6814_IOCTL_WRITE_8_BITS \
    _IOW( \
	DM6814_IOCTL_MAGIC, \
	(DM6814_IOCTL_REQUEST_BASE + 3), \
	dm6814_ioctl_argument_t \
    )

/**
 * @def
 *      Request code for reading 16-bits from device I/O space
 */

#define DM6814_IOCTL_READ_16_BITS \
    _IOWR( \
	DM6814_IOCTL_MAGIC, \
	(DM6814_IOCTL_REQUEST_BASE + 4), \
	dm6814_ioctl_argument_t \
    )

/**
 * @def
 *      Request code for writing 16-bits to device I/O space
 */

#define DM6814_IOCTL_WRITE_16_BITS \
    _IOW( \
	DM6814_IOCTL_MAGIC, \
	(DM6814_IOCTL_REQUEST_BASE + 5), \
	dm6814_ioctl_argument_t \
    )


#ifdef __cplusplus

}

#endif

#endif
