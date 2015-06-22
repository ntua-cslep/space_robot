/**
    @file

    @brief
        DM6814 library header

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

    $Id: dm6814_library.h 45949 2010-05-05 15:45:09Z wtate $
*/


#ifndef __dm6814_library_h__
#define __dm6814_library_h__


#include <sys/ioctl.h>
#include <sys/types.h>

#include <dm6814_ioctl.h>


/*=============================================================================
Macro definitions
 =============================================================================*/


/*-----------------------------------------------------------------------------
LoadIRQRegister6814() definitions
 -----------------------------------------------------------------------------*/
 
/**
 * @def
 *      Enable IRQ sharing
 *
 * @param
 *      irq_enable
 *
 *      Current IRQ control byte
 *
 * @retval
 *      irq_enable
 *
 *      Modified IRQ control byte with IRQ sharing enabled
 */
 
#define ENABLE_IRQ_SHARING(irq_enable)		((irq_enable) &= ~0x04)

/**
 * @def
 *      Disable IRQ sharing
 *
 * @param
 *      irq_enable
 *
 *      Current IRQ control byte
 *
 * @retval
 *      irq_enable
 *
 *      Modified IRQ control byte with IRQ sharing disabled
 */

#define DISABLE_IRQ_SHARING(irq_enable)		((irq_enable) |= 0x04)

/**
 * @def
 *      Set IRQ polarity to positive
 *
 * @param
 *      irq_enable
 *
 *      Current IRQ control byte
 *
 * @retval
 *      irq_enable
 *
 *      Modified IRQ control byte with positive IRQ polarity
 */

#define POSITIVE_IRQ_POLARITY(irq_enable)	((irq_enable) &= ~0x02)

/**
 * @def
 *      Set IRQ polarity to negative
 *
 * @param
 *      irq_enable
 *
 *      Current IRQ control byte
 *
 * @retval
 *      irq_enable
 *
 *      Modified IRQ control byte with negative IRQ polarity
 */

#define NEGATIVE_IRQ_POLARITY(irq_enable)	((irq_enable) |= 0x02)

/**
 * @def
 *      Enable P14 IRQ
 *
 * @param
 *      irq_enable
 *
 *      Current IRQ control byte
 *
 * @retval
 *      irq_enable
 *
 *      Modified IRQ control byte with P14 IRQ enabled
 */

#define ENABLE_P14_IRQ(irq_enable)		((irq_enable) |= 0x01)

/**
 * @def
 *      Disable P14 IRQ
 *
 * @param
 *      irq_enable
 *
 *      Current IRQ control byte
 *
 * @retval
 *      irq_enable
 *
 *      Modified IRQ control byte with P14 IRQ disabled
 */

#define DISABLE_P14_IRQ(irq_enable)		((irq_enable) &= ~0x01)

/*-----------------------------------------------------------------------------
GetIntStatus6814() definitions
 -----------------------------------------------------------------------------*/

/**
 * @def
 *      Check if P14 interrupt has occured
 *
 * @param
 *      irq_status
 *
 *      IRQ status received from the board
 *
 * @retval
 *      true
 * 
 *      P14 interrupt occured.
 */

#define P14_INT_OCCURRED(irq_status)	(((irq_status) & 0x08) ? true : false)

/**
 * @def
 *      Check if incremental encoder 3 interrupt has occured
 *
 * @param
 *      irq_status
 *
 *      IRQ status received from the board
 *
 * @retval
 *      true
 * 
 *      incremental encoder 3 interrupt occured.
 *
 * @retval
 *      false
 *
 *      incremental encoder 1 interrupt did not occur.
 */

#define ENC3_INT_OCCURRED(irq_status)	(((irq_status) & 0x04) ? true : false)

/**
 * @def
 *      Check if incremental encoder 2 interrupt has occured
 *
 * @param
 *      irq_status
 *
 *      IRQ status received from the board
 *
 * @retval
 *      true
 * 
 *      incremental encoder 2 interrupt occured.
 *
 * @retval
 *      false
 *
 *      incremental encoder 2 interrupt did not occur.
 */

#define ENC2_INT_OCCURRED(irq_status)	(((irq_status) & 0x02) ? true : false)

/**
 * @def
 *      Check if incremental encoder 1 interrupt has occured
 *
 * @param
 *      irq_status
 *
 *      IRQ status received from the board
 *
 * @retval
 *      true
 * 
 *      incremental encoder 1 interrupt occured.
 *
 * @retval
 *      false
 *
 *      incremental encoder 1 interrupt did not occur.
 */

#define ENC1_INT_OCCURRED(irq_status)	(((irq_status) & 0x01) ? true : false)

/*-----------------------------------------------------------------------------
Incremental encoder control register selector definitions
 -----------------------------------------------------------------------------*/

/**
 * @def
 *      Value to write to encoder control register for clear functionality
 */
#define SELECT_CLEAR_REGISTER	0
/**
 * @def
 *      Value to write to encoder control register for hold functionality
 */
#define SELECT_HOLD_REGISTER	1

/**
 * @def
 *      Value to write to encoder control register for digital I/O functionality
 */
#define SELECT_DIO_REGISTER	2


/*=============================================================================
DM6814 device class
 =============================================================================*/

/**
 * @brief
 *      DM6814 Board Class
 */

class DM6814Device {

    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Private type definitions
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

    private:


	/**********************************************************************
	Information about an incremental encoder chip
	 **********************************************************************/

	struct incr_encoder_info {

	    /**
	     * Offset from base I/O address of encoder value LSB register
	     */

	    uint8_t	lsb_offset;

	    /**
	     * Offset from base I/O address of encoder value MSB register
	     */

	    uint8_t	msb_offset;

	    /**
	     * Offset from base I/O address of encoder control register
	     */

	    uint8_t	control_offset;

	    /**
	     * Offset from base I/O address of encoder mode register
	     */

	    uint8_t	mode_offset;
	};

	typedef struct incr_encoder_info incr_encoder_info_t;

    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Private member functions
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

    private:


        /**********************************************************************
        @brief
            Read an 8-bit value from given offset within I/O-mapped space.

        @param
            offset
            
            Offset in I/O space where read should occur.
            
        @param
            data_p
            
            Address where value read should be stored.  The contents of this 
            memory is undefined if the function fails.

        @retval
            true

            Success.

        @retval
            false
            
            Failure.  Please see the ioctl(2) man page for information on 
            possible values errno may have in this case.
         **********************************************************************/

	bool inb(uint8_t offset, uint8_t *data_p);


        /**********************************************************************
        @brief
            Write an 8-bit value to given offset within I/O-mapped space.

        @param
            offset
            
            Offset in I/O space where read should occur.
            
        @param
            data
            
            Data to write.

        @retval
            true

            Success.
            
        @retval
            false

            Failure.@n@n
            Please see the ioctl(2) man page for information on possible values 
            errno may have in this case.
         **********************************************************************/

	bool outb(uint8_t offset, uint8_t data);


        /**********************************************************************
        @brief
            Set the control register which will be accessed for an incremental 
            encoder.

        @param
            offset
            
            Offset from base I/O address of the Encoder Mode Register.
            
        @param
            select
            
            Control register to access.

        @retval
            0

            Success.

        @retval
            -1

            Failure.@n@n
            
            Please see the descriptions of inb() and outb() for information on 
            possible values errno may have in this case.

        @note
            This function does not validate the select parameter as it is
            assumed either to have been previously validated or to be
            correct on entry.
         **********************************************************************/

	int select_encoder_control_register(uint8_t offset, uint8_t select);


        /***********************************************************************
        @brief
            Validate an 8254 timer/counter number passed into other library
            functions.  Also, determine the timer's offset from base I/O
            address.

        @param
            timer
            
            8254 timer/counter number to validate.  Valid values are 0, 1 and 2.
            
        @param
            offset_p
            
            Address where timer's offset from base I/O address should be stored.
            The contents of this memory is undefined if the function fails.

        @retval
            0

            Success.
            
        @retval
            -1
            
            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      timer is not valid.
         **********************************************************************/

	int validate_8254_timer(uint8_t timer, uint8_t *offset_p);


        /**********************************************************************
        @brief
            Validate an incremental encoder number passed into other
            library functions.  This function also determines the following
            information for the encoder:
                * Offset of encoder's least significant byte from base I/O
                  address.
                * Offset of encoder's most significant byte from base I/O
                  address.
                * Offset of encoder's control register from base I/O
                  address.
                * Offset of encoder's mode register from base I/O address.

        @param
            encoder
            
            Incremental encoder number to validate. Valid values are 1 - 3.
            
        @param
            encoder_info_p
            
            Address where encoder information will be stored.  The contents of 
            this memory is undefined if the function fails.

        @retval
            0

            Success.
            
        @retval
            -1

            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      encoder is not valid.
         **********************************************************************/

	int validate_incr_encoder(
	    uint8_t encoder,
	    incr_encoder_info_t *encoder_info_p
	);

    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Public member functions
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

    public:


        /**********************************************************************
        @brief
            Clear the incremental encoder chip for the given incremental
            encoder.  According to the hardware manual, clearing an encoder
            chip will 1) clear the encoder's IRQ status flag, 2) set
            digital I/O bits 0 and 1 to input, and 3) clear the encoder
            counter value.

        @param
            Encoder
            
            Incremental encoder chip to clear.  Valid values are:
                   1 - Incremental encoder 1
                   2 - Incremental encoder 2
                   3 - Incremental encoder 3

        @retval
            true

            Success.

        @retval
            false

            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      Encoder is not valid.

            Please see the descriptions of outb() and
            select_encoder_control_register() for information on other
            possible values errno may have in this case.
         **********************************************************************/

	bool ClearEncoderChip6814(uint8_t Encoder);


        /**********************************************************************
        @brief
            Set the divisor for the given 8254 timer/counter.

        @param
            Timer
            
            The timer to operate on.  Valid values are:
                   0 - Timer/counter 0
                   1 - Timer/counter 1
                   2 - Timer/counter 2
           
       @param
            Divisor
            
            Counter divisor.  Valid values are 0 through 65535.

        @retval
            true

            Success.

        @retval
            false
            
            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      Timer is not valid.

            Please see the description of outb() for information on
            other possible values errno may have in this case.

        @note
            Before calling this function, you must ensure that the
            indicated timer/counter is set to be programmed least
            significant byte first then most significant byte.
         **********************************************************************/

	bool ClockDivisor6814(uint8_t Timer, uint16_t Divisor);


        /**********************************************************************
        @brief
            Set the mode for the given 8254 timer/counter.

        @param
            Timer
            
            The timer to operate on.  Valid values are:
                 0 - Timer/counter 0
                 1 - Timer/counter 1
                 2 - Timer/counter 2
                     
        @param                                     
            Mode

            The counter mode to set.  Valid values are:
                 0 - Event count
                 1 - Programmable one shot
                 2 - Rate generator
                 3 - Square wave rate generator
                 4 - Software triggered strobe
                 5 - Hardware triggered strobe

        @retval
            true

            Success.
            
        @retval
            false
            
            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      Timer is not valid.
            @arg \c
                EINVAL      Mode is not valid.

            Please see the description of outb() for information on
            other possible values errno may have in this case.

        @note
            This function puts the indicated timer/counter into binary
            mode.

        @note
            This function sets the timer/counter to read/load least
            significant byte first then most significant byte.
         **********************************************************************/

	bool ClockMode6814(uint8_t Timer, uint8_t Mode);


        /**********************************************************************
        @brief
            Close a DM6814 device file.

        @retval
            true

            Success.

        @retval
            false

            Failure.@n@n
            
            Please see the close(2) man page for information on possible values 
            errno may have in this case.

        @note
            The kernel, when processing this function, performs all actions
            indicated in InitBoard6814().
         **********************************************************************/

	bool CloseBoard6814(void);


        /**********************************************************************
        @brief
            DM6814Device class constructor.
         **********************************************************************/

	DM6814Device(void);


        /**********************************************************************
        @brief
            DM6814Device class destructor.
            
        @note
            This function closes the DM6814 device file associated with the
            object.
         **********************************************************************/

	~DM6814Device(void);


        /**********************************************************************
        @brief
            Enable or disable the given incremental encoder.

        @param
            Encoder
            
            Incremental encoder to modify state of.  Valid values are:
                   1 - Incremental encoder 1
                   2 - Incremental encoder 2
                   3 - Incremental encoder 3
       
       @param
            Enable
            
            Flag indicating whether or not the encoder should be enabled.  A 
            value of false means disable the encoder.  A value of true means 
            enable the encoder.

        @retval
            true
            
            Success.

        @retval
            false

            Failure.@n@n
            The following values may be returned as:
            @arg \c
                EINVAL      Encoder is not valid.

            Please see the descriptions of inb() and outb() for
            information on other possible values errno may have in this
            case.
         **********************************************************************/

	bool EnableEncoder6814(uint8_t Encoder, bool Enable);


        /**********************************************************************
        @brief
            Enable or disable encoder clear via bit 2 in the even-numbered
            digital I/O port for the given incremental encoder.

        @param
            Encoder
            
            Incremental encoder to modify clear state of.  Valid values are:
                   1 - Incremental encoder 1
                   2 - Incremental encoder 2
                   3 - Incremental encoder 3
                
        @param
            Enable
            
            Flag indicating whether or not encoder clear should be enabled.  A 
            value of false means disable encoder clear.  A value of true means 
            enable encoder clear.

        @retval
            true
            
            Success.
            
        @retval
            false
            
            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      Encoder is not valid.

            Please see the descriptions of inb() and outb() for
            information on other possible values errno may have in this
            case.
         **********************************************************************/

	bool EnableEncoderClear6814(uint8_t Encoder, bool Enable);


        /**********************************************************************
        @brief
            Enable or disable incremental encoder interrupts for the given
            incremental encoder.

        @param
            Encoder
            
            Incremental encoder to modify interrupt state of. Valid values are:
                    1 - Incremental encoder 1
                    2 - Incremental encoder 2
                    3 - Incremental encoder 3
                    
        @param
            Enable
            
            Flag indicating whether or not interrupts should be enabled.  A 
            value of false means disable the encoder interrupt.  A value of 
            true means enable the encoder interrupt.

        @retval
            true

            Success.

        @retval
            false

            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      Encoder is not valid.

                Please see the descriptions of inb() and outb() for
                information on other possible values errno may have in this
                case.
         **********************************************************************/

	bool EnableEncoderIrq6814(uint8_t Encoder, bool Enable);


        /**********************************************************************
        @brief
                Enable or disable blocking I/O on a device file.  This controls
                whether or not GetIntStatus6814() can block inside the kernel.

        @param
            Enable
            
            Flag which controls whether or not blocking I/O is enabled.  A value
            of false means blocking I/O is disabled and GetIntStatus6814() will 
            never block in the kernel waiting for an interrupt to occur.  A 
            value of true means blocking I/O is enabled and GetIntStatus6814() 
            can block in the kernel waiting for an interrupt to occur.

        @retval
            true

            Success.
            
        @retval                       
            false
            
            Failure.@n@n
            
            Please see the fcntl(2) man page for information on possible values 
            errno may have in this case.

        @note
            The default behavior when a device file is opened is to disable
            blocking I/O.
         **********************************************************************/

	bool EnableGetIntStatusWait6814(bool Enable);


        /**********************************************************************
        @brief
            Get the driver version number.  The version number is an
            unsigned integer encoding the major, minor, and patch level
            numbers.

        @param
            version_p
            
            Address where version number should be stored. The contents of this 
            memory is undefined if the function fails.

        @retval
            true

            Success.
            
        @retval
            false
            
            Failure.@n@n
            
            Please see the ioctl(2) man page for information on possible values 
            errno may have in this case.

        @note
            The driver version is encoded according to the formula:
            Version = (
                (MajorVersion << 16)
                |
                (MinorVersion << 8)
                |
                PatchLevelNumber
            )
         **********************************************************************/

	bool GetDriverVersion6814(uint32_t *version_p);


        /**********************************************************************
        @brief
            Atomically obtain the driver's current interrupt count and
            cached IRQ Status Register value.

        @param
            int_count_p
            
            Address where interrupt count should be stored.  The contents of 
            this memory is undefined if the function fails.

        @param                    
            status_reg_p
            
            Address where Status Register value should be stored.  The contents 
            of this memory is undefined if the function fails.

        @retval
            true

            Success.
            
        @retval
            false
            
            Failure.@n@n
            The following values may be returned:
            @arg \c
                EBADMSG     The expected number of bytes were not returned by
                            read(2).  This indicates a serious problem with the
                            driver.
            @arg \c
                EINTR       The process received a signal before an interrupt
                            occurred.  This occurs only if blocking I/O was set
                            for the device file.  This is not a fatal error but
                            rather means the read should be retried.
            @arg \c
                EIO         No IRQ line was allocated to the device when the
                            driver was loaded.

             Please see the read(2) man page for information on other
             possible values errno may have in this case.

        @note
            This function clears the driver's cached IRQ Status Register
            value.  The value will remain cleared until the next interrupt
            occurs.

        @note
            Only bits 0 through 3 in the cached IRQ Status Register value
            are returned.

        @note
            The macros P14_INT_OCCURRED(), ENC3_INT_OCCURRED(),
            ENC2_INT_OCCURRED(), and ENC1_INT_OCCURRED() should be used to
            examine specific Status Register value bits to determine the
            type(s) of interrupt(s) which occurred.  Each macro returns
            true if the associated interrupt occurred and false if it did
            not.

        @note
            This function disables interrupts on the associated DM6814
            device for a very brief time to obtain accurate status
            information.  If you call the function repeatedly in a loop
            (such as when busy-waiting for an interrupt to occur), this can
            interfere with DM6814 interrupts.  It is strongly suggested
            that you do not busy-wait for interrupts.

        @note
            If the device file has been set to use blocking I/O, this
            function will block inside the kernel until an interrupt occurs.

        @note
            If this function is being used to wait for interrupts, signals
            can wake up the process before an interrupt occurs.  If a
            signal is delivered to the process during a wait, the
            application is responsible for dealing with the premature
            awakening in a reasonable manner.

        @note
            When this function is being used to wait for interrupts, it can
            be woken up by a signal before an interrupt occurs and an
            interrupt may be missed if signals are delivered rapidly enough
            or at inopportune times.  To decrease the chances of this, it
            is strongly suggested that you 1) do not use signals or 2)
            minimize their use in your application.

        @note
            This function uses the read(2) system call to wait for an
            interrupt.  read(2) can watch only a single file descriptor for
            activity.
         **********************************************************************/

	bool GetIntStatus6814(uint32_t *int_count_p, uint8_t *status_reg_p);


        /**********************************************************************
        @brief
            Initialize a DM6814 device by performing the following actions:@n
                1) disable IRQ sharing@n
                2) set positive edge P14 interrupt polarity@n
                3) disable P14 interrupts@n
                4) clear each of the incremental encoder chips@n
                5) disable interrupts on each encoder chip@n

        @retval
            true

            Success.

        @retval
            false

            Failure.@n@n
            Please see the descriptions of LoadIRQRegister6814(), 
            ClearEncoderChip6814(), and EnableEncoderIrq6814() for information 
            on possible values errno may have in this case.
         **********************************************************************/

	bool InitBoard6814(void);


        /**********************************************************************
        @brief
            Load a value into the given incremental encoder.

        @param
            Encoder
            
            The incremental encoder to load.  Valid values are:
                   1 - Incremental encoder 1
                   2 - Incremental encoder 2
                   3 - Incremental encoder 3
            
       @param
           Value
           
           The value to load into the encoder.

        @retval
            true

            Success.

        @retval
            false

            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      Encoder is not valid.

                Please see the ioctl(2) man page for information on other
                possible values errno may have in this case.

        @note
            Before loading a value into an incremental encoder, the encoder
            must be disabled.
         **********************************************************************/

	bool LoadEncoder6814(uint8_t Encoder, uint16_t Value);

        /**********************************************************************
        @brief
            Load an 8-bit value into a board's Clear IRQ/IRQ Enable
            Register at base I/O address + 16.

        @param
            Value       
            
            Value to store in register.  Valid values are 0 through 7.

        @retval
            true

            Success.
            
        @retval
            false
            
            Failure.@n@n
            Please see the description of outb() for information on possible 
            values errno may have in this case.

        @note
            The DM6814 interrupt handler is not designed to process
            interrupts shared between devices.  To avoid unpredictable
            behavior or worse, do not share an interrupt between devices.

        @note
            Interrupts do not need to be shared in order to use several
            interrupt sources on a single board.  For example if you wish
            to use both P14 and encoder 1 interrupts on a single DM6814
            device, then interrupts do not need to be shared.

        @note
            The macros ENABLE_IRQ_SHARING(), DISABLE_IRQ_SHARING(),
            POSITIVE_IRQ_POLARITY(), NEGATIVE_IRQ_POLARITY(),
            ENABLE_P14_IRQ(), and DISABLE_P14_IRQ() should be used to set
            or clear bits in the value passed to this function.
         **********************************************************************/

	bool LoadIRQRegister6814(uint8_t Value);


        /**********************************************************************
        @brief
            Open a DM6814 device file.

        @param
            nDevice
            
            Minor number of DM6814 device file.

        @retval
            true

            Success.
            
        @retval
            false
            
            
            Failure.@n@n
            
            Please see the open(2) man page for information on possible values 
            errno may have in this case.

        @note
            Once a device file is open, it cannot be opened again until it
            is closed.

        @note
            The device file is opened with non-blocking I/O enabled.

        @note
            The kernel, when processing this function, performs all actions
            indicated in InitBoard6814().
         **********************************************************************/

	bool OpenBoard6814(uint32_t nDevice);


        /**********************************************************************
        @brief
            Read an 8-bit value from the given offset within a DM6814
            board's I/O memory.

        @param
            offset
            
            Offset within I/O memory to read.
            
        @param
            data_p
            
            Address where data read should be stored.  The contents of this 
            memory is undefined if the function fails.

        @retval
            true

            Success.

        @retval
            false
               
            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      offset is not valid.
            @arg \c
                EOPNOTSUPP  offset is valid but it represents a write-only
                            register.

                Please see the ioctl(2) man page for information on other
                possible values errno may have in this case.

        @note
            It is strongly suggested that you use other library functions
            instead of directly accessing a board's registers.
         **********************************************************************/

	bool ReadByte6814(uint8_t offset, uint8_t *data_p);


        /**********************************************************************
        @brief
            Read the value in the given incremental encoder.

        @param
            Encoder
            
            The incremental encoder to read.  Valid values are:
                   1 - Incremental encoder 1
                   2 - Incremental encoder 2
                   3 - Incremental encoder 3
                   
       @param
            value_p
            
            Address where encoder value should be stored.  The contents of this 
            memory is undefined if the function fails. 

        @retval
            true

            Success.
            
        @retval
            false
            
            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      Encoder is not valid.

                Please see the ioctl(2) man page and the descriptions of
                select_encoder_control_register() and outb() for
                information on other possible values errno may have in this
                case.
         **********************************************************************/

	bool ReadEncoder6814(uint8_t Encoder, uint16_t *value_p);


        /**********************************************************************
        @brief
            Read the digital I/O lines for the given incremental encoder.

        @param
            Encoder
            
            The incremental encoder to read digital I/O lines of.  Valid values 
            are:
                   1 - Incremental encoder 1
                   2 - Incremental encoder 2
                   3 - Incremental encoder 3
       
        @param
            value_p
            
            Address where digital I/O value should be stored.  The contents of 
            this memory is undefined if the function fails.

        @retval
            true

            Success.
            
        @retval
            false
            
            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      Encoder is not valid.

            Please see the descriptions of inb() and
            select_encoder_control_register() for information on other
            possible values errno may have in this case.

        @note
            Bits 0 and 1 in the value read are undefined if those port bits
            were not previously set to input.

        @note
            The port bits associated with bits 2 through 7 in the value
            read are always set to input.
         **********************************************************************/

	bool ReadEncoderDIO6814(uint8_t Encoder, uint8_t *value_p);


        /**********************************************************************
        @brief
            Read the count for the given timer/counter.

        @param
            Timer
            
            The timer to operate on.  Valid values are:
                   0 - Timer/counter 0
                   1 - Timer/counter 1
                   2 - Timer/counter 2
                   
       @param
            count_p
            
            Address where timer count should be stored.  The contents of this 
            memory is undefined if the function fails.

        @retval
            true

            Success.

        @retval                    
            false
                    
            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      Timer is not valid.

                Please see the descriptions of outb() and inb() for
                information on other possible values errno may have in this
                case.
         **********************************************************************/

	bool ReadTimerCounter6814(uint8_t Timer, uint16_t *count_p);


        /**********************************************************************
        @brief
            Set the digital I/O line bit 0 and 1 directions for the given
            incremental encoder.

        @param
            Encoder
            
            The incremental encoder to set digital I/O line direction of.  
            Valid values are:
                  1 - Incremental encoder 1
                  2 - Incremental encoder 2
                  3 - Incremental encoder 3
                  
        @param
            Bit0Output
            
            Flag indicating whether or not bit 0 should be set to output.  A 
            value of false means set bit 0 to input.  A value of true means set 
            bit 0 to output.
            
        @param
            Bit1Output
            
            Flag indicating whether or not bit 1 should be set to output.  A 
            value of false means set bit 1 to input.  A value of true means set 
            bit 1 to output.

        @retval
            true

            Success.
            
        @retval
            false
            
            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      Encoder is not valid.

                Please see the descriptions of inb() and outb() for
                information on other possible values errno may have in this
                case.
         **********************************************************************/

	bool SetEncoderDIODirection6814(
	    uint8_t Encoder,
	    bool Bit0Output,
	    bool Bit1Output
	);


        /**********************************************************************
        @brief
            Set the given timer/counter into rate generator mode and
            program its divisor value based upon the specified input and
            output rates.

        @param
            Timer
            
            The timer to program.  Valid values are:
                 0 - Timer/counter 0
                 1 - Timer/counter 1
                 2 - Timer/counter 2
        @param
            InputRate
            
            Input clock rate to timer/counter.
            
        @param
            OutputRate
            
            Desired output rate.
            
        @param
            actual_rate_p
            
            Address where actual programmed frequency should be stored.  The 
            contents of this memory is undefined if the function fails.

        @retval
            true

            Success.
            
        @retval
            false
            
            Failure.@n@n
            Please see the descriptions of ClockDivisor6814() and 
            ClockMode6814() for information on possible values errno may have in
            this case.
         **********************************************************************/

	bool SetUserClock6814(
	    uint8_t Timer,
	    float InputRate,
	    float OutputRate,
	    float *actual_rate_p
	);


        /**********************************************************************
        @brief
            Wait for an interrupt to occur on a device.

        @retval
            true

            Success.

        @retval
            false

            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINTR   The process received a signal before an interrupt
                        occurred.  This is not a fatal error but rather
                        means the wait should be retried.
            @arg \c
                EIO     No IRQ line was allocated to the device when the
                        driver was loaded.
            @arg \c
                ENODATA No signal was delivered and select(2) woke up
                        without indicating that an interrupt occurred.
                        This indicates serious problems within the driver.

                Please see the select(2) man page for information on other
                possible values errno may have in this case.

        @note
            Signals can wake up the process before an interrupt occurs.  If
            a signal is delivered to the process during a wait, the
            application is responsible for dealing with the premature
            awakening in a reasonable manner.

        @note
            Because this function can be woken up by a signal before an
            interrupt occurs, an interrupt may be missed if signals are
            delivered rapidly enough or at inopportune times.  To decrease
            the chances of this, it is strongly suggested that you 1) do
            not use signals or 2) minimize their use in your application.

        @note
            This function uses the select(2) system call to wait for an
            interrupt.  If necessary, select(2) can watch multiple file
            descriptors for activity.
         **********************************************************************/

	bool WaitForInterrupt6814(void);


        /**********************************************************************
        @brief
            Write an 8-bit value to the given offset within a DM6814 board's I/O
            memory.

        @param
            offset
            
            Offset within I/O memory to write.
            
        @param
            data
            
            Data to write.

        @retval
            true
            
            Success.
            
        @retval
            false

            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      offset is not valid.
            @arg \c
                EOPNOTSUPP  offset is valid but it represents a read-only
                            register.

                Please see the ioctl(2) man page for information on other
                possible values errno may have in this case.

        @note
            It is strongly suggested that you use other library functions
            instead of directly accessing a board's registers.
         **********************************************************************/

	bool WriteByte6814(uint8_t offset, uint8_t data);


        /**********************************************************************
        @brief
            Write the digital I/O lines for the given incremental encoder.

        @param
            Encoder
            
            The incremental encoder to write digital I/O lines of.  Valid values
            are:
                   1 - Incremental encoder 1
                   2 - Incremental encoder 2
                   3 - Incremental encoder 3
                   
       @param
            Value
            
            The digital value to write.

        @retval
            true

            Success.

        @retval
            false

            Failure.@n@n
            The following values may be returned:
            @arg \c
                EINVAL      Encoder is not valid.

                Please see the descriptions of outb() and
                select_encoder_control_register() for information on other
                possible values errno may have in this case.

        @note
            To write to digital I/O port bits 0 and 1, these bits must have
            been previously set to output.

        @note
            The port bits associated with bits 2 through 7 in the value to
            write are ignored since they are always set to input.
         **********************************************************************/

	bool WriteEncoderDIO6814(uint8_t Encoder, uint8_t Value);

    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Private member variables
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

    private:

	/**
	 * File descriptor for DM6814 device file
	 */

	int hDevice;
};


#endif
