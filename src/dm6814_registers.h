/**
    @file

    @brief
        DM6814 registers

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

    $Id: dm6814_registers.h 45949 2010-05-05 15:45:09Z wtate $
*/


#ifndef __dm6814_registers_h__
#define __dm6814_registers_h__


/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Each register is defined as an offset from a board's base I/O address
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*
 * Encoder 1
 */
/**
 * @def
 *      Incremental Encoder 1 Least-Significant Byte
 */
#define INCREMENTAL_ENCODER_1_LSB	0
/**
 * @def
 *      Incremental Encoder 1 Most-Significant Byte
 */
#define INCREMENTAL_ENCODER_1_MSB	1
/**
 * @def
 *      Incremental Encoder 1 Clear
 */
#define INCREMENTAL_ENCODER_1_CLEAR	2
/**
 * @def
 *      Incremental Encoder 1 Hold
 */
#define INCREMENTAL_ENCODER_1_HOLD	2
/**
 * @def
 *      Incremental Encoder 1 Digital I/O
 */
#define INCREMENTAL_ENCODER_1_DIO	2
/**
 * @def
 *      Incremental Encoder 1 Mode
 */
#define INCREMENTAL_ENCODER_1_MODE	3
/*
 * Encoder 2
 */
/**
 * @def
 *      Incremental Encoder 2 Least-Significant Byte
 */
#define INCREMENTAL_ENCODER_2_LSB	4
/**
 * @def
 *      Incremental Encoder 2 Most-Significant Byte
 */
#define INCREMENTAL_ENCODER_2_MSB	5
/**
 * @def
 *      Incremental Encoder 2 Clear
 */
#define INCREMENTAL_ENCODER_2_CLEAR	6
/**
 * @def
 *      Incremental Encoder 2 Hold
 */
#define INCREMENTAL_ENCODER_2_HOLD	6
/**
 * @def
 *      Incremental Encoder 2 Digital I/O
 */
#define INCREMENTAL_ENCODER_2_DIO	6
/**
 * @def
 *      Incremental Encoder 2 Mode
 */
#define INCREMENTAL_ENCODER_2_MODE	7
/*
 * Encoder 3
 */
/**
 * @def
 *      Incremental Encoder 3 Least-Significant Byte
 */
#define INCREMENTAL_ENCODER_3_LSB	8
/**
 * @def
 *      Incremental Encoder 3 Most-Significant Byte
 */
#define INCREMENTAL_ENCODER_3_MSB	9
/**
 * @def
 *      Incremental Encoder 3 Clear
 */
#define INCREMENTAL_ENCODER_3_CLEAR	10
/**
 * @def
 *      Incremental Encoder 3 Hold
 */
#define INCREMENTAL_ENCODER_3_HOLD	10
/**
 * @def
 *      Incremental Encoder 3 Digital I/O
 */
#define INCREMENTAL_ENCODER_3_DIO	10
/**
 * @def
 *      Incremental Encoder 3 Mode
 */
#define INCREMENTAL_ENCODER_3_MODE	11
/*
 * Timer/Counters
 */
/**
 * @def
 *      Timer/Counter 0
 */
#define TIMER_COUNTER_0			12
/**
 * @def
 *      Timer/Counter 1
 */
#define TIMER_COUNTER_1			13
/**
 * @def
 *      Timer/Counter 2
 */
#define TIMER_COUNTER_2			14
/**
 * @def
 *      Timer/Counter Control
 */
#define TIMER_COUNTER_CONTROL		15
/*
 * General
 */
/**
 * @def
 *      IRQ Control
 */
#define IRQ_CONTROL			16
/**
 * @def
 *      IRQ Status
 */
#define IRQ_STATUS			17


#endif
