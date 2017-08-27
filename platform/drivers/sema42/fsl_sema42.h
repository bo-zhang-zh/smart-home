/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FSL_SEMA42_H_
#define _FSL_SEMA42_H_

#include "fsl_common.h"

/*!
 * @addtogroup sema42
 * @{
 */

/*! @file */

/******************************************************************************
 * Definitions
 *****************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief SEMA42 driver version */
#define FSL_SEMA42_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Driver version 2.0.0 */
/*@}*/

/*!
 * @brief SEMA42 status return codes.
 */
enum _sema42_status
{
    kStatus_SEMA42_Busy = MAKE_STATUS(kStatusGroup_SEMA42, 0), /*!< SEMA42 gate has been locked by other processor. */
    kStatus_SEMA42_Reseting = MAKE_STATUS(kStatusGroup_SEMA42, 1) /*!< SEMA42 gate reseting is ongoing. */
};

/*!
 * @brief SEMA42 gate lock status.
 */
typedef enum _sema42_gate_status
{
    kSEMA42_Unlocked = 0U,        /*!< The gate is unlocked.               */
    kSEMA42_LockedByProc0 = 1U,   /*!< The gate is locked by processor 0.  */
    kSEMA42_LockedByProc1 = 2U,   /*!< The gate is locked by processor 1.  */
    kSEMA42_LockedByProc2 = 3U,   /*!< The gate is locked by processor 2.  */
    kSEMA42_LockedByProc3 = 4U,   /*!< The gate is locked by processor 3.  */
    kSEMA42_LockedByProc4 = 5U,   /*!< The gate is locked by processor 4.  */
    kSEMA42_LockedByProc5 = 6U,   /*!< The gate is locked by processor 5.  */
    kSEMA42_LockedByProc6 = 7U,   /*!< The gate is locked by processor 6.  */
    kSEMA42_LockedByProc7 = 8U,   /*!< The gate is locked by processor 7.  */
    kSEMA42_LockedByProc8 = 9U,   /*!< The gate is locked by processor 8.  */
    kSEMA42_LockedByProc9 = 10U,  /*!< The gate is locked by processor 9.  */
    kSEMA42_LockedByProc10 = 11U, /*!< The gate is locked by processor 10. */
    kSEMA42_LockedByProc11 = 12U, /*!< The gate is locked by processor 11. */
    kSEMA42_LockedByProc12 = 13U, /*!< The gate is locked by processor 12. */
    kSEMA42_LockedByProc13 = 14U, /*!< The gate is locked by processor 13. */
    kSEMA42_LockedByProc14 = 15U  /*!< The gate is locked by processor 14. */
} sema42_gate_status_t;

/*! @brief The number to reset all SEMA42 gates. */
#define SEMA42_GATE_NUM_RESET_ALL (64U)

/*! @brief SEMA42 gate n register address.
 *
 * The SEMA42 gates are sorted in the order 3, 2, 1, 0, 7, 6, 5, 4, ... not in the order
 * 0, 1, 2, 3, 4, 5, 6, 7, ... The macro SEMA42_GATEn gets the SEMA42 gate based on the gate
 * index.
 *
 * The input gate index is XOR'ed with 3U:
 * 0 ^ 3 = 3
 * 1 ^ 3 = 2
 * 2 ^ 3 = 1
 * 3 ^ 3 = 0
 * 4 ^ 3 = 7
 * 5 ^ 3 = 6
 * 6 ^ 3 = 5
 * 7 ^ 3 = 4
 * ...
 */
#define SEMA42_GATEn(base, n) (*(&((base)->GATE3) + ((n) ^ 3U)))

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the SEMA42 module.
 *
 * This function initializes the SEMA42 module. It only enables the clock but does
 * not reset the gates because the module might be used by other processors
 * at the same time. To reset the gates, call either SEMA42_ResetGate or
 * SEMA42_ResetAllGates function.
 *
 * @param base SEMA42 peripheral base address.
 */
static inline void SEMA42_Init(SEMA42_Type *base)
{
    if (SEMA420 == base)
    {
        CLOCK_EnableClock(kCLOCK_Sema420);
    }
    else if (SEMA421 == base)
    {
        CLOCK_EnableClock(kCLOCK_Sema421);
    }
    else
    {
    }
}

/*!
 * @brief De-initializes the SEMA42 module.
 *
 * This function de-initializes the SEMA42 module. It only disables the clock.
 *
 * @param base SEMA42 peripheral base address.
 */
static inline void SEMA42_Deinit(SEMA42_Type *base)
{
    if (SEMA420 == base)
    {
        CLOCK_DisableClock(kCLOCK_Sema420);
    }
    else if (SEMA421 == base)
    {
        CLOCK_DisableClock(kCLOCK_Sema421);
    }
    else
    {
    }
}

/*!
 * @brief Tries to lock the SEMA42 gate.
 *
 * This function tries to lock the specific SEMA42 gate. If the gate has been
 * locked by another processor, this function returns an error code.
 *
 * @param base SEMA42 peripheral base address.
 * @param gateNum  Gate number to lock.
 * @param procNum  Current processor number.
 *
 * @retval kStatus_Success     Lock the sema42 gate successfully.
 * @retval kStatus_SEMA42_Busy Sema42 gate has been locked by another processor.
 */
status_t SEMA42_TryLock(SEMA42_Type *base, uint8_t gateNum, uint8_t procNum);

/*!
 * @brief Locks the SEMA42 gate.
 *
 * This function locks the specific SEMA42 gate. If the gate has been
 * locked by other processors, this function waits until it is unlocked and then
 * lock it.
 *
 * @param base SEMA42 peripheral base address.
 * @param gateNum  Gate number to lock.
 * @param procNum  Current processor number.
 */
void SEMA42_Lock(SEMA42_Type *base, uint8_t gateNum, uint8_t procNum);

/*!
 * @brief Unlocks the SEMA42 gate.
 *
 * This function unlocks the specific SEMA42 gate. It only writes unlock value
 * to the SEMA42 gate register. However, it does not check whether the SEMA42 gate is locked
 * by the current processor or not. As a result, if the SEMA42 gate is not locked by the current
 * processor, this function has no effect.
 *
 * @param base SEMA42 peripheral base address.
 * @param gateNum  Gate number to unlock.
 */
static inline void SEMA42_Unlock(SEMA42_Type *base, uint8_t gateNum)
{
    assert(gateNum < FSL_FEATURE_SEMA42_GATE_COUNT);

    /* ^= 0x03U because SEMA42 gates are in the order 3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8, 7, ...*/
    SEMA42_GATEn(base, gateNum) = kSEMA42_Unlocked;
}

/*!
 * @brief Gets the status of the SEMA42 gate.
 *
 * This function checks the lock status of a specific SEMA42 gate.
 *
 * @param base SEMA42 peripheral base address.
 * @param gateNum  Gate number.
 *
 * @return status  Current status.
 */
static inline sema42_gate_status_t SEMA42_GetGateStatus(SEMA42_Type *base, uint8_t gateNum)
{
    assert(gateNum < FSL_FEATURE_SEMA42_GATE_COUNT);

    return (sema42_gate_status_t)(SEMA42_GATEn(base, gateNum));
}

/*!
 * @brief Resets the SEMA42 gate to an unlocked status.
 *
 * This function resets a SEMA42 gate to an unlocked status.
 *
 * @param base SEMA42 peripheral base address.
 * @param gateNum  Gate number.
 *
 * @retval kStatus_Success         SEMA42 gate is reset successfully.
 * @retval kStatus_SEMA42_Reseting Some other reset process is ongoing.
 */
status_t SEMA42_ResetGate(SEMA42_Type *base, uint8_t gateNum);

/*!
 * @brief Resets all SEMA42 gates to an unlocked status.
 *
 * This function resets all SEMA42 gate to an unlocked status.
 *
 * @param base SEMA42 peripheral base address.
 *
 * @retval kStatus_Success         SEMA42 is reset successfully.
 * @retval kStatus_SEMA42_Reseting Some other reset process is ongoing.
 */
static inline status_t SEMA42_ResetAllGates(SEMA42_Type *base)
{
    return SEMA42_ResetGate(base, SEMA42_GATE_NUM_RESET_ALL);
}

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */

#endif /* _FSL_SEMA42_H_ */
