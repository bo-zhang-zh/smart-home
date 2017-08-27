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
#ifndef __FSL_LPI2C_UCOSII_H__
#define __FSL_LPI2C_UCOSII_H__

#include <ucos_ii.h>

#include "fsl_lpi2c.h"

/*!
 * @addtogroup lpi2c_ucosii_driver LPI2C µCOS/II driver
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief LPI2C uCOS II handle */
typedef struct _lpi2c_rtos_handle
{
    LPI2C_Type *base;                 /*!< LPI2C base address */
    lpi2c_master_handle_t drv_handle; /*!< Handle of the underlying driver, treated as opaque by the RTOS layer */
    status_t async_status;
    OS_EVENT *mutex; /*!< Mutex to lock the handle during a trasfer */
#define RTOS_LPI2C_COMPLETE 0x1
    OS_FLAG_GRP *event; /*!< Semaphore to notify and unblock task when transfer ends */
} lpi2c_rtos_handle_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name LPI2C RTOS Operation
 * @{
 */

/*!
 * @brief Initializes LPI2C.
 *
 * This function initializes the LPI2C module and related RTOS context.
 *
 * @param handle The RTOS LPI2C handle, the pointer to an allocated space for RTOS context.
 * @param base The pointer base address of the LPI2C instance to initialize.
 * @param masterConfig Configuration structure to set-up LPI2C in master mode.
 * @param srcClock_Hz Frequency of input clock of the LPI2C module.
 * @return status of the operation.
 */
status_t LPI2C_RTOS_Init(lpi2c_rtos_handle_t *handle,
                         LPI2C_Type *base,
                         const lpi2c_master_config_t *masterConfig,
                         uint32_t srcClock_Hz);

/*!
 * @brief Deinitializes the LPI2C.
 *
 * This function deinitializes the LPI2C module and related RTOS context.
 *
 * @param handle The RTOS LPI2C handle.
 */
status_t LPI2C_RTOS_Deinit(lpi2c_rtos_handle_t *handle);

/*!
 * @brief Performs I2C transfer.
 *
 * This function performs an I2C transfer using LPI2C module according to data given in the transfer structure.
 *
 * @param handle The RTOS LPI2C handle.
 * @param transfer Structure specifying the transfer parameters.
 * @return status of the operation.
 */
status_t LPI2C_RTOS_Transfer(lpi2c_rtos_handle_t *handle, lpi2c_master_transfer_t *transfer);

/*!
 * @}
 */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */

#endif /* __FSL_LPI2C_UCOSII_H__ */
