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
#ifndef __FSL_LPSPI_UCOSII_H__
#define __FSL_LPSPI_UCOSII_H__

#include <ucos_ii.h>

#include "fsl_lpspi.h"

/*!
 * @addtogroup lpspi_ucosii_driver LPSPI uCOS II driver
 * @{
 */

/**********************************************************************************************************************
 * Definitions
 *********************************************************************************************************************/

/*! @brief LPSPI uCOS II handle */
typedef struct _lpspi_rtos_handle
{
    LPSPI_Type *base;                 /*!< LPSPI base address */
    lpspi_master_handle_t drv_handle; /*!< Handle of the underlying driver, treated as opaque by the RTOS layer */
    status_t async_status;
    OS_EVENT *mutex; /*!< Mutex to lock the handle during a transfer */
#define RTOS_LPSPI_COMPLETE 0x1
    OS_FLAG_GRP *event; /*!< Semaphore to notify and unblock task when transfer ends */
} lpspi_rtos_handle_t;

/**********************************************************************************************************************
 * API
 *********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name LPSPI RTOS Operation
 * @{
 */

/*!
 * @brief Initializes LPSPI.
 *
 * This function initializes the LPSPI module and related RTOS context.
 *
 * @param handle The RTOS LPSPI handle, the pointer to an allocated space for RTOS context.
 * @param base The pointer base address of the LPSPI instance to initialize.
 * @param masterConfig Configuration structure to set-up LPSPI in master mode.
 * @param srcClock_Hz Frequency of input clock of the LPSPI module.
 * @return status of the operation.
 */
status_t LPSPI_RTOS_Init(lpspi_rtos_handle_t *handle,
                         LPSPI_Type *base,
                         const lpspi_master_config_t *masterConfig,
                         uint32_t srcClock_Hz);

/*!
 * @brief Deinitializes the LPSPI.
 *
 * This function deinitializes the LPSPI module and related RTOS context.
 *
 * @param handle The RTOS LPSPI handle.
 */
status_t LPSPI_RTOS_Deinit(lpspi_rtos_handle_t *handle);

/*!
 * @brief Performs SPI transfer.
 *
 * This function performs an SPI transfer according to data given in the transfer structure.
 *
 * @param handle The RTOS LPSPI handle.
 * @param transfer Structure specifying the transfer parameters.
 * @return status of the operation.
 */
status_t LPSPI_RTOS_Transfer(lpspi_rtos_handle_t *handle, lpspi_transfer_t *transfer);

/*!
 * @}
 */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */

#endif /* __FSL_LPSPI_UCOSII_H__ */
