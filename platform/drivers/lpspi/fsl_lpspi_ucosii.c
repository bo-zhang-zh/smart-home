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

#include "fsl_lpspi_ucosii.h"
#include <ucos_ii.h>
#include <lib_mem.h>

static void LPSPI_RTOS_Callback(LPSPI_Type *base, lpspi_master_handle_t *drv_handle, status_t status, void *userData)
{
    uint8_t err;
    lpspi_rtos_handle_t *handle = (lpspi_rtos_handle_t *)userData;
    OSFlagPost(handle->event, RTOS_LPSPI_COMPLETE, OS_FLAG_SET, &err);
}

status_t LPSPI_RTOS_Init(lpspi_rtos_handle_t *handle,
                         LPSPI_Type *base,
                         const lpspi_master_config_t *masterConfig,
                         uint32_t srcClock_Hz)
{
    uint8_t err;

    if (handle == NULL)
    {
        return kStatus_InvalidArgument;
    }

    if (base == NULL)
    {
        return kStatus_InvalidArgument;
    }

    memset(handle, 0, sizeof(lpspi_rtos_handle_t));

    handle->mutex = OSSemCreate(1);
    if (NULL == handle->mutex)
    {
        return kStatus_Fail;
    }

    handle->event = OSFlagCreate(0, &err);
    if (OS_ERR_NONE != err)
    {
        OSSemDel(handle->mutex, OS_DEL_ALWAYS, &err);
        return kStatus_Fail;
    }

    handle->base = base;

    LPSPI_MasterInit(handle->base, masterConfig, srcClock_Hz);
    LPSPI_MasterTransferCreateHandle(handle->base, &handle->drv_handle, LPSPI_RTOS_Callback, (void *)handle);

    return kStatus_Success;
}

status_t LPSPI_RTOS_Deinit(lpspi_rtos_handle_t *handle)
{
    uint8_t err;

    LPSPI_Deinit(handle->base);
    OSFlagDel(handle->event, OS_DEL_ALWAYS, &err);

    /* Give the semaphore. This is for functional safety */
    OSSemPost(handle->mutex);
    OSSemDel(handle->mutex, OS_DEL_ALWAYS, &err);

    return kStatus_Success;
}

status_t LPSPI_RTOS_Transfer(lpspi_rtos_handle_t *handle, lpspi_transfer_t *transfer)
{
    OS_FLAGS ev;
    uint8_t err;
    status_t status;

    /* Lock resource mutex */
    OSSemPend(handle->mutex, 0, &err);
    if (OS_ERR_NONE != err)
    {
        /* We could not take the semaphore, exit with 0 data received */
        return kStatus_LPSPI_Busy;
    }

    ev = OSFlagPost(handle->event, RTOS_LPSPI_COMPLETE, OS_FLAG_CLR, &err);
    assert((ev & RTOS_LPSPI_COMPLETE) == 0);

    status = LPSPI_MasterTransferNonBlocking(handle->base, &handle->drv_handle, transfer);
    if (status != kStatus_Success)
    {
        OSSemPost(handle->mutex);
        return status;
    }

    /* Wait for transfer to finish */
    ev = OSFlagPend(handle->event, RTOS_LPSPI_COMPLETE, OS_FLAG_WAIT_SET_ALL, 0, &err);
    if (!(ev & RTOS_LPSPI_COMPLETE))
    {
        OSSemPost(handle->mutex);
        return kStatus_Fail;
    }

    /* Unlock resource mutex */
    if (OS_ERR_NONE != OSSemPost(handle->mutex))
    {
        /* We could not post back the semaphore, exit with error */
        return kStatus_Fail;
    }

    /* Return status captured by callback function */
    return handle->async_status;
}
