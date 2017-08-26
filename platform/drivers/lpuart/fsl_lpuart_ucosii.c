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

#include "fsl_lpuart_ucosii.h"
#include <ucos_ii.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

static void LPUART_RTOS_Callback(LPUART_Type *base, lpuart_handle_t *state, status_t status, void *param)
{
    uint8_t err;
    lpuart_rtos_handle_t *handle = (lpuart_rtos_handle_t *)param;

    if (status == kStatus_LPUART_RxIdle)
    {
        OSFlagPost(handle->rx_event, RTOS_LPUART_COMPLETE, OS_FLAG_SET, &err);
    }
    else if (status == kStatus_LPUART_TxIdle)
    {
        OSFlagPost(handle->tx_event, RTOS_LPUART_COMPLETE, OS_FLAG_SET, &err);
    }
    else if (status == kStatus_LPUART_RxRingBufferOverrun)
    {
        OSFlagPost(handle->rx_event, RTOS_LPUART_RING_BUFFER_OVERRUN, OS_FLAG_SET, &err);
    }
    else if (status == kStatus_LPUART_RxHardwareOverrun)
    {
        /* Clear Overrun flag (OR) in LPUART STAT register */
        LPUART_ClearStatusFlags(base, kLPUART_RxOverrunFlag);
        OSFlagPost(handle->rx_event, RTOS_LPUART_HARDWARE_BUFFER_OVERRUN, OS_FLAG_SET, &err);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_RTOS_Init
 * Description   : Initializes the LPUART instance for application
 *
 *END**************************************************************************/
int LPUART_RTOS_Init(lpuart_rtos_handle_t *handle, lpuart_handle_t *t_handle, const struct rtos_lpuart_config *cfg)
{
    lpuart_config_t defcfg;
    uint8_t err;

    if (NULL == handle)
    {
        return kStatus_InvalidArgument;
    }
    if (NULL == t_handle)
    {
        return kStatus_InvalidArgument;
    }
    if (NULL == cfg)
    {
        return kStatus_InvalidArgument;
    }
    if (NULL == cfg->base)
    {
        return kStatus_InvalidArgument;
    }
    if (0 == cfg->srcclk)
    {
        return kStatus_InvalidArgument;
    }
    if (0 == cfg->baudrate)
    {
        return kStatus_InvalidArgument;
    }

    handle->base = cfg->base;
    handle->t_state = t_handle;

    handle->tx_sem = OSSemCreate(1);
    if (NULL == handle->tx_sem)
    {
        return kStatus_Fail;
    }
    handle->rx_sem = OSSemCreate(1);
    if (NULL == handle->rx_sem)
    {
        OSSemDel(handle->tx_sem, OS_DEL_ALWAYS, &err);
        return kStatus_Fail;
    }
    handle->tx_event = OSFlagCreate(0, &err);
    if (OS_ERR_NONE != err)
    {
        OSSemDel(handle->rx_sem, OS_DEL_ALWAYS, &err);
        OSSemDel(handle->tx_sem, OS_DEL_ALWAYS, &err);
        return kStatus_Fail;
    }
    handle->rx_event = OSFlagCreate(0, &err);
    if (OS_ERR_NONE != err)
    {
        OSFlagDel(handle->rx_event, OS_DEL_ALWAYS, &err);
        OSSemDel(handle->rx_sem, OS_DEL_ALWAYS, &err);
        OSSemDel(handle->tx_sem, OS_DEL_ALWAYS, &err);
        return kStatus_Fail;
    }

    LPUART_GetDefaultConfig(&defcfg);

    defcfg.baudRate_Bps = cfg->baudrate;
    defcfg.parityMode = cfg->parity;
#if defined(FSL_FEATURE_LPUART_HAS_STOP_BIT_CONFIG_SUPPORT) && FSL_FEATURE_LPUART_HAS_STOP_BIT_CONFIG_SUPPORT
    defcfg.stopBitCount = cfg->stopbits;
#endif

    LPUART_Init(handle->base, &defcfg, cfg->srcclk);
    LPUART_TransferCreateHandle(handle->base, handle->t_state, LPUART_RTOS_Callback, handle);
    LPUART_TransferStartRingBuffer(handle->base, handle->t_state, cfg->buffer, cfg->buffer_size);

    LPUART_EnableTx(handle->base, true);
    LPUART_EnableRx(handle->base, true);

    return 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_RTOS_Deinit
 * Description   : Deinitializes the LPUART instance and frees resources
 *
 *END**************************************************************************/
int LPUART_RTOS_Deinit(lpuart_rtos_handle_t *handle)
{
    uint8_t err;

    LPUART_Deinit(handle->base);

    OSFlagDel(handle->tx_event, OS_DEL_ALWAYS, &err);
    OSFlagDel(handle->rx_event, OS_DEL_ALWAYS, &err);

    /* Give the semaphore. This is for functional safety */
    OSSemPost(handle->tx_sem);
    OSSemPost(handle->rx_sem);

    OSSemDel(handle->tx_sem, OS_DEL_ALWAYS, &err);
    OSSemDel(handle->rx_sem, OS_DEL_ALWAYS, &err);

    /* Invalidate the handle */
    handle->t_state = NULL;
    handle->base = NULL;

    return 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_RTOS_Send
 * Description   : Initializes the LPUART instance for application
 *
 *END**************************************************************************/
int LPUART_RTOS_Send(lpuart_rtos_handle_t *handle, const uint8_t *buffer, uint32_t length)
{
    OS_FLAGS ev;
    uint8_t err;
    int retval = kStatus_Success;

    if (NULL == handle->base)
    {
        /* Invalid handle. */
        return kStatus_Fail;
    }
    if (0 == length)
    {
        return 0;
    }
    if (NULL == buffer)
    {
        return kStatus_InvalidArgument;
    }

    OSSemPend(handle->tx_sem, 0, &err);
    if (OS_ERR_NONE != err)
    {
        /* We could not take the semaphore, exit with 0 data received */
        return kStatus_Fail;
    }

    handle->tx_xfer.data = (uint8_t *)buffer;
    handle->tx_xfer.dataSize = (uint32_t)length;

    /* Non-blocking call */
    LPUART_TransferSendNonBlocking(handle->base, handle->t_state, &handle->tx_xfer);

    ev = OSFlagPend(handle->tx_event, RTOS_LPUART_COMPLETE, OS_FLAG_WAIT_SET_ALL + OS_FLAG_CONSUME, 0, &err);
    if (!(ev & RTOS_LPUART_COMPLETE))
    {
        retval = kStatus_Fail;
    }

    if (OS_ERR_NONE != OSSemPost(handle->tx_sem))
    {
        /* We could not post the semaphore, exit with error */
        retval = kStatus_Fail;
    }

    return retval;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_RTOS_Recv
 * Description   : Receives chars for the application
 *
 *END**************************************************************************/
int LPUART_RTOS_Receive(lpuart_rtos_handle_t *handle, uint8_t *buffer, uint32_t length, size_t *received)
{
    uint8_t err;
    OS_FLAGS ev;
    size_t n = 0;
    int retval = kStatus_Fail;
    size_t local_received = 0;

    if (NULL == handle->base)
    {
        /* Invalid handle. */
        return kStatus_Fail;
    }
    if (0 == length)
    {
        if (received != NULL)
        {
            *received = 0;
        }
        return 0;
    }
    if (NULL == buffer)
    {
        return kStatus_InvalidArgument;
    }

    /* New transfer can be performed only after current one is finished */
    OSSemPend(handle->rx_sem, 0, &err);
    if (OS_ERR_NONE != err)
    {
        /* We could not take the semaphore, exit with 0 data received */
        return kStatus_Fail;
    }

    handle->rx_xfer.data = buffer;
    handle->rx_xfer.dataSize = (uint32_t)length;

    /* Non-blocking call */
    LPUART_TransferReceiveNonBlocking(handle->base, handle->t_state, &handle->rx_xfer, &n);

    ev = OSFlagPend(handle->rx_event,
                    RTOS_LPUART_COMPLETE | RTOS_LPUART_RING_BUFFER_OVERRUN | RTOS_LPUART_HARDWARE_BUFFER_OVERRUN,
                    OS_FLAG_WAIT_SET_ANY + OS_FLAG_CONSUME, 0, &err);
    if (ev & RTOS_LPUART_HARDWARE_BUFFER_OVERRUN)
    {
        /* Stop data transfer to application buffer, ring buffer is still active */
        LPUART_TransferAbortReceive(handle->base, handle->t_state);
        /* Prevent false indication of successful transfer in next call of LPUART_RTOS_Receive.
           RTOS_LPUART_COMPLETE flag could be set meanwhile overrun is handled */
        OSFlagPost(handle->rx_event, RTOS_LPUART_COMPLETE, OS_FLAG_CLR, &err);
        retval = kStatus_LPUART_RxHardwareOverrun;
        local_received = 0;
    }
    else if (ev & RTOS_LPUART_RING_BUFFER_OVERRUN)
    {
        /* Stop data transfer to application buffer, ring buffer is still active */
        LPUART_TransferAbortReceive(handle->base, handle->t_state);
        /* Prevent false indication of successful transfer in next call of LPUART_RTOS_Receive.
           RTOS_LPUART_COMPLETE flag could be set meanwhile overrun is handled */
        OSFlagPost(handle->rx_event, RTOS_LPUART_COMPLETE, OS_FLAG_CLR, &err);
        retval = kStatus_LPUART_RxRingBufferOverrun;
        local_received = 0;
    }
    else if (ev & RTOS_LPUART_COMPLETE)
    {
        retval = kStatus_Success;
        local_received = length;
    }

    /* Prevent repetitive NULL check */
    if (received != NULL)
    {
        *received = local_received;
    }

    /* Enable next transfer. Current one is finished */
    if (OS_ERR_NONE != OSSemPost(handle->rx_sem))
    {
        /* We could not post the semaphore, exit with error */
        retval = kStatus_Fail;
    }
    return retval;
}
