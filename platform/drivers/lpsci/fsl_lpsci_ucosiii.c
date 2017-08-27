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

#include "fsl_lpsci_ucosiii.h"
#include <os.h>
#include <lib_mem.h>

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

static void LPSCI_RTOS_Callback(UART0_Type *base, lpsci_handle_t *state, status_t status, void *param)
{
    OS_ERR err;
    lpsci_rtos_handle_t *handle = (lpsci_rtos_handle_t *)param;

    if (status == kStatus_LPSCI_RxIdle)
    {
        OSFlagPost(&handle->rx_event, RTOS_LPSCI_COMPLETE, OS_OPT_POST_FLAG_SET, &err);
    }
    else if (status == kStatus_LPSCI_TxIdle)
    {
        OSFlagPost(&handle->tx_event, RTOS_LPSCI_COMPLETE, OS_OPT_POST_FLAG_SET, &err);
    }
    else if (status == kStatus_LPSCI_RxRingBufferOverrun)
    {
        OSFlagPost(&handle->rx_event, RTOS_LPSCI_RING_BUFFER_OVERRUN, OS_OPT_POST_FLAG_SET, &err);
    }
    else if (status == kStatus_LPSCI_RxHardwareOverrun)
    {
        LPSCI_ClearStatusFlags(base, kLPSCI_RxOverrunFlag);
        OSFlagPost(&handle->rx_event, RTOS_LPSCI_HARDWARE_BUFFER_OVERRUN, OS_OPT_POST_FLAG_SET, &err);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSCI_RTOS_Init
 * Description   : Initializes the LPSCI instance for application
 *
 *END**************************************************************************/
int LPSCI_RTOS_Init(lpsci_rtos_handle_t *handle, lpsci_handle_t *t_handle, const struct rtos_lpsci_config *cfg)
{
    lpsci_config_t defcfg;
    OS_ERR err;

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

    OSSemCreate(&handle->tx_sem, "LPSCI TX", (OS_SEM_CTR)1, &err);
    if (OS_ERR_NONE != err)
    {
        return kStatus_Fail;
    }
    OSSemCreate(&handle->rx_sem, "LPSCI TX", (OS_SEM_CTR)1, &err);
    if (OS_ERR_NONE != err)
    {
#if (OS_CFG_SEM_DEL_EN == DEF_ENABLED)
        OSSemDel(&handle->tx_sem, OS_OPT_DEL_ALWAYS, &err);
#endif
        return kStatus_Fail;
    }
    OSFlagCreate(&handle->tx_event, "LPSCI TX", (OS_FLAGS)0, &err);
    if (OS_ERR_NONE != err)
    {
#if (OS_CFG_SEM_DEL_EN == DEF_ENABLED)
        OSSemDel(&handle->rx_sem, OS_OPT_DEL_ALWAYS, &err);
        OSSemDel(&handle->tx_sem, OS_OPT_DEL_ALWAYS, &err);
#endif
        return kStatus_Fail;
    }
    OSFlagCreate(&handle->rx_event, "LPSCI RX", (OS_FLAGS)0, &err);
    if (OS_ERR_NONE != err)
    {
#if (OS_CFG_FLAG_DEL_EN == DEF_ENABLED)
        OSFlagDel(&handle->rx_event, OS_OPT_DEL_ALWAYS, &err);
#endif
#if (OS_CFG_SEM_DEL_EN == DEF_ENABLED)
        OSSemDel(&handle->rx_sem, OS_OPT_DEL_ALWAYS, &err);
        OSSemDel(&handle->tx_sem, OS_OPT_DEL_ALWAYS, &err);
#endif
        return kStatus_Fail;
    }

    LPSCI_GetDefaultConfig(&defcfg);

    defcfg.baudRate_Bps = cfg->baudrate;
    defcfg.parityMode = cfg->parity;
#if defined(FSL_FEATURE_LPSCI_HAS_STOP_BIT_CONFIG_SUPPORT) && FSL_FEATURE_LPSCI_HAS_STOP_BIT_CONFIG_SUPPORT
    defcfg.stopBitCount = cfg->stopbits;
#endif

    LPSCI_Init(handle->base, &defcfg, cfg->srcclk);
    LPSCI_TransferCreateHandle(handle->base, handle->t_state, LPSCI_RTOS_Callback, handle);
    LPSCI_TransferStartRingBuffer(handle->base, handle->t_state, cfg->buffer, cfg->buffer_size);

    LPSCI_EnableTx(handle->base, true);
    LPSCI_EnableRx(handle->base, true);

    return 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSCI_RTOS_Deinit
 * Description   : Deinitializes the LPSCI instance and frees resources
 *
 *END**************************************************************************/
int LPSCI_RTOS_Deinit(lpsci_rtos_handle_t *handle)
{
    LPSCI_Deinit(handle->base);

#if (OS_CFG_FLAG_DEL_EN == DEF_ENABLED)
    OSFlagDel(&handle->tx_event, OS_OPT_DEL_ALWAYS, &err);
    OSFlagDel(&handle->rx_event, OS_OPT_DEL_ALWAYS, &err);
#endif

#if (OS_CFG_SEM_DEL_EN == DEF_ENABLED)
    OSSemDel(&handle->tx_sem, OS_OPT_DEL_ALWAYS, &err);
    OSSemDel(&handle->rx_sem, OS_OPT_DEL_ALWAYS, &err);
#endif

    /* Invalidate the handle */
    handle->t_state = NULL;

    return 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSCI_RTOS_Send
 * Description   : Initializes the LPSCI instance for application
 *
 *END**************************************************************************/
int LPSCI_RTOS_Send(lpsci_rtos_handle_t *handle, const uint8_t *buffer, uint32_t length)
{
    OS_FLAGS ev;
    OS_ERR err;
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

    OSSemPend(&handle->tx_sem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    if (OS_ERR_NONE != err)
    {
        /* We could not take the semaphore, exit with 0 data received */
        return kStatus_Fail;
    }

    handle->tx_xfer.data = (uint8_t *)buffer;
    handle->tx_xfer.dataSize = (uint32_t)length;

    /* Non-blocking call */
    LPSCI_TransferSendNonBlocking(handle->base, handle->t_state, &handle->tx_xfer);

    ev = OSFlagPend(&handle->tx_event, RTOS_LPSCI_COMPLETE, 0, OS_OPT_PEND_FLAG_SET_ALL + OS_OPT_PEND_FLAG_CONSUME,
                    NULL, &err);
    if (!(ev & RTOS_LPSCI_COMPLETE))
    {
        retval = kStatus_Fail;
    }

    OSSemPost(&handle->tx_sem, OS_OPT_POST_1, &err);
    if (OS_ERR_NONE != err)
    {
        /* We could not post the semaphore, exit with error */
        retval = kStatus_Fail;
    }

    return retval;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPSCI_RTOS_Recv
 * Description   : Receives chars for the application
 *
 *END**************************************************************************/
int LPSCI_RTOS_Receive(lpsci_rtos_handle_t *handle, uint8_t *buffer, uint32_t length, size_t *received)
{
    OS_ERR err;
    OS_FLAGS ev;
    uint32_t ticks;
    size_t n = 0;
    int retval = kStatus_Success;
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
    OSSemPend(&handle->rx_sem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    if (OS_ERR_NONE != err)
    {
        /* We could not take the semaphore, exit with 0 data received */
        return kStatus_Fail;
    }

    handle->rx_xfer.data = buffer;
    handle->rx_xfer.dataSize = (uint32_t)length;

    /* Non-blocking call */
    LPSCI_TransferReceiveNonBlocking(handle->base, handle->t_state, &handle->rx_xfer, &n);

    ev = OSFlagPend(&handle->rx_event,
                    RTOS_LPSCI_COMPLETE | RTOS_LPSCI_RING_BUFFER_OVERRUN | RTOS_LPSCI_HARDWARE_BUFFER_OVERRUN, 0,
                    OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME, NULL, &err);
    if (ev & RTOS_LPSCI_HARDWARE_BUFFER_OVERRUN)
    {
        /* Stop data transfer to application buffer, ring buffer is still active */
        LPSCI_TransferAbortReceive(handle->base, handle->t_state);
        /* Prevent false indication of successful transfer in next call of LPSCI_RTOS_Receive.
           RTOS_LPSCI_COMPLETE flag could be set meanwhile overrun is handled */
        OSFlagPost(&handle->rx_event, RTOS_LPSCI_COMPLETE, OS_OPT_POST_FLAG_CLR, &err);
        retval = kStatus_LPSCI_RxHardwareOverrun;
        local_received = 0;
    }
    else if (ev & RTOS_LPSCI_RING_BUFFER_OVERRUN)
    {
        /* Stop data transfer to application buffer, ring buffer is still active */
        LPSCI_TransferAbortReceive(handle->base, handle->t_state);
        /* Prevent false indication of successful transfer in next call of LPSCI_RTOS_Receive.
           RTOS_LPSCI_COMPLETE flag could be set meanwhile overrun is handled */
        OSFlagPost(&handle->rx_event, RTOS_LPSCI_COMPLETE, OS_OPT_POST_FLAG_CLR, &err);
        retval = kStatus_LPSCI_RxRingBufferOverrun;
        local_received = 0;
    }
    else if (ev & RTOS_LPSCI_COMPLETE)
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
    OSSemPost(&handle->rx_sem, OS_OPT_POST_1, &err);
    if (OS_ERR_NONE != err)
    {
        /* We could not post the semaphore, exit with error */
        retval = kStatus_Fail;
    }
    return retval;
}
