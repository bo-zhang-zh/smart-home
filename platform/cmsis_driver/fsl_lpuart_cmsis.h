/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

#ifndef _FSL_LPUART_CMSIS_H_
#define _FSL_LPUART_CMSIS_H_

#include "fsl_common.h"
#include "Driver_USART.h"
#include "RTE_Device.h"

#if defined(LPUART0)
extern ARM_DRIVER_USART LPUART0_NonBlocking_Driver;
#endif /* LPUART0 */

#if defined(LPUART1)
extern ARM_DRIVER_USART LPUART1_NonBlocking_Driver;
#endif /* LPUART1 */

#if defined(LPUART2)
extern ARM_DRIVER_USART LPUART2_NonBlocking_Driver;
#endif /* LPUART2 */

#if defined(LPUART3)
extern ARM_DRIVER_USART LPUART3_NonBlocking_Driver;
#endif /* LPUART3 */

#if defined(LPUART4)
extern ARM_DRIVER_USART LPUART4_NonBlocking_Driver;
#endif /* LPUART4 */

#if defined(LPUART5)
extern ARM_DRIVER_USART LPUART5_NonBlocking_Driver;
#endif /* LPUART5 */

#if (defined(FSL_FEATURE_SOC_EDMA_COUNT) && FSL_FEATURE_SOC_EDMA_COUNT)

#if defined(LPUART0)
extern ARM_DRIVER_USART LPUART0_Edma_Driver;
#endif /* LPUART0 */

#if defined(LPUART1)
extern ARM_DRIVER_USART LPUART1_Edma_Driver;
#endif /* LPUART1 */

#if defined(LPUART2)
extern ARM_DRIVER_USART LPUART2_Edma_Driver;
#endif /* LPUART2 */

#if defined(LPUART3)
extern ARM_DRIVER_USART LPUART3_Edma_Driver;
#endif /* LPUART3 */

#if defined(LPUART4)
extern ARM_DRIVER_USART LPUART4_Edma_Driver;
#endif /* LPUART4 */

#if defined(LPUART5)
extern ARM_DRIVER_USART LPUART5_Edma_Driver;
#endif /* LPUART5 */

#endif /* FSL_FEATURE_SOC_EDMA_COUNT */

#if (defined(FSL_FEATURE_SOC_DMA_COUNT) && FSL_FEATURE_SOC_DMA_COUNT)

#if defined(LPUART0)
extern ARM_DRIVER_USART LPUART0_Dma_Driver;
#endif /* LPUART0 */

#if defined(LPUART1)
extern ARM_DRIVER_USART LPUART1_Dma_Driver;
#endif /* LPUART1 */

#if defined(LPUART2)
extern ARM_DRIVER_USART LPUART2_Dma_Driver;
#endif /* LPUART2 */

#if defined(LPUART3)
extern ARM_DRIVER_USART LPUART3_Dma_Driver;
#endif /* LPUART3 */

#if defined(LPUART4)
extern ARM_DRIVER_USART LPUART4_Dma_Driver;
#endif /* LPUART4 */

#if defined(LPUART5)
extern ARM_DRIVER_USART LPUART5_Dma_Driver;
#endif /* LPUART5 */

#endif /* FSL_FEATURE_SOC_DMA_COUNT */

#endif /* _FSL_LPUART_CMSIS_H_ */
