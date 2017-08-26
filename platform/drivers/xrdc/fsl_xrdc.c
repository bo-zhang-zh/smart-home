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

#include "fsl_xrdc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define XRDC_DERR_W1_EST_VAL(w1) ((w1 & XRDC_DERR_W_EST_MASK) >> XRDC_DERR_W_EST_SHIFT)
#define XRDC_DERR_W1_EPORT_VAL(w1) ((w1 & XRDC_DERR_W_EPORT_MASK) >> XRDC_DERR_W_EPORT_SHIFT)
#define XRDC_DERR_W1_ERW_VAL(w1) ((w1 & XRDC_DERR_W_ERW_MASK) >> XRDC_DERR_W_ERW_SHIFT)
#define XRDC_DERR_W1_EATR_VAL(w1) ((w1 & XRDC_DERR_W_EATR_MASK) >> XRDC_DERR_W_EATR_SHIFT)
#define XRDC_DERR_W1_EDID_VAL(w1) ((w1 & XRDC_DERR_W_EDID_MASK) >> XRDC_DERR_W_EDID_SHIFT)

#define XRDC_MRGD_DXACP_WIDTH (3U) /* The width of XRDC_MRDG_DxACP. */
#define XRDC_PDAC_DXACP_WIDTH (3U) /* The width of XRDC_PDAC_DxACP. */

/*******************************************************************************
 * Code
 ******************************************************************************/

#if ((__CORTEX_M == 0U) && (defined(__ICCARM__)))
/*!
 * @brief Count the leading zeros.
 *
 * Count the leading zeros of an 32-bit data. This function is only defined
 * for CM0 and CM0+ for IAR, because other cortex series have the clz instruction,
 * KEIL and ARMGCC have toolchain build in function for this purpose.
 *
 * @param data The data to process.
 * @return Count of the leading zeros.
 */
static uint32_t XRDC_CountLeadingZeros(uint32_t data)
{
    uint32_t count = 0U;
    uint32_t mask = 0x80000000U;

    while ((data & mask) == 0U)
    {
        count++;
        mask >>= 1U;
    }

    return count;
}
#endif

void XRDC_GetHardwareConfig(XRDC_Type *base, xrdc_hardware_config_t *config)
{
    assert(config);

    config->masterNumber = ((base->HWCFG0 & XRDC_HWCFG0_NMSTR_MASK) >> XRDC_HWCFG0_NMSTR_SHIFT) + 1U;
    config->domainNumber = ((base->HWCFG0 & XRDC_HWCFG0_NDID_MASK) >> XRDC_HWCFG0_NDID_SHIFT) + 1U;
    config->pacNumber = ((base->HWCFG0 & XRDC_HWCFG0_NPAC_MASK) >> XRDC_HWCFG0_NPAC_SHIFT) + 1U;
    config->mrcNumber = ((base->HWCFG0 & XRDC_HWCFG0_NMRC_MASK) >> XRDC_HWCFG0_NMRC_SHIFT) + 1U;
}

status_t XRDC_GetAndClearFirstDomainError(XRDC_Type *base, xrdc_error_t *error)
{
    assert(error);

    uint8_t did;          /* Domain ID.                       */
    uint32_t errorBitMap; /* Domain error location bit map.   */
    uint32_t errorIndex;  /* The index of first domain error. */
    uint32_t regW1;       /* To save XRDC_DERR_W1.            */

    /* Get current domain ID. */
    did = XRDC_GetCurrentMasterDomainId(base);

    /* Get the error bitmap. */
    errorBitMap = base->DERRLOC[did];

    if (!errorBitMap) /* No error captured. */
    {
        return kStatus_XRDC_NoError;
    }

/* Get the first error controller index. */
#if ((__CORTEX_M == 0U) && (defined(__ICCARM__)))
    errorIndex = 31U - XRDC_CountLeadingZeros(errorBitMap);
#else
    errorIndex = 31U - __CLZ(errorBitMap);
#endif

    /* Get the error information. */
    regW1 = base->DERR_W[errorIndex][1];
    error->controller = (xrdc_controller_t)errorIndex;
    error->address = base->DERR_W[errorIndex][0];
    error->errorState = (xrdc_error_state_t)XRDC_DERR_W1_EST_VAL(regW1);
    error->errorAttr = (xrdc_error_attr_t)XRDC_DERR_W1_EATR_VAL(regW1);
    error->errorType = (xrdc_error_type_t)XRDC_DERR_W1_ERW_VAL(regW1);
    error->errorPort = XRDC_DERR_W1_EPORT_VAL(regW1);
    error->domainId = XRDC_DERR_W1_EDID_VAL(regW1);

    /* Clear error pending. */
    base->DERR_W[errorIndex][3] = XRDC_DERR_W_RECR(0x01U);

    return kStatus_Success;
}

void XRDC_GetMemAccessDefaultConfig(xrdc_mem_access_config_t *config)
{
    uint8_t i;

    config->enableSema = false;
    config->semaNum = 0U;
    config->subRegionDisableMask = 0U;
    config->size = kXRDC_MemSizeNone;
    config->lockMode = kXRDC_AccessConfigLockWritable;
    config->baseAddress = 0U;

    for (i = 0U; i < 16U; i++)
    {
        config->policy[i] = kXRDC_AccessPolicyNone;
    }
}

void XRDC_SetMemAccessConfig(XRDC_Type *base, const xrdc_mem_access_config_t *config)
{
    uint32_t i;
    uint32_t regValue;
    uint8_t index = (uint8_t)config->mem;

    assert(config);
    /* Not allowed to set sub-region disable mask for memory region smaller than 256-bytes. */
    assert(!((config->size < kXRDC_MemSize256B) && (config->subRegionDisableMask)));
    /* Memory region minimum size = 32 bytes and base address must be aligned to 0-module-2**(SZ+1). */
    assert(config->size >= kXRDC_MemSize32B);
    assert(!(config->baseAddress & ((1U << (config->size + 1U)) - 1U)));

    base->MRGD_W[index][0] = config->baseAddress;
    base->MRGD_W[index][1] = XRDC_MRGD_W_SZ(config->size) | XRDC_MRGD_W_SRD(config->subRegionDisableMask);

    /* Set MRGD_W2[D0ACP ~ D7ACP]. */
    regValue = 0U;
    i = 8U;
    while (i--)
    {
        regValue <<= XRDC_MRGD_DXACP_WIDTH;
        regValue |= config->policy[i];
    }
    /* Set MRGD_W2. */
    base->MRGD_W[index][2] = regValue | XRDC_MRGD_W_SE(config->enableSema) | XRDC_MRGD_W_SNUM(config->semaNum);

    /* Set MRGD_W3[D8ACP ~ D15ACP]. */
    regValue = 0U;
    for (i = 15U; i > 7U; i--)
    {
        regValue <<= XRDC_MRGD_DXACP_WIDTH;
        regValue |= config->policy[i];
    }
    /* Set MRGD_W3. */
    base->MRGD_W[index][3] = regValue | XRDC_MRGD_W_VLD_MASK | XRDC_MRGD_W_LK2(config->lockMode);
}

void XRDC_GetPeriphAccessDefaultConfig(xrdc_periph_access_config_t *config)
{
    uint8_t i;

    config->enableSema = false;
    config->semaNum = 0U;
    config->lock = kXRDC_AccessConfigLockWritable;

    for (i = 0U; i < 16U; i++)
    {
        config->policy[i] = kXRDC_AccessPolicyNone;
    }
}

void XRDC_SetPeriphAccessConfig(XRDC_Type *base, const xrdc_periph_access_config_t *config)
{
    uint32_t i;
    uint32_t regValue;
    uint8_t index = (uint8_t)config->periph;

    /* Set PDAC_W2[D0ACP ~ D7ACP]. */
    regValue = 0U;
    i = 8U;
    while (i--)
    {
        regValue <<= XRDC_MRGD_DXACP_WIDTH;
        regValue |= config->policy[i];
    }
    /* Set PDAC_W0. */
    base->PDAC_W[index][0U] = regValue | XRDC_PDAC_W_SE(config->enableSema) | XRDC_PDAC_W_SNUM(config->semaNum);

    /* Set PDAC_W3[D8ACP ~ D15ACP]. */
    regValue = 0U;
    for (i = 15U; i > 7U; i--)
    {
        regValue <<= XRDC_MRGD_DXACP_WIDTH;
        regValue |= config->policy[i];
    }
    /* Set PDAC_W1. */
    base->PDAC_W[index][1] = regValue | XRDC_PDAC_W_VLD_MASK | XRDC_PDAC_W_LK2(config->lock);
}
