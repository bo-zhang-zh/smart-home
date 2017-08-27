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

#include "fsl_pwt.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Gets the instance from the base address
 *
 * @param base PWT peripheral base address
 *
 * @return The PWT instance
 */
static uint32_t PWT_GetInstance(PWT_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to PWT bases for each instance. */
static PWT_Type *const s_pwtBases[] = PWT_BASE_PTRS;

/*! @brief Pointers to PWT clocks for each instance. */
static const clock_ip_name_t s_pwtClocks[] = PWT_CLOCKS;

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t PWT_GetInstance(PWT_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < FSL_FEATURE_SOC_PWT_COUNT; instance++)
    {
        if (s_pwtBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < FSL_FEATURE_SOC_PWT_COUNT);

    return instance;
}

void PWT_Init(PWT_Type *base, const pwt_config_t *config)
{
    assert(config);

    /* Ungate the PWT module clock*/
    CLOCK_EnableClock(s_pwtClocks[PWT_GetInstance(base)]);

    /* Reset the module */
    PWT_Reset(base);

    /*
     * Clear all interrupt, disable the counter, config the first counter load enable bit,
     * enable module interrupt bit.
     */
    base->CS = PWT_CS_FCTLE(config->enableFirstCounterLoad) | PWT_CS_PWTIE_MASK;

    /* Set clock source, prescale and input source */
    base->CR = PWT_CR_PCLKS(config->clockSource) | PWT_CR_PRE(config->prescale) | PWT_CR_PINSEL(config->inputSelect);
}

void PWT_Deinit(PWT_Type *base)
{
    /* Disable the counter */
    base->CS &= ~PWT_CS_PWTEN_MASK;

    /* Gate the PWT clock */
    CLOCK_DisableClock(s_pwtClocks[PWT_GetInstance(base)]);
}

void PWT_GetDefaultConfig(pwt_config_t *config)
{
    assert(config);

    /* Use the IP Bus clock as source clock for the PWT submodule */
    config->clockSource = kPWT_BusClock;
    /* Clock source prescale is set to divide by 1*/
    config->prescale = kPWT_Prescale_Divide_1;
    /* PWT input signal coming from Port 0 */
    config->inputSelect = kPWT_InputPort_0;
    /* Do not load the first counter values to the corresponding registers */
    config->enableFirstCounterLoad = false;
}
