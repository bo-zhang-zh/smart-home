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

#include "fsl_lpadc.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Get instance number for LPADC module.
 *
 * @param base LPADC peripheral base address
 */
static uint32_t LPADC_GetInstance(ADC_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to LPADC bases for each instance. */
static ADC_Type *const s_lpadcBases[] = ADC_BASE_PTRS;
/*! @brief Pointers to LPADC clocks for each instance. */
static const clock_ip_name_t s_lpadcClocks[] = LPADC_CLOCKS;

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t LPADC_GetInstance(ADC_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < FSL_FEATURE_SOC_LPADC_COUNT; instance++)
    {
        if (s_lpadcBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < FSL_FEATURE_SOC_LPADC_COUNT);

    return instance;
}

void LPADC_Init(ADC_Type *base, const lpadc_config_t *config)
{
    /* Check if the pointer is available. */
    assert(config != NULL);

    uint32_t tmp32 = 0U;

    /* Enable the clock for LPADC instance. */
    CLOCK_EnableClock(s_lpadcClocks[LPADC_GetInstance(base)]);

    /* Reset the module. */
    LPADC_DoResetConfig(base);
    LPADC_DoResetFIFO(base);

    /* Disable the module before setting configuration. */
    LPADC_Enable(base, false);

    /* Configure the module generally. */
    if (config->enableInDozeMode)
    {
        base->CTRL &= ~ADC_CTRL_DOZEN_MASK;
    }
    else
    {
        base->CTRL |= ADC_CTRL_DOZEN_MASK;
    }

    /* ADCx_CFG. */
    if (config->enableAnalogPreliminary)
    {
        tmp32 |= ADC_CFG_PWREN_MASK;
    }
    tmp32 |= ADC_CFG_PUDLY(config->powerUpDelay)                /* Power up delay. */
             | ADC_CFG_REFSEL(config->referenceVoltageSource)   /* Reference voltage. */
             | ADC_CFG_TPRICTRL(config->triggerPrioirtyPolicy); /* Trigger priority policy. */
    base->CFG = tmp32;

    /* ADCx_PAUSE. */
    if (config->enableConvPause)
    {
        base->PAUSE = ADC_PAUSE_PAUSEEN_MASK | ADC_PAUSE_PAUSEDLY(config->convPauseDelay);
    }
    else
    {
        base->PAUSE = 0U;
    }

    /* ADCx_FCTRL. */
    base->FCTRL = ADC_FCTRL_FWMARK(config->FIFOWatermark);

    /* Enable the module after setting configuration. */
    LPADC_Enable(base, true);
}

void LPADC_GetDefaultConfig(lpadc_config_t *config)
{
    config->enableInDozeMode = true;
    config->enableAnalogPreliminary = false;
    config->powerUpDelay = 0x80;
    config->referenceVoltageSource = kLPADC_ReferenceVoltageAlt1;
    config->powerLevelMode = kLPADC_PowerLevelAlt1;
    config->triggerPrioirtyPolicy = kLPADC_TriggerPriorityPreemptImmediately;
    config->enableConvPause = false;
    config->convPauseDelay = 0U;
    config->FIFOWatermark = 0U;
}

void LPADC_Deinit(ADC_Type *base)
{
    /* Disable the module. */
    LPADC_Enable(base, false);

    /* Gate the clock. */
    CLOCK_DisableClock(s_lpadcClocks[LPADC_GetInstance(base)]);
}

bool LPADC_GetConvResult(ADC_Type *base, lpadc_conv_result_t *result)
{
    assert(result != NULL); /* Check if the input pointer is available. */

    uint32_t tmp32;

    tmp32 = base->RESFIFO;

    if (0U == (ADC_RESFIFO_VALID_MASK & tmp32))
    {
        return false; /* FIFO is empty. Discard any read from RESFIFO. */
    }

    result->commandIdSource = (tmp32 & ADC_RESFIFO_CMDSRC_MASK) >> ADC_RESFIFO_CMDSRC_SHIFT;
    result->loopCountIndex = (tmp32 & ADC_RESFIFO_LOOPCNT_MASK) >> ADC_RESFIFO_LOOPCNT_SHIFT;
    result->triggerIdSource = (tmp32 & ADC_RESFIFO_TSRC_MASK) >> ADC_RESFIFO_TSRC_SHIFT;
    result->convValue = (uint16_t)(tmp32 & ADC_RESFIFO_D_MASK);

    return true;
}

void LPADC_SetConvTriggerConfig(ADC_Type *base, uint32_t triggerId, const lpadc_conv_trigger_config_t *config)
{
    assert(triggerId < ADC_TCTRL_COUNT); /* Check if the triggerId is available in this device. */
    assert(config != NULL);              /* Check if the input pointer is available. */

    uint32_t tmp32;

    tmp32 = ADC_TCTRL_TCMD(config->targetCommandId) /* Trigger command select. */
            | ADC_TCTRL_TDLY(config->delayPower)    /* Trigger delay select. */
            | ADC_TCTRL_TPRI(config->priority);     /* Trigger priority setting. */
    if (config->enableHardwareTrigger)
    {
        tmp32 |= ADC_TCTRL_HTEN_MASK;
    }

    base->TCTRL[triggerId] = tmp32;
}

void LPADC_GetDefaultConvTriggerConfig(lpadc_conv_trigger_config_t *config)
{
    assert(config != NULL); /* Check if the input pointer is available. */

    config->targetCommandId = 0U;
    config->delayPower = 0U;
    config->priority = 0U;
    config->enableHardwareTrigger = false;
}

void LPADC_SetConvCommandConfig(ADC_Type *base, uint32_t commandId, const lpadc_conv_command_config_t *config)
{
    assert(commandId < (ADC_CMDL_COUNT + 1U)); /* Check if the commandId is available on this device. */
    assert(config != NULL);                    /* Check if the input pointer is available. */

    uint32_t tmp32;

    commandId--; /* The available command number are 1-15, while the index of register group are 0-14. */

    /* ADCx_CMDL. */
    tmp32 = ADC_CMDL_CSCALE(config->sampleScaleMode) /* Full/Part scale input voltage. */
            | ADC_CMDL_ADCH(config->channelNumber);  /* Channel number. */
    switch (config->sampleChannelMode)               /* Sample input. */
    {
        case kLPADC_SampleChannelSingleEndSideB:
            tmp32 |= ADC_CMDL_ABSEL_MASK;
            break;
        case kLPADC_SampleChannelDiffBothSideAB:
            tmp32 |= ADC_CMDL_DIFF_MASK;
            break;
        case kLPADC_SampleChannelDiffBothSideBA:
            tmp32 |= ADC_CMDL_ABSEL_MASK | ADC_CMDL_DIFF_MASK;
            break;
        default: /* kLPADC_SampleChannelSingleEndSideA. */
            break;
    }
    base->CMD[commandId].CMDL = tmp32;

    /* ADCx_CMDH. */
    tmp32 = ADC_CMDH_NEXT(config->chainedNextCommandNumber) /* Next Command Select. */
            | ADC_CMDH_LOOP(config->loopCount)              /* Loop Count Select. */
            | ADC_CMDH_AVGS(config->hardwareAverageMode)    /* Hardware Average Select. */
            | ADC_CMDH_STS(config->sampleTimeMode)          /* Sample Time Select. */
            | ADC_CMDH_CMPEN(config->hardwareCompareMode);  /* Hardware compare enable. */
    if (config->enableAutoChannelIncrement)
    {
        tmp32 |= ADC_CMDH_LWI_MASK;
    }
    base->CMD[commandId].CMDH = tmp32;

    /* Hardware compare settings.
    * Not all Command Buffers have an associated Compare Value register. The compare function is only available on
    * Command Buffers that have a corresponding Compare Value register.
    */
    if (kLPADC_HardwareCompareDisabled != config->hardwareCompareMode)
    {
        /* Check if the hardware compare feature is available for indicated command buffer. */
        assert(commandId < ADC_CV_COUNT);

        /* Set CV register. */
        base->CV[commandId] = ADC_CV_CVH(config->hardwareCompareValueHigh)   /* Compare value high. */
                              | ADC_CV_CVL(config->hardwareCompareValueLow); /* Compare value low. */
    }
}

void LPADC_GetDefaultConvCommandConfig(lpadc_conv_command_config_t *config)
{
    assert(config != NULL); /* Check if the input pointer is available. */

    config->sampleScaleMode = kLPADC_SampleFullScale;
    config->sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
    config->channelNumber = 0U;
    config->chainedNextCommandNumber = 0U; /* No next command defined. */
    config->enableAutoChannelIncrement = false;
    config->loopCount = 0U;
    config->hardwareAverageMode = kLPADC_HardwareAverageCount1;
    config->sampleTimeMode = kLPADC_SampleTimeADCK3;
    config->hardwareCompareMode = kLPADC_HardwareCompareDisabled;
    config->hardwareCompareValueHigh = 0U; /* No used. */
    config->hardwareCompareValueLow = 0U;  /* No used. */
}
