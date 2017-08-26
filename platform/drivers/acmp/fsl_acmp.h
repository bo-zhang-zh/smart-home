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

#ifndef _FSL_ACMP_H_
#define _FSL_ACMP_H_

#include "fsl_common.h"

/*!
 * @addtogroup acmp
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief ACMP driver version 2.0.2. */
#define FSL_ACMP_DRIVER_VERSION (MAKE_VERSION(2U, 0U, 2U))
/*@}*/

/*! @brief Interrupt enable/disable mask. */
enum _acmp_interrupt_enable
{
    kACMP_OutputRisingInterruptEnable = (1U << 0U),  /*!< Enable the interrupt when comparator outputs rising. */
    kACMP_OutputFallingInterruptEnable = (1U << 1U), /*!< Enable the interrupt when comparator outputs falling. */
    kACMP_RoundRobinInterruptEnable = (1U << 2U),    /*!< Enable the Round-Robin interrupt. */
};

/*! @brief Status flag mask. */
enum _acmp_status_flags
{
    kACMP_OutputRisingEventFlag = CMP_C0_CFR_MASK,  /*!< Rising-edge on compare output has occurred. */
    kACMP_OutputFallingEventFlag = CMP_C0_CFF_MASK, /*!< Falling-edge on compare output has occurred. */
    kACMP_OutputAssertEventFlag = CMP_C0_COUT_MASK, /*!< Return the current value of the analog comparator output. */
};

/*!
 * @brief Comparator hard block offset control.
 *
 * If OFFSET level is 1, then there is no hysteresis in the case of positive port input crossing negative port
 * input in the positive direction (or negative port input crossing positive port input in the negative direction).
 * Hysteresis still exists for positive port input crossing negative port input in the falling direction.
 * If OFFSET level is 0, then the hysteresis selected by acmp_hysteresis_mode_t is valid for both directions.
 */
typedef enum _acmp_offset_mode
{
    kACMP_OffsetLevel0 = 0U, /*!< The comparator hard block output has level 0 offset internally. */
    kACMP_OffsetLevel1 = 1U, /*!< The comparator hard block output has level 1 offset internally. */
} acmp_offset_mode_t;

/*!
 * @brief Comparator hard block hysteresis control.
 *
 * See chip data sheet to get the actual hysteresis value with each level.
 */
typedef enum _acmp_hysteresis_mode
{
    kACMP_HysteresisLevel0 = 0U, /*!< Offset is level 0 and Hysteresis is level 0. */
    kACMP_HysteresisLevel1 = 1U, /*!< Offset is level 0 and Hysteresis is level 1. */
    kACMP_HysteresisLevel2 = 2U, /*!< Offset is level 0 and Hysteresis is level 2. */
    kACMP_HysteresisLevel3 = 3U, /*!< Offset is level 0 and Hysteresis is level 3. */
} acmp_hysteresis_mode_t;

/*! @brief CMP Voltage Reference source. */
typedef enum _acmp_reference_voltage_source
{
    kACMP_VrefSourceVin1 = 0U, /*!< Vin1 is selected as resistor ladder network supply reference Vin. */
    kACMP_VrefSourceVin2 = 1U, /*!< Vin2 is selected as resistor ladder network supply reference Vin. */
} acmp_reference_voltage_source_t;

/*! @brief Port input source */
typedef enum _acmp_port_input
{
    kACMP_PortInputFromDAC = 0U, /*!< Port input from the 8-bit DAC output. */
    kACMP_PortInputFromMux = 1U, /*!< Port input from the analog 8-1 mux. */
} acmp_port_input_t;

/*! @brief Fixed mux port */
typedef enum _acmp_fixed_port
{
    kACMP_FixedPlusPort = 0U,  /*!< Only the inputs to the Minus port are swept in each round. */
    kACMP_FixedMinusPort = 1U, /*!< Only the inputs to the Plus port are swept in each round. */
} acmp_fixed_port_t;

/*! @brief Configuration for ACMP. */
typedef struct _acmp_config
{
    acmp_offset_mode_t offsetMode;         /*!< Offset mode. */
    acmp_hysteresis_mode_t hysteresisMode; /*!< Hysteresis mode. */
    bool enableHighSpeed;                  /*!< Enable High Speed (HS) comparison mode. */
    bool enableInvertOutput;               /*!< Enable inverted comparator output. */
    bool useUnfilteredOutput;              /*!< Set compare output(COUT) to equal COUTA(true) or COUT(false). */
    bool enablePinOut;                     /*!< The comparator output is available on the associated pin. */
} acmp_config_t;

/*!
 * @brief Configuration for channel.
 *
 * The comparator's port can be input from channel mux or DAC. If port input is from channel mux, detailed channel
 * number for the mux should be configured.
 */
typedef struct _acmp_channel_config
{
    acmp_port_input_t positivePortInput; /*!< Input source of the comparator's positive port. */
    uint32_t plusMuxInput; /*!< Plus mux input channel(0~7), only valid when positive port input isn't from DAC. */
    acmp_port_input_t negativePortInput; /*!< Input source of the comparator's negative port. */
    uint32_t minusMuxInput; /*!< Minus mux input channel(0~7), only valid when negative port input isn't from DAC */
} acmp_channel_config_t;

/*! @brief Configuration for filter. */
typedef struct _acmp_filter_config
{
    bool enableSample;     /*!< Using external SAMPLE as sampling clock input, or using divided bus clock. */
    uint32_t filterCount;  /*!< Filter Sample Count. Available range is 1-7, 0 would cause the filter disabled. */
    uint32_t filterPeriod; /*!< Filter Sample Period. The divider to bus clock. Available range is 0-255. */
} acmp_filter_config_t;

/*! @brief Configuration for DAC */
typedef struct _acmp_dac_config
{
    acmp_reference_voltage_source_t referenceVoltageSource; /*!< Supply voltage reference source. */
    uint32_t DACValue; /*!< Value for DAC Output Voltage. Available range is 0-63. */
} acmp_dac_config_t;

/*! @brief Configuration for round robin mode. */
typedef struct _acmp_round_robin_config
{
    acmp_fixed_port_t fixedPort; /*!< Fixed mux port. */
    uint32_t fixedChannelNumber; /*!< Indicates which channel is fixed in the fixed mux port. */
    uint32_t checkerChannelMask; /*!< Mask of checker channel index. Available range is channel0:0x01 to channel7:0x80
                                    for round-robin checker. */
    uint32_t sampleClockCount;   /*!< Specifies how many round-robin clock cycles(0~3) later the sample takes place. */
    uint32_t delayModulus;       /*!< Comparator and DAC initialization delay modulus. */
} acmp_round_robin_config_t;

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief Initializes the ACMP.
 *
 * The default configuration can be got by calling ACMP_GetDefaultConfig().
 *
 * @param base ACMP peripheral base address.
 * @param config Pointer to ACMP configuration structure.
 */
void ACMP_Init(CMP_Type *base, const acmp_config_t *config);

/*!
 * @brief Deinitializes the ACMP.
 *
 * @param base ACMP peripheral base address.
 */
void ACMP_Deinit(CMP_Type *base);

/*!
 * @brief Gets the default configuration for ACMP.
 *
 * This function initializes the user configuration structure to default value. The default value are:
 *
 * Example:
   @code
   config->enableHighSpeed = false;
   config->enableInvertOutput = false;
   config->useUnfilteredOutput = false;
   config->enablePinOut = false;
   config->enableHysteresisBothDirections = false;
   config->hysteresisMode = kACMP_hysteresisMode0;
   @endcode
 *
 * @param config Pointer to ACMP configuration structure.
 */
void ACMP_GetDefaultConfig(acmp_config_t *config);

/* @} */

/*!
 * @name Basic Operations
 * @{
 */

/*!
 * @brief Enables or disables the ACMP.
 *
 * @param base ACMP peripheral base address.
 * @param enable True to enable the ACMP.
 */
void ACMP_Enable(CMP_Type *base, bool enable);

/*!
 * @brief Sets the channel configuration.
 *
 * Note that the plus/minus mux's setting is only valid when the positive/negative port's input isn't from DAC but
 * from channel mux.
 *
 * Example:
   @code
   acmp_channel_config_t configStruct = {0};
   configStruct.positivePortInput = kACMP_PortInputFromDAC;
   configStruct.negativePortInput = kACMP_PortInputFromMux;
   configStruct.minusMuxInput = 1U;
   ACMP_SetChannelConfig(CMP0, &configStruct);
   @endcode
 *
 * @param base ACMP peripheral base address.
 * @param config Pointer to channel configuration structure.
 */
void ACMP_SetChannelConfig(CMP_Type *base, const acmp_channel_config_t *config);

/* @} */

/*!
 * @name Advanced Operations
 * @{
 */

/*!
 * @brief Enables or disables DMA.
 *
 * @param base ACMP peripheral base address.
 * @param enable True to enable DMA.
 */
void ACMP_EnableDMA(CMP_Type *base, bool enable);

/*!
 * @brief Enables or disables window mode.
 *
 * @param base ACMP peripheral base address.
 * @param enable True to enable window mode.
 */
void ACMP_EnableWindowMode(CMP_Type *base, bool enable);

/*!
 * @brief Configures the filter.
 *
 * The filter can be enabled when the filter count is bigger than 1, the filter period is greater than 0 and the sample
 * clock is from divided bus clock or the filter is bigger than 1 and the sample clock is from external clock. Detailed
 * usage can be got from the reference manual.
 *
 * Example:
   @code
   acmp_filter_config_t configStruct = {0};
   configStruct.filterCount = 5U;
   configStruct.filterPeriod = 200U;
   configStruct.enableSample = false;
   ACMP_SetFilterConfig(CMP0, &configStruct);
   @endcode
 *
 * @param base ACMP peripheral base address.
 * @param config Pointer to filter configuration structure.
 */
void ACMP_SetFilterConfig(CMP_Type *base, const acmp_filter_config_t *config);

/*!
 * @brief Configures the internal DAC.
 *
 * Example:
   @code
   acmp_dac_config_t configStruct = {0};
   configStruct.referenceVoltageSource = kACMP_VrefSourceVin1;
   configStruct.DACValue = 20U;
   ACMP_SetDACConfig(CMP0, &configStruct);
   @endcode
 *
 * @param base ACMP peripheral base address.
 * @param config Pointer to DAC configuration structure. "NULL" is for disabling the feature.
 */
void ACMP_SetDACConfig(CMP_Type *base, const acmp_dac_config_t *config);

/*!
 * @brief Configures the round robin mode.
 *
 * Example:
   @code
   acmp_round_robin_config_t configStruct = {0};
   configStruct.fixedPort = kACMP_FixedPlusPort;
   configStruct.fixedChannelNumber = 3U;
   configStruct.checkerChannelMask = 0xF7U;
   configStruct.sampleClockCount = 0U;
   configStruct.delayModulus = 0U;
   ACMP_SetRoundRobinConfig(CMP0, &configStruct);
   @endcode
 * @param base ACMP peripheral base address.
 * @param config Pointer to round robin mode configuration structure. "NULL" is for disabling the feature.
 */
void ACMP_SetRoundRobinConfig(CMP_Type *base, const acmp_round_robin_config_t *config);

/*!
 * @brief Defines the pre-set state of channels in round robin mode.
 *
 * Note: The pre-state has different circuit with get-round-robin-result in the SOC even though they are same bits.
 * So get-round-robin-result can't return the same value as the value are set by pre-state.
 *
 * @param base ACMP peripheral base address.
 * @param mask Mask of round robin channel index. Available range is channel0:0x01 to channel7:0x80.
 */
void ACMP_SetRoundRobinPreState(CMP_Type *base, uint32_t mask);

/*!
 * @brief Gets the channel input changed flags in round robin mode.
 *
 * @param base ACMP peripheral base address.
 * @return Mask of channel input changed asserted flags. Available range is channel0:0x01 to channel7:0x80.
 */
static inline uint32_t ACMP_GetRoundRobinStatusFlags(CMP_Type *base)
{
    return (((base->C2) & (CMP_C2_CH0F_MASK | CMP_C2_CH1F_MASK | CMP_C2_CH2F_MASK | CMP_C2_CH3F_MASK |
                           CMP_C2_CH4F_MASK | CMP_C2_CH5F_MASK | CMP_C2_CH6F_MASK | CMP_C2_CH7F_MASK)) >>
            CMP_C2_CH0F_SHIFT);
}

/*!
 * @brief Clears the channel input changed flags in round robin mode.
 *
 * @param base ACMP peripheral base address.
 * @param mask Mask of channel index. Available range is channel0:0x01 to channel7:0x80.
 */
void ACMP_ClearRoundRobinStatusFlags(CMP_Type *base, uint32_t mask);

/*!
 * @brief Gets the round robin result.
 *
 * Note that the set-pre-state has different circuit with get-round-robin-result in the SOC even though they are same
 * bits. So ACMP_GetRoundRobinResult() can't return the same value as the value are set by ACMP_SetRoundRobinPreState.

 * @param base ACMP peripheral base address.
 * @return Mask of round robin channel result. Available range is channel0:0x01 to channel7:0x80.
 */
static inline uint32_t ACMP_GetRoundRobinResult(CMP_Type *base)
{
    return ((base->C2 & CMP_C2_ACOn_MASK) >> CMP_C2_ACOn_SHIFT);
}

/* @} */

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enables interrupts.
 *
 * @param base ACMP peripheral base address.
 * @param mask Interrupts mask. See "_acmp_interrupt_enable".
 */
void ACMP_EnableInterrupts(CMP_Type *base, uint32_t mask);

/*!
 * @brief Disables interrupts.
 *
 * @param base ACMP peripheral base address.
 * @param mask Interrupts mask. See "_acmp_interrupt_enable".
 */
void ACMP_DisableInterrupts(CMP_Type *base, uint32_t mask);

/* @} */

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Gets status flags.
 *
 * @param base ACMP peripheral base address.
 * @return Status flags asserted mask. See "_acmp_status_flags".
 */
uint32_t ACMP_GetStatusFlags(CMP_Type *base);

/*!
 * @brief Clears status flags.
 *
 * @param base ACMP peripheral base address.
 * @param mask Status flags mask. See "_acmp_status_flags".
 */
void ACMP_ClearStatusFlags(CMP_Type *base, uint32_t mask);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_ACMP_H_ */
