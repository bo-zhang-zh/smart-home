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

#ifndef _FSL_HSADC_H_
#define _FSL_HSADC_H_

#include "fsl_common.h"

/*!
 * @addtogroup hsadc
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief HSADC driver version. */
#define FSL_HSADC_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*! @brief Converter index to mask for sample slot. */
#define HSADC_SAMPLE_MASK(index) (1U << (index))

/*!
 * @brief HSADC status flags.
 */
enum _hsadc_status_flags
{
    kHSADC_ZeroCrossingFlag = (1U << 0U),               /*!< Zero crossing. */
    kHSADC_HighLimitFlag = (1U << 1U),                  /*!< High limit. */
    kHSADC_LowLimitFlag = (1U << 2U),                   /*!< Low limit. */
    kHSADC_ConverterAEndOfScanFlag = (1U << 3U),        /*!< End of Scan, converter A. */
    kHSADC_ConverterBEndOfScanFlag = (1U << 4U),        /*!< End of Scan, converter B. */
    kHSADC_ConverterAEndOfCalibrationFlag = (1U << 5U), /*!< End of Calibration, converter A. */
    kHSADC_ConverterBEndOfCalibrationFlag = (1U << 6U), /*!< End of Calibration, converter B. */
    kHSADC_ConverterAConvertingFlag = (1U << 7U),       /*!< Conversion in progress, converter A. */
    kHSADC_ConverterBConvertingFlag = (1U << 8U),       /*!< Conversion in progress, converter B. */
    kHSADC_ConverterADummyConvertingFlag = (1U << 9U),  /*!< Dummy conversion in progress, converter A. */
    kHSADC_ConverterBDummyConvertingFlag = (1U << 10U), /*!< Dummy conversion in progress, converter B. */
    kHSADC_ConverterACalibratingFlag = (1U << 11U),     /*!< Calibration in progress, converter A. */
    kHSADC_ConverterBCalibratingFlag = (1U << 12U),     /*!< Calibration in progress, converter B. */
    kHSADC_ConverterAPowerDownFlag = (1U << 13U),       /*!< The converter is powered down, converter A. */
    kHSADC_ConverterBPowerDownFlag = (1U << 14U),       /*!< The converter is powered down, converter B. */
};

/*!
 * @brief HSADC Interrupts.
 */
enum _hsadc_interrupt_enable
{
    kHSADC_ZeroCrossingInterruptEnable = (1U << 0U),               /*!< Zero crossing interrupt. */
    kHSADC_HighLimitInterruptEnable = (1U << 1U),                  /*!< High limit interrupt. */
    kHSADC_LowLimitInterruptEnable = (1U << 2U),                   /*!< Low limit interrupt. */
    kHSADC_ConverterAEndOfScanInterruptEnable = (1U << 3U),        /*!< End of Scan interrupt, converter A. */
    kHSADC_ConverterBEndOfScanInterruptEnable = (1U << 4U),        /*!< End of Scan interrupt, converter B.*/
    kHSADC_ConverterAEndOfCalibrationInterruptEnable = (1U << 5U), /*!< End of Calibration, converter A. */
    kHSADC_ConverterBEndOfCalibrationInterruptEnable = (1U << 6U), /*!< End of Calibration, converter B. */
};

/*!
 * @brief HSADC Converter identifier.
 */
enum _hsadc_converter_id
{
    kHSADC_ConverterA = (1U << 0U), /*!< Converter A. */
    kHSADC_ConverterB = (1U << 1U), /*!< Converter B. */
};

/*!
 * @brief Define the enumeration for dual converter's scan mode.
 */
typedef enum _hsadc_dual_converter_scan_mode
{
    kHSADC_DualConverterWorkAsOnceSequential = 0U,      /*!< Once (single) sequential. */
    kHSADC_DualConverterWorkAsOnceParallel = 1U,        /*!< Once parallel. */
    kHSADC_DualConverterWorkAsLoopSequential = 2U,      /*!< Loop sequential. */
    kHSADC_DualConverterWorkAsLoopParallel = 3U,        /*!< Loop parallel. */
    kHSADC_DualConverterWorkAsTriggeredSequential = 4U, /*!< Triggered sequential. */
    kHSADC_DualConverterWorkAsTriggeredParallel = 5U,   /*!< Triggered parallel. */
} hsadc_dual_converter_scan_mode_t;

typedef enum _hsadc_resolution
{
    kHSADC_Resolution6Bit = 0U,  /*!< 6 bit resolution mode. */
    kHSADC_Resolution8Bit = 1U,  /*!< 8 bit resolution mode. */
    kHSADC_Resolution10Bit = 2U, /*!< 10 bit resolution mode. */
    kHSADC_Resolution12Bit = 3U, /*!< 12 bit resolution mode. */
} hsadc_resolution_t;

/*!
 * @brief Define the enumeration for DMA trigger's source.
 */
typedef enum _hsadc_dma_trigger_source
{
    kHSADC_DMATriggerSourceAsEndOfScan = 0U,   /*!< DMA trigger source is end of scan interrupt. */
    kHSADC_DMATriggerSourceAsSampleReady = 1U, /*!< DMA trigger source is RDY bits. */
} hsadc_dma_trigger_source_t;

/*!
 * @brief Define the enumeration for the sample slot's zero crossing event.
 */
typedef enum _hsadc_zero_crossing_mode
{
    kHSADC_ZeroCorssingDisabled = 0U,          /*!< Zero Crossing disabled. */
    kHSADC_ZeroCorssingForPtoNSign = 1U,       /*!< Zero Crossing enabled for positive to negative sign change. */
    kHSADC_ZeroCorssingForNtoPSign = 2U,       /*!< Zero Crossing enabled for negative to positive sign change. */
    kHSADC_ZeroCorssingForAnySignChanged = 3U, /*!< Zero Crossing enabled for any sign change. */
} hsadc_zero_crossing_mode_t;

/*!
 * @brief Define the enumeration for converter's work mode in idle mode.
 */
typedef enum _hsadc_idle_work_mode
{
    kHSADC_IdleKeepNormal = 0U,    /*!< Keep normal. */
    kHSADC_IdleAutoStandby = 1U,   /*!< Fall into standby mode automatically. */
    kHSADC_IdleAutoPowerDown = 2U, /*!< Fall into power down mode automatically. */
} hsadc_idle_work_mode_t;

/*!
 * @brief Converter's calibration mode.
 */
enum _hsadc_calibration_mode
{
    kHSADC_CalibrationModeDifferential = (1U << 0U), /*!< Calibration request for differential mode. */
    kHSADC_CalibrationModeSingleEnded = (1U << 1U),  /*!< Calibration request for single ended mode. */
};

/*! @brief Bit mask of calibration value for converter A in single ended mode. */
#define HSADC_CALIBRATION_VALUE_A_SINGLE_ENDED_MASK HSADC_CALVAL_A_CALVSING_MASK
/*! @brief Bit shift of calibration value for converter A in single ended mode. */
#define HSADC_CALIBRATION_VALUE_A_SINGLE_ENDED_SHIFT HSADC_CALVAL_A_CALVSING_SHIFT
/*! @brief Bit mask of calibration value for converter A in differential mode. */
#define HSADC_CALIBRATION_VALUE_A_DIFFERENTIAL_MASK HSADC_CALVAL_A_CALVDIF_MASK
/*! @brief Bit shift of calibration value for converter A in differential mode. */
#define HSADC_CALIBRATION_VALUE_A_DIFFERENTIAL_SHIFT HSADC_CALVAL_A_CALVDIF_SHIFT
/*! @brief Bit mask of calibration value for converter B in single ended mode. */
#define HSADC_CALIBRATION_VALUE_B_SINGLE_ENDED_MASK (HSADC_CALVAL_B_CALVSING_MASK << 16U)
/*! @brief Bit shift of calibration value for converter B in single ended mode. */
#define HSADC_CALIBRATION_VALUE_B_SINGLE_ENDED_SHIFT (HSADC_CALVAL_B_CALVSING_SHIFT + 16U)
/*! @brief Bit mask of calibration value for converter B in differential mode. */
#define HSADC_CALIBRATION_VALUE_B_DIFFERENTIAL_MASK (HSADC_CALVAL_B_CALVDIF_MASK << 16U)
/*! @brief Bit shift of calibration value for converter B in differential mode. */
#define HSADC_CALIBRATION_VALUE_B_DIFFERENTIAL_SHIFT (HSADC_CALVAL_B_CALVDIF_SHIFT + 16U)

/*!
 * @brief Define the structure for configuring the HSADC's common setting.
 */
typedef struct _hsadc_config
{
    hsadc_dual_converter_scan_mode_t dualConverterScanMode; /*!< Dual converter's scan mode. */
    bool enableSimultaneousMode;                            /*!< Using Simultaneous mode. */
    hsadc_resolution_t resolution;                          /*!< Resolution mode. */
    hsadc_dma_trigger_source_t DMATriggerSoruce;            /*!< DMA trigger source. */
    hsadc_idle_work_mode_t idleWorkMode;                    /*!< Converter's work mode when idle. */
    uint16_t powerUpDelayCount; /*!< Delay count united as 32 clocks to wait for the clock to be stable. Available
                                range is 0-63. */
} hsadc_config_t;

/*!
 * @brief Define the structure for configuring each converter.
 */
typedef struct _hsadc_converter_config
{
    uint16_t clockDivisor;      /*!< Converter's clock divisor for the clock source. Available range is 2-64. */
    uint16_t samplingTimeCount; /*!< Sampling time count. The resultant sampling time is (1.5 + samplingTimeCount) x
                                clock period. Available range is 0-255. */
    uint16_t
        powerUpCalibrationModeMask; /*!< Calibration mode mask in the power up period. See to "_hsadc_calibration_mode".
                                If this field isn't zero, user should call the function HSADC_GetStatusFlags() to
                                check whether the End of Calibration flag be set to wait the calibration process
                                complete. If this is zero, it indicates no calibration will be executed in power
                                up period. */
} hsadc_converter_config_t;

/*!
 * @brief Define the structure for configuring the sample slot.
 *
 * channelNumber, channel67MuxNumber and enableDifferentialPair have following relationship:
 * channelNumber equals 0~7 represents channel 0~7 of converter A. channelNumber equals 8~15 represents channel 0~7 of
 * converter B.
 * 1) When channelNumber = 6 and enableDifferentialPair = false, channel67MuxNumber represents converter A's channel
 *    6's sub multiplex channel number.
 * 2) When channelNumber = 6 and enableDifferentialPair = true, channel67MuxNumber represents converter A's channel
 *    6 and channel 7's sub multiplex channel number.
 * 3) When channelNumber = 7 and enableDifferentialPair = false, channel67MuxNumber represents converter A's channel
 *    7's sub multiplex channel number.
 * 4) When channelNumber = 7 and enableDifferentialPair = true, channel67MuxNumber represents converter A's channel
 *    6 and channel 7's sub multiplex channel number.
 * 5) When channelNumber = 14 and enableDifferentialPair = false, channel67MuxNumber represents converter B's channel
 *    6's sub multiplex channel number.
 * 6) When channelNumber = 14 and enableDifferentialPair = true, channel67MuxNumber represents converter B's channel
 *    6 and channel 7's sub multiplex channel number.
 * 7) When channelNumber = 15 and enableDifferentialPair = false, channel67MuxNumber represents converter B's channel
 *    7's sub multiplex channel number.
 * 8) When channelNumber = 15 and enableDifferentialPair = true, channel67MuxNumber represents converter B's channel
 *    6 and channel 7's sub multiplex channel number.
 * 9) In other cases, channel67MuxNumber won't be functional.
 */
typedef struct _hsadc_sample_config
{
    /* Conversion channel setting. */
    uint16_t channelNumber;      /*!< Channel number. Available range is 0-15.  */
    uint16_t channel67MuxNumber; /*!< Channel 6/7's sub multiplex channel number. When channelNumber = 6 or 14, its
                                 available range is 0~6. When channelNumber = 7 or 15, its available range is 0~5. */
    bool enableDifferentialPair; /*!< Use differential sample input or not. In differential mode, the sub multiplex
                                 channel number of channel 6 and channel 7 must be configured to be same. */

    /* sample result setting. */
    hsadc_zero_crossing_mode_t zeroCrossingMode; /*!< Zero crossing mode. */
    uint16_t highLimitValue; /*!< High limit value. Original value format as hardware register, with 3-bits left
                             shifted. */
    uint16_t lowLimitValue;  /*!< Low limit value. Original value format as hardware register, with 3-bits left
                             shifted. */
    uint16_t offsetValue;    /*!< Offset value. Original value format as hardware register, with 3-bits left shifted. */
    bool enableWaitSync;     /*!< Wait for sync input to launch this sample's conversion or not. */
} hsadc_sample_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name HSADC Initialization and deinitialization.
 * @{
 */

/*!
 * @brief Initialization for the HSADC module.
 *
 * This function is to make the initialization for using HSADC module.
 * The operations are:
 *  - Enable the clock for HSADC.
 *  - Set the global settings for HSADC converter.
 *
 * @param base   HSADC peripheral base address.
 * @param config Pointer to configuration structure. See to "hsadc_config_t".
 */
void HSADC_Init(HSADC_Type *base, const hsadc_config_t *config);

/*!
 * @brief Get an available pre-defined settings for module's configuration.
 *
 * This function initializes the module's configuration structure with an available settings.
 * The default value are:
 * @code
 *   config->dualConverterScanMode = kHSADC_DualConverterWorkAsTriggeredParallel;
 *   config->enableSimultaneousMode = true;
 *   config->resolution = kHSADC_Resolution12Bit;
 *   config->DMATriggerSoruce = kHSADC_DMATriggerSourceAsEndOfScan;
 *   config->idleWorkMode = kHSADC_IdleKeepNormal;
 *   config->powerUpDelay = 18U;
 * @endcode
 * @param config Pointer to configuration structure. See to "hsadc_config_t"
 */
void HSADC_GetDefaultConfig(hsadc_config_t *config);

/*!
 * @brief De-Initialization for the HSADC module.
 *
 * This function is to make the de-initialization for using HSADC module.
 * The operations are:
 *  - Power down both converter.
 *  - Disable the clock for HSADC.
 *
 * @param base HSADC peripheral base address.
 */
void HSADC_Deinit(HSADC_Type *base);

/* @} */

/*!
 * @name Converter.
 * @{
 */

/*!
 * @brief Configure the converter.
 *
 * @param base          HSADC peripheral base address.
 * @param converterMask Mask for converters to be configured. See to "_hsadc_converter_id".
 * @param config        Pointer to configuration structure. See to "hsadc_converter_config_t".
 */
void HSADC_SetConverterConfig(HSADC_Type *base, uint16_t converterMask, const hsadc_converter_config_t *config);

/*!
 * @brief Get an available pre-defined settings for each converter's configuration.
 *
 * This function initializes each converter's configuration structure with an available settings.
 * The default value are:
 * @code
 *   config->clockDivisor = 4U;
 *   config->samplingTimeCount = 0U;
 *   config->enablePowerUpCalibration = false;
 *   config->powerUpCalibrationModeMask = kHSADC_CalibrationModeSingleEnded;
 * @endcode
 * @param config Pointer to configuration structure. See to "hsadc_converter_config_t"
 */
void HSADC_GetDefaultConverterConfig(hsadc_converter_config_t *config);

/*!
 * @brief Enable the converter's conversion.
 *
 * This function is to enable the converter's conversion by making the converter exit stop mode. The conversion should
 * only be launched after the converter is enabled. When this feature is asserted to be "false", the current scan is
 * stopped and no further scans can start. All the software trigger and hardware trigger are ignored.
 *
 * @param base          HSADC peripheral base address.
 * @param converterMask Mask for converters to be operated. See to "_hsadc_converter_id".
 * @param enable         Enable the feature or not.
 */
void HSADC_EnableConverter(HSADC_Type *base, uint16_t converterMask, bool enable);

/*!
 * @brief Enable the input of external sync signal.
 *
 * This function is to enable the input of external sync signal. The external sync signal could be used to trigger the
 * conversion if the hardware trigger related setting is used.
 * Note: When in "Once" scan mode, this gate would be off automatically after an available sync is received. User needs
 * to enable the input again manually if another sync signal is wanted.
 *
 * @param base          HSADC peripheral base address.
 * @param converterMask Mask for converters to be operated. See to "_hsadc_converter_id".
 * @param enable        Enable the feature or not.
 */
void HSADC_EnableConverterSyncInput(HSADC_Type *base, uint16_t converterMask, bool enable);

/*!
 * @brief Enable power for the converter.
 *
 * This function is to enable the power for the converter. The converter should be powered on before conversion. Once
 * this API is called, the converter would be powered on after a few moment (so-called power up delay) to make the
 * power to be stable.
 *
 * @param base          HSADC peripheral base address.
 * @param converterMask Mask for converters to be operated. See to "_hsadc_converter_id".
 * @param enable        Enable the feature or not.
 */
void HSADC_EnableConverterPower(HSADC_Type *base, uint16_t converterMask, bool enable);

/*!
 * @brief Trigger the converter by software trigger.
 *
 * This function is to do the software trigger to the converter. The software trigger can be used to start a conversion
 * sequence.
 *
 * @param base          HSADC peripheral base address.
 * @param converterMask Mask for converters to be operated. See to "_hsadc_converter_id".
 */
void HSADC_DoSoftwareTriggerConverter(HSADC_Type *base, uint16_t converterMask);

/*!
 * @brief Enable the DMA feature
 *
 * @param base          HSADC peripheral base address.
 * @param converterMask Mask for converters to be operated. See to "_hsadc_converter_id".
 * @param enable        Enable the feature or not.
 */
void HSADC_EnableConverterDMA(HSADC_Type *base, uint16_t converterMask, bool enable);

/*!
 * @brief Enable the interrupts.
 *
 * @param base HSADC peripheral base address.
 * @param mask Mask value for interrupt events. See to "_hsadc_interrupt_enable".
 */
void HSADC_EnableInterrupts(HSADC_Type *base, uint16_t mask);

/*!
 * @brief Disable the interrupts.
 *
 * @param base HSADC peripheral base address.
 * @param mask Mask value for interrupt events. See to "_hsadc_interrupt_enable".
 */
void HSADC_DisableInterrupts(HSADC_Type *base, uint16_t mask);

/*!
 * @brief  Get the status flags.
 *
 * @param  base HSADC peripheral base address.
 *
 * @return      Mask value for the event flags. See to "_hsadc_status_flags".
 */
uint16_t HSADC_GetStatusFlags(HSADC_Type *base);

/*!
 * @brief Clear the status flags.
 *
 * @param base  HSADC peripheral base address.
 * @param flags Mask value for the event flags to be cleared. See to "_hsadc_status_flags".
 */
void HSADC_ClearStatusFlags(HSADC_Type *base, uint16_t mask);

/* @} */

/*!
 * @name Sample.
 * @{
 */

/*!
 * @brief Configure the sample slot.
 *
 * Sample list in this module works like a conversion sequence. Each sample slot can be used to designate to sample
 * which channel in converter A and converter B. The detail mapping relationship between sample slot and converter's
 * channel can be got from the reference manual.
 *
 * @param base        HSADC peripheral base address.
 * @param sampleIndex Index of sample slot in conversion sequence. Available range is 0-15.
 * @param config      Pointer to configure structure. See to "hsadc_sample_config_t".
 */
void HSADC_SetSampleConfig(HSADC_Type *base, uint16_t sampleIndex, const hsadc_sample_config_t *config);

/*!
 * @brief Get the default sample configuration.
 *
 * This function initializes each sample's configuration structure with an available settings.
 * The default value are:
 * @code
 *   config->channelNumber = 0U;
 *   config->channel6MuxNumber = 0U;
 *   config->channel7MuxNumber = 0U;
 *   config->enableDifferentialPair = false;
 *   config->zeroCrossingMode = kHSADC_ZeroCorssingDisabled;
 *   config->highLimitValue = 0x7FF8U;
 *   config->lowLimitValue = 0U;
 *   config->offsetValue = 0U;
 *   config->enableWaitSync = false;
 * @endcode
 * @param config Pointer to configure structure. See to "hsadc_sample_config_t".
 */
void HSADC_GetDefaultSampleConfig(hsadc_sample_config_t *config);

/*!
 * @brief Enable the sample slot.
 *
 * This function is to enable the sample slot. Only the enabled sample slot can join the conversion sequence.
 *
 * @param base       HSADC peripheral base address.
 * @param sampleMask Mask value of sample slots in conversion sequence. Each bit is corresponding to a sample slot.
 * @param enable     Enable the feature or not.
 */
static inline void HSADC_EnableSample(HSADC_Type *base, uint16_t sampleMask, bool enable)
{
    if (enable)
    {
        base->SDIS &= ~HSADC_SDIS_DS(sampleMask);
    }
    else
    {
        base->SDIS |= HSADC_SDIS_DS(sampleMask);
    }
}

/*!
 * @brief Enable the interrupt for each sample slot when its result is ready.
 *
 * @param base       HSADC peripheral base address.
 * @param sampleMask Mask value of sample slots in conversion sequence. Each bit is corresponding to a sample slot.
 * @param enable     Enable the feature or not.
 */
static inline void HSADC_EnableSampleResultReadyInterrupts(HSADC_Type *base, uint16_t sampleMask, bool enable)
{
    if (enable)
    {
        base->SCINTEN |= HSADC_SCINTEN_SCINTEN(sampleMask);
    }
    else
    {
        base->SCINTEN &= ~HSADC_SCINTEN_SCINTEN(sampleMask);
    }
}

/*!
 * @brief  Return the sample ready flags of sample slots.
 *
 * @param  base HSADC peripheral base address.
 *
 * @return      Mask value for the sample slots if their result are ready.
 */
static inline uint16_t HSADC_GetSampleReadyStatusFlags(HSADC_Type *base)
{
    return (HSADC_RDY_RDY_MASK & base->RDY);
}

/*!
 * @brief  Get the low limit flags of sample slots.
 *
 * @param  base HSADC peripheral base address.
 *
 * @return      Mask value for the sample slots if their results exceed the low limit.
 */
static inline uint16_t HSADC_GetSampleLowLimitStatusFlags(HSADC_Type *base)
{
    return (HSADC_LOLIMSTAT_LLS_MASK & base->LOLIMSTAT);
}

/*!
 * @brief Clear low limit flags of sample slots.
 *
 * @param base       HSADC peripheral base address.
 * @param sampleMask Mask value for the sample slots' flags to be cleared.
 */
static inline void HSADC_ClearSampleLowLimitStatusFlags(HSADC_Type *base, uint16_t sampleMask)
{
    base->LOLIMSTAT = HSADC_LOLIMSTAT_LLS(sampleMask);
}

/*!
 * @brief  Get the high limit flags of sample slots.
 *
 * @param  base HSADC peripheral base address.
 *
 * @return      Mask value for the sample slots if their results exceed the high limit.
 */
static inline uint16_t HSADC_GetSampleHighLimitStatusFlags(HSADC_Type *base)
{
    return (HSADC_HILIMSTAT_HLS_MASK & base->HILIMSTAT);
}

/*!
 * @brief Clear high limit flags of sample slots.
 *
 * @param base       HSADC peripheral base address.
 * @param sampleMask Mask value for the sample slots to be cleared flags.
 */
static inline void HSADC_ClearSampleHighLimitStatusFlags(HSADC_Type *base, uint16_t sampleMask)
{
    base->HILIMSTAT = HSADC_HILIMSTAT_HLS(sampleMask);
}
/*!
 * @brief  Get the zero crossing flags of sample slots.
 *
 * @param  base HSADC peripheral base address.
 *
 * @return      Mask value for the sample slots if their results cause the zero crossing event.
 */
static inline uint16_t HSADC_GetSampleZeroCrossingStatusFlags(HSADC_Type *base)
{
    return (HSADC_ZXSTAT_ZCS_MASK & base->ZXSTAT);
}

/*!
 * @brief Clear zero crossing flags of sample slots.
 *
 * @param base       HSADC peripheral base address.
 * @param sampleMask Mask value for the sample slots to be cleared flags.
 */
static inline void HSADC_ClearSampleZeroCrossingStatusFlags(HSADC_Type *base, uint16_t sampleMask)
{
    base->ZXSTAT = HSADC_ZXSTAT_ZCS(sampleMask);
}

/*!
 * @brief  Get the sample result value.
 *
 * This function is to get the sample result value. This returned value keeps it original formation just like in
 * hardware result register. It includes the sign bit as the MSB and 3-bit left shifted value.
 *
 * @param  base        HSADC peripheral base address.
 * @param  sampleIndex Index of sample slot.
 *
 * @return             Sample's conversion value.
 */
static inline uint16_t HSADC_GetSampleResultValue(HSADC_Type *base, uint16_t sampleIndex)
{
    return base->RSLT[sampleIndex];
}

/* @} */

/*!
 * @name Calibration.
 * @{
 */

/*!
 * @brief Start the hardware calibration.
 *
 * This function can start the single ended calibration and differential calibration for converter A and converter B
 * at the same time.
 * Note that this is a non blocking function. End of Scan flag and End of Calibration flag will both be set after
 * calibration process. So user should check these two flags by using the function HSADC_GetStatusFlags() to wait the
 * calibration process complete.
 *
 * @param base                HSADC peripheral base address.
 * @param converterMask       Mask for converters to be operated. See to "_hsadc_converter_id".
 * @param calibrationModeMask Mask for calibration mode to be operated. See to "_hsadc_calibration_mode". Shouldn't be
 * zero.
 */
void HSADC_DoAutoCalibration(HSADC_Type *base, uint16_t converterMask, uint16_t calibrationModeMask);

/*!
 * @brief Get the calibration result value.
 *
 * This function returns the single ended calibration value and differential calibration value for converter A and
 * converter B. The calibration value of each calibration mode for each converter can be got from this function's
 * return value by using the mask and shift definition from HSADC_CALIBRATION_VALUE_A_SINGLE_ENDED_MASK to
 * HSADC_CALIBRATION_VALUE_B_DIFFERENTIAL_SHIFT.
 *
 * @param base                HSADC peripheral base address.
 * @return                    Calibration value for converter A and converter B.
 */
uint32_t HSADC_GetCalibrationResultValue(HSADC_Type *base);

/*!
 * @brief Enable or disable the calibration result value.
 *
 * This function enable or disable converter A and converter B to use the calibration values to obtain the final
 * conversion result by calibration sum operation.
 *
 * @param base          HSADC peripheral base address.
 * @param converterMask Mask for converters to be operated. See to "_hsadc_converter_id".
 * @param enable        Enable the feature or not.
 */
void HSADC_EnableCalibrationResultValue(HSADC_Type *base, uint16_t converterMask, bool enable);

/* @} */
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_HSADC_H_ */
