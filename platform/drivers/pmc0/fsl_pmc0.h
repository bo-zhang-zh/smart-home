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
#ifndef _FSL_PMC0_H_
#define _FSL_PMC0_H_

#include "fsl_common.h"

/*! @addtogroup pmc0 */
/*! @{ */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief PMC 0 driver version */
#define FSL_PMC0_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*!
 * @brief MAX valid values of Core Regulator Voltage Level
 */
#define CORE_REGULATOR_VOLT_LEVEL_MAX 50U

/*!
 * @brief High Voltage Detect Monitor Select
 */
typedef enum _pmc0_high_volt_detect_monitor_select
{
    kPMC0_HighVoltDetectLowPowerMonitor = 0U, /*!< LP monitor is selected. */
    kPMC0_HighVoltDetectHighPowerMonitor = 1U /*!< HP monitor is selected. */
} pmc0_high_volt_detect_monitor_select_t;

/*!
 * @brief Low Voltage Detect Monitor Select
 */
typedef enum _pmc0_low_volt_detect_monitor_select
{
    kPMC0_LowVoltDetectLowPowerMonitor = 0U, /*!< LP monitor is selected. */
    kPMC0_LowVoltDetectHighPowerMonitor = 1U /*!< HP monitor is selected. */
} pmc0_low_volt_detect_monitor_select_t;

/*!
 * @brief Core Regulator Select
 */
typedef enum _pmc0_core_regulator_select
{
    kPMC0_CoreLowPowerRegulator = 0U, /*!< Core LP regulator is selected. */
    kPMC0_CoreHighPowerRegulator = 1U /*!< Core HP regulator is selected. */
} pmc0_core_regulator_select_t;

/*!
 * @brief Array Regulator Select
 */
typedef enum _pmc0_array_regulator_select
{
    kPMC0_ArrayLowPowerRegulator = 0U, /*!< Array LP regulator is selected. */
    kPMC0_ArrayHighPowerRegulator = 1U /*!< Array HP regulator is selected. */
} pmc0_array_regulator_select_t;

/*!
 * @brief VLLS mode array Regulator Select
 */
typedef enum _pmc0_vlls_array_regulator_select
{
    kPMC0_VllsArrayRegulatorOff = 0U, /*!< Array regulator is selected OFF. This is selectable only for VLLS mode. */
    kPMC0_VllsArrayLowPowerRegulator = 2U, /*!< Array LP regulator is selected. */
    kPMC0_VllsArrayHighPowerRegulator = 3U /*!< Array HP regulator is selected. */
} pmc0_vlls_array_regulator_select_t;

/*!
 * @brief PMC 0 status flags
 */
enum _pmc0_status_flags
{
    kPMC0_LowVoltDetectEventFlag =
        PMC0_STATUS_LVD1P2F_MASK, /*!< 1.2V Low-Voltage Detector Flag, sets when low-voltage event was detected. */
    kPMC0_LowVoltDetectValueFlag = PMC0_STATUS_LVD1P2V_MASK, /*!< 1.2V Low-Voltage Detector Value, sets when current
                                                                value of the 1.2V LVD monitor output is 1. */
    kPMC0_HighVoltDetectEventFlag =
        PMC0_STATUS_HVD1P2F_MASK, /*!< 1.2V High-Voltage Detector Flag, sets when high-voltage event was detected. */
    kPMC0_HighVoltDetectValueFlag = PMC0_STATUS_HVD1P2V_MASK, /*!< 1.2V High-Voltage Detector Value, sets when current
                                                                 value of the 1.2V HVD monitor output is 1. */
    kPMC0_CoreRegulatorVoltLevelFlag =
        PMC0_STATUS_COREVLF_MASK, /*!< Core Regulator Voltage Level Flag, sets when core regulator voltage level is
                                     changing (not stable). */
    kPMC0_SramFlag =
        PMC0_STATUS_SRAMF_MASK /*!< SRAM Flag, sets when a change mode request is being processed in the SRAMs. */
};

/*!
 * @brief PMC 0 HSRUN mode configuration.
 */
typedef struct _pmc0_hsrun_mode_config
{
    uint32_t reserved1 : 16;             /*!< Reserved. */
    uint32_t coreRegulatorVoltLevel : 6; /*!< Core Regulator Voltage Level. */
    uint32_t reserved2 : 2;              /*!< Reserved. */
    uint32_t enableForwardBias : 1;      /*!< Enable forward bias. */
    uint32_t reserved3 : 7;              /*!< Reserved. */
} pmc0_hsrun_mode_config_t;

/*!
 * @brief PMC 0 RUN mode configuration.
 */
typedef struct _pmc0_run_mode_config
{
    uint32_t reserved1 : 16;             /*!< Reserved. */
    uint32_t coreRegulatorVoltLevel : 6; /*!< Core Regulator Voltage Level. */
    uint32_t reserved2 : 10;             /*!< Reserved. */
} pmc0_run_mode_config_t;

/*!
 * @brief PMC 0 VLPR mode configuration.
 */
typedef struct _pmc0_vlpr_mode_config
{
    uint32_t arrayRegulatorSelect : 1;   /*!< Array Regulator Select. @ref pmc0_array_regulator_select_t*/
    uint32_t reserved1 : 1;              /*!< Reserved. */
    uint32_t coreRegulatorSelect : 1;    /*!< Core Regulator Select. @ref pmc0_core_regulator_select_t*/
    uint32_t reserved2 : 1;              /*!< Reserved. */
    uint32_t lvdMonitorSelect : 1;       /*!< 1.2V LVD Monitor Select. @ref pmc0_low_volt_detect_monitor_select_t */
    uint32_t hvdMonitorSelect : 1;       /*!< 1.2V HVD Monitor Select. @ref pmc0_high_volt_detect_monitor_select_t */
    uint32_t reserved3 : 1;              /*!< Reserved. */
    uint32_t enableForceHpBandgap : 1;   /*!< Enable force HP band-gap. */
    uint32_t reserved4 : 8;              /*!< Reserved. */
    uint32_t coreRegulatorVoltLevel : 6; /*!< Core Regulator Voltage Level. */
    uint32_t reserved5 : 6;              /*!< Reserved. */
    uint32_t enableReverseBackBias : 1;  /*!< Enable reverse back bias. */
    uint32_t reserved6 : 3;              /*!< Reserved. */
} pmc0_vlpr_mode_config_t;

/*!
 * @brief PMC 0 STOP mode configuration.
 */
typedef struct _pmc0_stop_mode_config
{
    uint32_t reserved1 : 16;         /*!< Reserved. */
    uint32_t coreRegulatorVoltLevel; /*!< Core Regulator Voltage Level. */
    uint32_t reserved2 : 10;         /*!< Reserved. */
} pmc0_stop_mode_config_t;

/*!
 * @brief PMC 0 VLPS mode configuration.
 */
typedef struct _pmc0_vlps_mode_config
{
    uint32_t arrayRegulatorSelect : 1;   /*!< Array Regulator Select. @ref pmc0_array_regulator_select_t*/
    uint32_t reserved1 : 1;              /*!< Reserved. */
    uint32_t coreRegulatorSelect : 1;    /*!< Core Regulator Select. @ref pmc0_core_regulator_select_t*/
    uint32_t reserved2 : 1;              /*!< Reserved. */
    uint32_t lvdMonitorSelect : 1;       /*!< 1.2V LVD Monitor Select. @ref pmc0_low_volt_detect_monitor_select_t */
    uint32_t hvdMonitorSelect : 1;       /*!< 1.2V HVD Monitor Select. @ref pmc0_high_volt_detect_monitor_select_t */
    uint32_t reserved3 : 1;              /*!< Reserved. */
    uint32_t enableForceHpBandgap : 1;   /*!< Enable force HP band-gap. */
    uint32_t reserved4 : 8;              /*!< Reserved. */
    uint32_t coreRegulatorVoltLevel : 6; /*!< Core Regulator Voltage Level. */
    uint32_t reserved5 : 6;              /*!< Reserved. */
    uint32_t enableReverseBackBias : 1;  /*!< Enable reverse back bias. */
    uint32_t reserved6 : 3;              /*!< Reserved. */
} pmc0_vlps_mode_config_t;

/*!
 * @brief PMC 0 LLS mode configuration.
 */
typedef struct _pmc0_lls_mode_config
{
    uint32_t arrayRegulatorSelect : 1;   /*!< Array Regulator Select. @ref pmc0_array_regulator_select_t*/
    uint32_t reserved1 : 1;              /*!< Reserved. */
    uint32_t coreRegulatorSelect : 1;    /*!< Core Regulator Select. @ref pmc0_core_regulator_select_t*/
    uint32_t reserved2 : 1;              /*!< Reserved. */
    uint32_t lvdMonitorSelect : 1;       /*!< 1.2V LVD Monitor Select. @ref pmc0_low_volt_detect_monitor_select_t */
    uint32_t hvdMonitorSelect : 1;       /*!< 1.2V HVD Monitor Select. @ref pmc0_high_volt_detect_monitor_select_t */
    uint32_t reserved3 : 1;              /*!< Reserved. */
    uint32_t enableForceHpBandgap : 1;   /*!< Enable force HP band-gap. */
    uint32_t reserved4 : 8;              /*!< Reserved. */
    uint32_t coreRegulatorVoltLevel : 6; /*!< Core Regulator Voltage Level. */
    uint32_t reserved5 : 6;              /*!< Reserved. */
    uint32_t enableReverseBackBias : 1;  /*!< Enable reverse back bias. */
    uint32_t reserved6 : 3;              /*!< Reserved. */
} pmc0_lls_mode_config_t;

/*!
 * @brief PMC 0 VLLS mode configuration.
 */
typedef struct _pmc0_vlls_mode_config
{
    uint32_t arrayRegulatorSelect : 2; /*!< Array Regulator Select. @ref pmc0_vlls_array_regulator_select_t */
    uint32_t reserved1 : 2;            /*!< Reserved. */
    uint32_t lvdMonitorSelect : 1;     /*!< 1.2V LVD Monitor Select. @ref pmc0_low_volt_detect_monitor_select_t */
    uint32_t hvdMonitorSelect : 1;     /*!< 1.2V HVD Monitor Select. @ref pmc0_high_volt_detect_monitor_select_t */
    uint32_t reserved2 : 1;            /*!< Reserved. */
    uint32_t enableForceHpBandgap : 1; /*!< Enable force HP band-gap. */
    uint32_t reserved3 : 24;           /*!< Reserved. */
} pmc0_vlls_mode_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name Power Management Controller Control APIs*/
/*@{*/

/*!
 * @brief Configure the HSRUN power mode.
 *
 * This function configures the HSRUN power mode, including the core regulator
 * voltage Level setting, enable forward bias or not.
 *
 * @param config  Low-Voltage detect configuration structure.
 */
static inline void PMC0_ConfigureHsrunMode(const pmc0_hsrun_mode_config_t *config)
{
    /*
     * The valid values of Core Regulator Voltage Level are between 0 and 50.
     * These values correspond to a valid range of 0.596V to 1.138V with resolution of 10.83mV.
     * The reset value correspond to 0.9V.
     */
    assert(config);
    assert(config->coreRegulatorVoltLevel <= CORE_REGULATOR_VOLT_LEVEL_MAX);

    PMC0->HSRUN = (*((const uint32_t *)config)) & (PMC0_HSRUN_COREREGVL_MASK | PMC0_HSRUN_FBBEN_MASK);
}

/*!
 * @brief Configure the RUN power mode.
 *
 * This function configures the RUN power mode, including the core regulator
 * voltage Level setting.
 *
 * @param config  Low-Voltage detect configuration structure.
 */
static inline void PMC0_ConfigureRunMode(const pmc0_run_mode_config_t *config)
{
    /*
     * The valid values of Core Regulator Voltage Level are between 0 and 50.
     * These values correspond to a valid range of 0.596V to 1.138V with resolution of 10.83mV.
     * The reset value correspond to 0.9V.
     */
    assert(config);
    assert(config->coreRegulatorVoltLevel <= CORE_REGULATOR_VOLT_LEVEL_MAX);

    PMC0->RUN = (*((const uint32_t *)config)) & PMC0_RUN_COREREGVL_MASK;
}

/*!
 * @brief Configure the VLPR power mode.
 *
 * This function configures the VLPR power mode, including the core regulator
 * voltage Level setting, enable reverse back bias or not, turn on force HP
 * band-gap or not, select of HVD/LVD monitor and select of core/array regulator.
 *
 * @param config  Low-Voltage detect configuration structure.
 */
static inline void PMC0_ConfigureVlprMode(const pmc0_vlpr_mode_config_t *config)
{
    /*
     * The valid values of Core Regulator Voltage Level are between 0 and 50.
     * These values correspond to a valid range of 0.596V to 1.138V with resolution of 10.83mV.
     * The reset value correspond to 0.704V.
     */
    assert(config);
    assert(config->coreRegulatorVoltLevel <= CORE_REGULATOR_VOLT_LEVEL_MAX);

    PMC0->VLPR = (*((const uint32_t *)config)) &
                 (PMC0_VLPR_ARRAYREG_MASK | PMC0_VLPR_COREREG_MASK | PMC0_VLPR_MON1P2LVDHP_MASK |
                  PMC0_VLPR_MON1P2HVDHP_MASK | PMC0_VLPR_FBGHP_MASK | PMC0_VLPR_COREREGVL_MASK | PMC0_VLPR_RBBEN_MASK);
}

/*!
 * @brief Configure the STOP power mode.
 *
 * This function configures the STOP power mode, including the core regulator
 * voltage Level setting.
 *
 * @param config  Low-Voltage detect configuration structure.
 */
static void PMC0_ConfigureStopMode(const pmc0_stop_mode_config_t *config)
{
    /*
     * The valid values of Core Regulator Voltage Level are between 0 and 50.
     * These values correspond to a valid range of 0.596V to 1.138V with resolution of 10.83mV.
     * The reset value correspond to 0.9V.
     */
    assert(config);
    assert(config->coreRegulatorVoltLevel <= CORE_REGULATOR_VOLT_LEVEL_MAX);

    PMC0->STOP = (*((const uint32_t *)config)) & PMC0_STOP_COREREGVL_MASK;
}

/*!
 * @brief Configure the VLPS power mode.
 *
 * This function configures the VLPS power mode, including the core regulator
 * voltage Level setting, enable reverse back bias or not, turn on force HP
 * band-gap or not, select of HVD/LVD monitor and select of core/array regulator.
 *
 * @param config  Low-Voltage detect configuration structure.
 */
static inline void PMC0_ConfigureVlpsMode(const pmc0_vlps_mode_config_t *config)
{
    /*
     * The valid values of Core Regulator Voltage Level are between 0 and 50.
     * These values correspond to a valid range of 0.596V to 1.138V with resolution of 10.83mV.
     * The reset value correspond to 0.704V.
     */
    assert(config);
    assert(config->coreRegulatorVoltLevel <= CORE_REGULATOR_VOLT_LEVEL_MAX);

    PMC0->VLPS = (*((const uint32_t *)config)) &
                 (PMC0_VLPS_ARRAYREG_MASK | PMC0_VLPS_COREREG_MASK | PMC0_VLPS_MON1P2LVDHP_MASK |
                  PMC0_VLPS_MON1P2HVDHP_MASK | PMC0_VLPS_FBGHP_MASK | PMC0_VLPS_COREREGVL_MASK | PMC0_VLPS_RBBEN_MASK);
}

/*!
 * @brief Configure the LLS power mode.
 *
 * This function configures the LLS power mode, including the core regulator
 * voltage Level setting, enable reverse back bias or not, turn on force HP
 * band-gap or not, select of HVD/LVD monitor and select of core/array regulator.
 *
 * @param config  Low-Voltage detect configuration structure.
 */
static inline void PMC0_ConfigureLlsMode(const pmc0_lls_mode_config_t *config)
{
    /*
     * The valid values of Core Regulator Voltage Level are between 0 and 50.
     * These values correspond to a valid range of 0.596V to 1.138V with resolution of 10.83mV.
     * The reset value correspond to 0.704V.
     */
    assert(config);
    assert(config->coreRegulatorVoltLevel <= CORE_REGULATOR_VOLT_LEVEL_MAX);

    PMC0->LLS = (*((const uint32_t *)config)) &
                (PMC0_LLS_ARRAYREG_MASK | PMC0_LLS_COREREG_MASK | PMC0_LLS_MON1P2LVDHP_MASK |
                 PMC0_LLS_MON1P2HVDHP_MASK | PMC0_LLS_FBGHP_MASK | PMC0_LLS_COREREGVL_MASK | PMC0_LLS_RBBEN_MASK);
}

/*!
 * @brief Configure the VLLS power mode.
 *
 * This function configures the VLLS power mode, including turn on force HP
 * band-gap or not, select of HVD/LVD monitor and select of core/array regulator.
 *
 * The select of array regulator is different from the other mode configurations.
 * PMC 0 VLLS config has three array regulator select options where the others have only the latter two,
 * see @ref pmc0_vlls_array_regulator_select_t.
 * Three array regulator select options in PMC 0 VLLS config are shown below:
 * - Regulator is off (diffrentiator)
 * - LP Regulator is on
 * - HP Regulator is on
 *
 * @param config  Low-Voltage detect configuration structure.
 */
static inline void PMC0_ConfigureVllsMode(const pmc0_vlls_mode_config_t *config)
{
    assert(config);

    PMC0->VLLS = (*((const uint32_t *)config)) & (PMC0_VLLS_ARRAYREG_MASK | PMC0_VLLS_MON1P2LVDHP_MASK |
                                                  PMC0_VLLS_MON1P2HVDHP_MASK | PMC0_VLLS_FBGHP_MASK);
}

/*!
 * @brief Gets PMC 0 status flags.
 *
 * This function gets all PMC 0 status flags. The flags are returned as the logical
 * OR value of the enumerators @ref _pmc0_status_flags. To check for a specific status,
 * compare the return value with enumerators in the @ref _pmc0_status_flags.
 * For example, to check whether core regulator voltage level is changing:
 * @code
 *     if (kPMC0_CoreRegulatorVoltLevelFlag & PMC0_GetStatusFlags(void))
 *     {
 *         ...
 *     }
 * @endcode
 *
 * @return PMC 0 status flags which are ORed by the enumerators in the _pmc0_status_flags.
 */
static inline uint32_t PMC0_GetStatusFlags(void)
{
    return PMC0->STATUS;
}

/*!
 * @brief Enables the 1.2V Low-Voltage Detector interrupt.
 *
 * This function enables the 1.2V Low-Voltage Detector interrupt.
 *
 */
static inline void PMC0_EnableLowVoltDetectInterrupt(void)
{
    PMC0->CTRL |= PMC0_CTRL_LVD1P2IE_MASK;
}

/*!
 * @brief Disables the 1.2V Low-Voltage Detector interrupt.
 *
 * This function disables the 1.2V Low-Voltage Detector interrupt.
 *
 */
static inline void PMC0_DisableLowVoltDetectInterrupt(void)
{
    PMC0->CTRL &= ~PMC0_CTRL_LVD1P2IE_MASK;
}

/*!
 * @brief Clears the 1.2V Low-Voltage Detector flag.
 *
 * This function enables the 1.2V Low-Voltage Detector flag.
 *
 */
static inline void PMC0_ClearLowVoltDetectFlag(void)
{
    PMC0->CTRL |= PMC0_CTRL_LVD1P2ACK_MASK;
}

/*!
 * @brief Enables the 1.2V High-Voltage Detector interrupt.
 *
 * This function enables the 1.2V High-Voltage Detector interrupt.
 *
 */
static inline void PMC0_EnableHighVoltDetectInterrupt(void)
{
    PMC0->CTRL |= PMC0_CTRL_HVD1P2IE_MASK;
}

/*!
 * @brief Disables the 1.2V High-Voltage Detector interrupt.
 *
 * This function disables the 1.2V High-Voltage Detector interrupt.
 *
 */
static inline void PMC0_DisableHighVoltDetectInterrupt(void)
{
    PMC0->CTRL &= ~PMC0_CTRL_HVD1P2IE_MASK;
}

/*!
 * @brief Clears the 1.2V High-Voltage Detector flag.
 *
 * This function enables the 1.2V High-Voltage Detector flag.
 *
 */
static inline void PMC0_ClearHighVoltDetectFlag(void)
{
    PMC0->CTRL |= PMC0_CTRL_HVD1P2ACK_MASK;
}

/*!
 * @brief Enables the 1.2V Low-Voltage Detector reset.
 *
 * This function enables 1.2V Low-Voltage Detector reset.
 *
 * @param enable Switcher of 1.2V Low-Voltage Detector reset feature. "true" means to enable, "false" means not.
 */
static inline void PMC0_EnableLowVoltDetectReset(bool enable)
{
    if (enable)
    {
        PMC0->CTRL |= PMC0_CTRL_LVD1P2RE_MASK;
    }
    else
    {
        PMC0->CTRL &= ~PMC0_CTRL_LVD1P2RE_MASK;
    }
}

/*!
 * @brief Enables the 1.2V High-Voltage Detector reset.
 *
 * This function enables 1.2V High-Voltage Detector reset.
 *
 * @param enable Switcher of 1.2V High-Voltage Detector reset feature. "true" means to enable, "false" means not.
 */
static inline void PMC0_EnableHighVoltDetectReset(bool enable)
{
    if (enable)
    {
        PMC0->CTRL |= PMC0_CTRL_HVD1P2RE_MASK;
    }
    else
    {
        PMC0->CTRL &= ~PMC0_CTRL_HVD1P2RE_MASK;
    }
}

/*!
 * @brief Releases/clears the isolation in the PADS.
 *
 * This function releases/clears the isolation in the PADS.
 *
 * The isolation in the pads only will be asserted during LLS/VLLS. On LLS exit, the isolation will
 * release automatically. ISOACK must be set after a VLLS to RUN mode transition has completed.
 *
 */
static inline void PMC0_ClearPadsIsolation(void)
{
    PMC0->CTRL |= PMC0_CTRL_ISOACK_MASK;
}

/*!
 * @brief Enables the PMC 1 Switch RBB.
 *
 * This function enables 1.2V High-Voltage Detector reset.
 *
 * @param enable Switcher of PMC 1 Switch RBB feature. "true" means to enable, "false" means not.
 */
static inline void PMC0_EnablePmc1SwitchRbb(bool enable)
{
    if (enable)
    {
        PMC0->CTRL |= PMC0_CTRL_SWRBBEN_MASK;
    }
    else
    {
        PMC0->CTRL &= ~PMC0_CTRL_SWRBBEN_MASK;
    }
}

/*!
 * @brief Powers on PMC 1.
 *
 * This function powers on PMC 1.
 *
 * When this bit field is asserted the PMC 1 is powered on. This bit would take action only once.
 * This bit will be rearmed after a POR event only.
 *
 */
static inline void PMC0_PowerOnPmc1(void)
{
    PMC0->CTRL |= PMC0_CTRL_PMC1ON_MASK;
}

/*!
 * @brief Enables to wait LDO OK signal.
 *
 * This function enables to wait LDO OK signal.
 *
 * @param enable Switcher of wait LDO OK signal feature. "true" means to enable, "false" means not.
 */
static inline void PMC0_EnableWaitLdoOkSignal(bool enable)
{
    if (enable)
    {
        PMC0->CTRL &= ~PMC0_CTRL_LDOOKDIS_MASK;
    }
    else
    {
        PMC0->CTRL |= PMC0_CTRL_LDOOKDIS_MASK;
    }
}

/*!
 * @brief Enables PMC 1 LDO Regulator.
 *
 * This function enables PMC 1 LDO Regulator.
 *
 * @param enable Switcher of PMC 1 LDO Regulator. "true" means to enable, "false" means not.
 */
static inline void PMC0_EnablePmc1LdoRegulator(bool enable)
{
    if (enable)
    {
        PMC0->CTRL |= PMC0_CTRL_LDOEN_MASK;
    }
    else
    {
        PMC0->CTRL &= ~PMC0_CTRL_LDOEN_MASK;
    }
}

/*!
 * @brief Configures PMC 0 SRAM bank power down.
 *
 * This function configures PMC 0 SRAM bank power down.
 *
 * The bit i controls the power mode of the PMC 0 SRAM bank i.
 * PMC0_SRAM_PD[i] = 1'b0 - PMC 0 SRAM bank i is not affected.
 * PMC0_SRAM_PD[i] = 1'b1 - PMC 0 SRAM bank i is in ASD or ARRAY_SHUTDOWN during all modes,
 * except VLLS. During VLLS is in POWER_DOWN mode.
 *
 * Example
 * @code
 *     // Enable band 0 and 1 in ASD or ARRAY_SHUTDOWN during all modes except VLLS
 *     PMC0_ConfigureSramBankPowerDown(0x3U);
 * @endcode
 *
 * @param bankMask The bands to enable. Logical OR of all bits of band index to enbale.
 */
static inline void PMC0_ConfigureSramBankPowerDown(uint32_t bankMask)
{
    PMC0->SRAMCTRL_0 = PMC0_SRAMCTRL_0_SRAM_PD(bankMask);
}

/*!
 * @brief Configures PMC 0 SRAM bank power down in stop modes.
 *
 * This function configures PMC 0 SRAM bank power down in stop modes.
 *
 * The bit i controls the PMC 0 SRAM bank i.
 * PMC0_SRAM_PDS[i] = 1'b0 - PMC 0 SRAM bank i is not affected.
 * PMC0_SRAM_PDS[i] = 1'b1 - PMC 0 SRAM bank i is in ASD or ARRAY_SHUTDOWN mode during
 * STOP, VLPS and LLS modes. During VLLS is in POWER_DOWN mode.
 *
 * Example
 * @code
 *     // Enable band 0 and 1 in ASD or ARRAY_SHUTDOWN during STOP, VLPS and LLS modes
 *     PMC0_ConfigureSramBankPowerDownStopMode(0x3U);
 * @endcode
 *
 * @param bankMask The bands to enable. Logical OR of all bits of band index to enbale.
 */
static inline void PMC0_ConfigureSramBankPowerDownStopMode(uint32_t bankMask)
{
    PMC0->SRAMCTRL_1 = PMC0_SRAMCTRL_1_SRAM_PDS(bankMask);
}

/*!
 * @brief Configures PMC 0 SRAM bank power down in Standby Mode.
 *
 * This function configures PMC 0 SRAM bank power down in Standby Mode.
 *
 * The bit i controls the PMC 0 SRAM bank i.
 * PMC0_SRAM_STDY[i] = 1'b0 - PMC 0 SRAM bank i is not affected.
 * PMC0_SRAM_STDY[i] = 1'b1 - PMC 0 SRAM bank i is in STANDBY mode during all modes (except VLLS and LLS).
 *
 * Example
 * @code
 *     // Enable band 0 and 1 in STANDBY mode except VLLS and LLS
 *     PMC0_ConfigureSramBankPowerDownStandbyMode(0x3U);
 * @endcode
 *
 * @param bankMask The bands to enable. Logical OR of all bits of band index to enbale.
 */
static inline void PMC0_ConfigureSramBankPowerDownStandbyMode(uint32_t bankMask)
{
    PMC0->SRAMCTRL_2 = PMC0_SRAMCTRL_2_SRAM_STDY(bankMask);
}
/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* _FSL_PMC_H_*/
