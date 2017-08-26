
#include "fsl_debug_console.h"

#include "fsl_pmc.h"

#include "fsl_lpsci.h"

#include "board.h"
#include "app.h"
#include "fsl_lptmr.h"
#include "bsp.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifdef PRINTF
#undef PRINTF
#define PRINTF
#endif

#define kAdcChannelTemperature (26U) /*! ADC channel of temperature sensor */
#define kAdcChannelBandgap (27U)     /*! ADC channel of BANDGAP */

#define UPPER_VALUE_LIMIT (1U) /*! This value/10 is going to be added to current Temp to set the upper boundary*/
#define LOWER_VALUE_LIMIT                                                               \
    (1U) /*! This Value/10 is going to be subtracted from current Temp to set the lower \
            boundary*/
#define UPDATE_BOUNDARIES_TIME                                                       \
    (20U) /*! This value indicates the number of cycles needed to update boundaries. \
              To know the Time it will take, multiply this value times LPTMR_COMPARE_VALUE*/

#define LPTMR_COMPARE_VALUE      ADC_TRIGGER_PERIOD    //(20U) /* Low Power Timer interrupt time in miliseconds */

#define BUFFER_LEN 4
/*!
* @brief Boundaries struct
*/
typedef struct lowPowerAdcBoundaries
{
    int32_t upperBoundary; /*! upper boundary in degree */
    int32_t lowerBoundary; /*! lower boundary in degree */
} lowPowerAdcBoundaries_t;

/*! @brief Flash driver Structure */
static flash_config_t s_flashDriver;
/*! @brief Buffer for program */
static uint32_t s_buffer[BUFFER_LEN];
flash_security_state_t securityStatus = kFLASH_SecurityStateNotSecure; /* Return protection status */
uint32_t pflashBlockBase = 0;
uint32_t pflashTotalSize = 0;
uint32_t pflashSectorSize = 0;

uint32_t EE_SettingAddr;


UART_Comm_t  g_Uart0Comm;
UART_Comm_t  g_Uart1Comm;
UART_Comm_t  g_Uart2Comm;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t adcValue = 0; /*! ADC value */
static uint32_t adcrTemp25 = 0;        /*! Calibrated ADCR_TEMP25 */
static uint32_t adcr100m = 0;
volatile bool conversionCompleted = false; /*! Conversion is completed Flag */

/*******************************************************************************
 * Code
 ******************************************************************************/
/* Enable the trigger source of LPTimer */
void LPTMR_InitTriggerSourceOfAdc(LPTMR_Type *base)
{
    lptmr_config_t lptmrUserConfig;

    /*
     * lptmrUserConfig.timerMode = kLPTMR_TimerModeTimeCounter;
     * lptmrUserConfig.pinSelect = kLPTMR_PinSelectInput_0;
     * lptmrUserConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
     * lptmrUserConfig.enableFreeRunning = false;
     * lptmrUserConfig.bypassPrescaler = true;
     * lptmrUserConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
     * lptmrUserConfig.value = kLPTMR_Prescale_Glitch_0;
     */
    LPTMR_GetDefaultConfig(&lptmrUserConfig);
    /* Init LPTimer driver */
    LPTMR_Init(base, &lptmrUserConfig);

    /* Set the LPTimer period */
    LPTMR_SetTimerPeriod(base, LPTMR_COMPARE_VALUE);

    /* Start the LPTimer */
    LPTMR_StartTimer(base);

    /* Configure SIM for ADC hw trigger source selection */
    BOARD_ConfigTriggerSource();
}

/*!
 * @brief ADC stop conversion
 */
void ADC16_PauseConversion(ADC_Type *base)
{
    adc16_channel_config_t adcChnConfig;

    adcChnConfig.channelNumber = 31U; /*!< AD31 channel */
    adcChnConfig.enableInterruptOnConversionCompleted = false;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adcChnConfig.enableDifferentialConversion = false;
#endif
    ADC16_SetChannelConfig(base, DEMO_ADC16_CHANNEL_GROUP, &adcChnConfig);
}

/*!
 * @brief calibrate parameters: VDD and ADCR_TEMP25
 */
void ADC16_CalibrateParams(ADC_Type *base)
{
    uint32_t bandgapValue = 0; /*! ADC value of BANDGAP */
    uint32_t vdd = 0;          /*! VDD in mV */

    adc16_config_t adcUserConfig;
    adc16_channel_config_t adcChnConfig;
    pmc_bandgap_buffer_config_t pmcBandgapConfig;

    pmcBandgapConfig.enable = true;

#if (defined(FSL_FEATURE_PMC_HAS_BGEN) && FSL_FEATURE_PMC_HAS_BGEN)
    pmcBandgapConfig.enableInLowPowerMode = false;
#endif
#if (defined(FSL_FEATURE_PMC_HAS_BGBDS) && FSL_FEATURE_PMC_HAS_BGBDS)
    pmcBandgapConfig.drive = kPmcBandgapBufferDriveLow;
#endif
    /* Enable BANDGAP reference voltage */
    PMC_ConfigureBandgapBuffer(PMC, &pmcBandgapConfig);

    /*
    * Initialization ADC for
    * 16bit resolution, interrupt mode, hw trigger disabled.
    * normal convert speed, VREFH/L as reference,
    * disable continuous convert mode
    */
    /*
     * adcUserConfig.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
     * adcUserConfig.clockSource = kADC16_ClockSourceAsynchronousClock;
     * adcUserConfig.enableAsynchronousClock = true;
     * adcUserConfig.clockDivider = kADC16_ClockDivider8;
     * adcUserConfig.resolution = kADC16_ResolutionSE12Bit;
     * adcUserConfig.longSampleMode = kADC16_LongSampleDisabled;
     * adcUserConfig.enableHighSpeed = false;
     * adcUserConfig.enableLowPower = false;
     * adcUserConfig.enableContinuousConversion = false;
     */
    ADC16_GetDefaultConfig(&adcUserConfig);
#if defined(FSL_FEATURE_ADC16_MAX_RESOLUTION) && (FSL_FEATURE_ADC16_MAX_RESOLUTION >= 16U)
    adcUserConfig.resolution = kADC16_Resolution16Bit;
#else
    adcUserConfig.resolution = kADC16_ResolutionSE12Bit;
#endif
    adcUserConfig.enableContinuousConversion = false;
    adcUserConfig.clockSource = kADC16_ClockSourceAsynchronousClock;
    adcUserConfig.enableLowPower = 1;
    adcUserConfig.longSampleMode = kADC16_LongSampleCycle24;
#ifdef BOARD_ADC_USE_ALT_VREF
    adcUserConfig.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
    ADC16_Init(base, &adcUserConfig);

#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    /* Auto calibration */
    ADC16_DoAutoCalibration(base);
#endif

#if defined(FSL_FEATURE_ADC16_HAS_HW_AVERAGE) && FSL_FEATURE_ADC16_HAS_HW_AVERAGE
    /* Use hardware average to increase stability of the measurement  */
    ADC16_SetHardwareAverage(base, kADC16_HardwareAverageCount32);
#endif /* FSL_FEATURE_ADC16_HAS_HW_AVERAGE */

    adcChnConfig.channelNumber = kAdcChannelBandgap;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adcChnConfig.enableDifferentialConversion = false;
#endif
    adcChnConfig.enableInterruptOnConversionCompleted = false;
    ADC16_SetChannelConfig(base, DEMO_ADC16_CHANNEL_GROUP, &adcChnConfig);

    /* Wait for the conversion to be done */
    while (!ADC16_GetChannelStatusFlags(base, DEMO_ADC16_CHANNEL_GROUP))
    {
    }

    /* Get current ADC BANDGAP value */
    bandgapValue = ADC16_GetChannelConversionValue(base, DEMO_ADC16_CHANNEL_GROUP);

    ADC16_PauseConversion(base);

    /* Get VDD value measured in mV: VDD = (ADCR_VDD x V_BG) / ADCR_BG */
    vdd = ADCR_VDD * V_BG / bandgapValue;
    /* Calibrate ADCR_TEMP25: ADCR_TEMP25 = ADCR_VDD x V_TEMP25 / VDD */
    adcrTemp25 = ADCR_VDD * V_TEMP25 / vdd;
    /* ADCR_100M = ADCR_VDD x tempM x 100 / VDD */
    adcr100m = (ADCR_VDD * tempM) / (vdd * 10);

    /* Disable BANDGAP reference voltage */
    pmcBandgapConfig.enable = false;
    PMC_ConfigureBandgapBuffer(PMC, &pmcBandgapConfig);
}

/*!
 * @brief Initialize the ADCx for Hardware trigger.
 */
bool ADC16_InitHardwareTrigger(ADC_Type *base)
{
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    uint16_t offsetValue = 0; /*!< Offset error from correction value. */
#endif
    adc16_config_t adcUserConfig;
    adc16_channel_config_t adcChnConfig;

#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    /* Auto calibration */
    ADC16_DoAutoCalibration(base);
    offsetValue = base->OFS;
    ADC16_SetOffsetValue(base, offsetValue);
#endif
    /*
    * Initialization ADC for
    * 16bit resolution, interrupt mode, hw trigger enabled.
    * normal convert speed, VREFH/L as reference,
    * disable continuous convert mode.
    */
    /*
     * adcUserConfig.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
     * adcUserConfig.clockSource = kADC16_ClockSourceAsynchronousClock;
     * adcUserConfig.enableAsynchronousClock = true;
     * adcUserConfig.clockDivider = kADC16_ClockDivider8;
     * adcUserConfig.resolution = kADC16_ResolutionSE12Bit;
     * adcUserConfig.longSampleMode = kADC16_LongSampleDisabled;
     * adcUserConfig.enableHighSpeed = false;
     * adcUserConfig.enableLowPower = false;
     * adcUserConfig.enableContinuousConversion = false;
     */
    ADC16_GetDefaultConfig(&adcUserConfig);
#if defined(FSL_FEATURE_ADC16_MAX_RESOLUTION) && (FSL_FEATURE_ADC16_MAX_RESOLUTION >= 16U)
    adcUserConfig.resolution = kADC16_Resolution16Bit;
#else
    adcUserConfig.resolution = kADC16_ResolutionSE12Bit;
#endif
    /* enabled hardware trigger  */
    ADC16_EnableHardwareTrigger(base, true);
    adcUserConfig.enableContinuousConversion = false;
    adcUserConfig.clockSource = kADC16_ClockSourceAsynchronousClock;

    adcUserConfig.longSampleMode = kADC16_LongSampleCycle24;
    adcUserConfig.enableLowPower = 1;
#if ((defined BOARD_ADC_USE_ALT_VREF) && BOARD_ADC_USE_ALT_VREF)
    adcUserConfig.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
    ADC16_Init(base, &adcUserConfig);

    adcChnConfig.channelNumber = kAdcChannelTemperature;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adcChnConfig.enableDifferentialConversion = false;
#endif
    adcChnConfig.enableInterruptOnConversionCompleted = true;
    /* Configure channel 0 */
    ADC16_SetChannelConfig(base, DEMO_ADC16_CHANNEL_GROUP, &adcChnConfig);
    return true;
}

static int32_t GetCurrentTempValue(void)
{
    int32_t currentTemperature = 0;
    /* Temperature = 25 - (ADCR_T - ADCR_TEMP25) * 100 / ADCR_100M */
    currentTemperature = (int32_t)(STANDARD_TEMP - ((int32_t)adcValue - (int32_t)adcrTemp25) * 100 / (int32_t)adcr100m);
    return currentTemperature;
}

//static lowPowerAdcBoundaries_t TempSensorCalibration(uint32_t updateBoundariesCounter, int32_t *tempArray)
//{
//    uint32_t avgTemp = 0;
//    lowPowerAdcBoundaries_t boundaries;
//
//    for (int i = 0; i < updateBoundariesCounter; i++)
//    {
//        avgTemp += tempArray[i];
//    }
//    /* Get average temperature */
//    avgTemp /= updateBoundariesCounter;
//
//    /* Set upper boundary */
//    boundaries.upperBoundary = avgTemp + UPPER_VALUE_LIMIT;
//
//    /* Set lower boundary */
//    boundaries.lowerBoundary = avgTemp - LOWER_VALUE_LIMIT;
//
//    return boundaries;
//}

/*!
 * @brief ADC Interrupt handler
 *
 * Get current ADC value and set conversionCompleted flag.
 */
void DEMO_ADC16_IRQ_HANDLER_FUNC(void)
{
    /* Get current ADC value */
    adcValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP);
    /* Set conversionCompleted flag. This prevents an wrong conversion in main function */
    conversionCompleted = true;
}


uint8_t init_eeprom(void)
{
    status_t result;    /* Return code from each flash driver function */

    /* Clean up Flash driver Structure*/
    memset(&s_flashDriver, 0, sizeof(flash_config_t));

    /* Setup flash driver structure for device and initialize variables. */
    result = FLASH_Init(&s_flashDriver);
    if (kStatus_FLASH_Success != result)
    {
        error_trap();
    }
    /* Get flash properties*/
    FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflashBlockBaseAddr, &pflashBlockBase);
    FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflashTotalSize, &pflashTotalSize);
    FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflashSectorSize, &pflashSectorSize);

    /* print welcome message */
    PRINTF("\r\n PFlash Example Start \r\n");
    /* Print flash information - PFlash. */
    PRINTF("\r\n PFlash Information: ");
    PRINTF("\r\n Total Program Flash Size:\t%d KB, Hex: (0x%x)", (pflashTotalSize / 1024), pflashTotalSize);
    PRINTF("\r\n Program Flash Sector Size:\t%d KB, Hex: (0x%x) ", (pflashSectorSize / 1024), pflashSectorSize);

    /* Check security status. */
    result = FLASH_GetSecurityState(&s_flashDriver, &securityStatus);
    if (kStatus_FLASH_Success != result)
    {
        error_trap();
    }
    /* Print security status. */
    switch (securityStatus)
    {
        case kFLASH_SecurityStateNotSecure:
            PRINTF("\r\n Flash is UNSECURE!");
            break;
        case kFLASH_SecurityStateBackdoorEnabled:
            PRINTF("\r\n Flash is SECURE, BACKDOOR is ENABLED!");
            break;
        case kFLASH_SecurityStateBackdoorDisabled:
            PRINTF("\r\n Flash is SECURE, BACKDOOR is DISABLED!");
            break;
        default:
            break;
    }
    PRINTF("\r\n");

    EE_SettingAddr = pflashBlockBase + (pflashTotalSize - pflashSectorSize);
    return 1;
}

uint8_t get_setting_from_eeprom(uint8_t *pBuf, uint32_t len)
{
    int i;
    uint32_t addr = EE_SettingAddr;
    for (i=0; i<len; i++)
    {
        pBuf[i] = *(volatile uint8_t *)addr;
        addr++;
    }
    if ((pBuf[0]==0xAA) && (pBuf[1]==0xBB) && (pBuf[2]==0xCC) && (pBuf[3]==0xDD)
        && (pBuf[4]==0xEE) && (pBuf[5]==0xFF))
        return 1;
    else
        return 0;
}
uint8_t store_setting_to_eeprom(uint32_t *pBuf, uint32_t len)
{
    status_t result;    /* Return code from each flash driver function */
    uint32_t destAdrss; /* Address of the target location */
    uint32_t i, failAddr, failDat;

    result = FLASH_GetSecurityState(&s_flashDriver, &securityStatus);
    if (kStatus_FLASH_Success != result)
    {
        error_trap();
    }
    /* Test pflash basic opeation only if flash is unsecure. */
    if (kFLASH_SecurityStateNotSecure == securityStatus)
    {
        /* Debug message for user. */
        /* Erase several sectors on upper pflash block where there is no code */
        PRINTF("\r\n Erase a sector of flash");

        /* Erase a sector from destAdrss. */
        //destAdrss = pflashBlockBase + (pflashTotalSize - pflashSectorSize);
        destAdrss =  EE_SettingAddr;
        result = FLASH_Erase(&s_flashDriver, destAdrss, pflashSectorSize, kFLASH_ApiEraseKey);
        if (kStatus_FLASH_Success != result)
        {
            error_trap();
        }

        /* Verify sector if it's been erased. */
        result = FLASH_VerifyErase(&s_flashDriver, destAdrss, pflashSectorSize, kFLASH_MarginValueUser);
        if (kStatus_FLASH_Success != result)
        {
            error_trap();
        }

        /* Print message for user. */
        PRINTF("\r\n Successfully Erased Sector 0x%x -> 0x%x\r\n", destAdrss, (destAdrss + pflashSectorSize));

        /* Print message for user. */
        PRINTF("\r\n Program a buffer to a sector of flash ");
        /* Prepare buffer. */
//        for (i = 0; i < BUFFER_LEN; i++)
//        {
//            s_buffer[i] = i;
//        }
//        destAdrss = pflashBlockBase + (pflashTotalSize - pflashSectorSize);
        result = FLASH_Program(&s_flashDriver, destAdrss, pBuf, len);
        if (kStatus_FLASH_Success != result)
        {
            error_trap();
        }

        /* Program Check user margin levels */
        result = FLASH_VerifyProgram(&s_flashDriver, destAdrss, len, pBuf, kFLASH_MarginValueUser,
                                     &failAddr, &failDat);
        if (kStatus_FLASH_Success != result)
        {
            error_trap();
        }
        PRINTF("\r\n Successfully Programmed and Verified Location 0x%x -> 0x%x \r\n", destAdrss,
               (destAdrss + len));
    }
    else
    {
        PRINTF("\r\n Erase/Program opeation will not be executed, as Flash is SECURE!");
    }

    /* Print finished message. */
    PRINTF("\r\n End of PFlash Example \r\n");

    return 1;
}

/*!
 * @brief Use Standard Software Drivers (SSD) to modify pflash.
 *
 * @details This function uses SSD to demonstrate flash mode:
 *            + Check pflash information.
 *            + Erase a sector and verify.
 *            + Program a sector and verify.
 */
void pflash_test(void)
{
    //flash_security_state_t securityStatus = kFLASH_SecurityStateNotSecure; /* Return protection status */
    status_t result;    /* Return code from each flash driver function */
    uint32_t destAdrss; /* Address of the target location */
    uint32_t i, failAddr, failDat;

    //uint32_t pflashBlockBase = 0;
    //uint32_t pflashTotalSize = 0;
    //uint32_t pflashSectorSize = 0;

    /* Init hardware */
//    BOARD_InitHardware();

    /* Clean up Flash driver Structure*/
    memset(&s_flashDriver, 0, sizeof(flash_config_t));

    /* Setup flash driver structure for device and initialize variables. */
    result = FLASH_Init(&s_flashDriver);
    if (kStatus_FLASH_Success != result)
    {
        error_trap();
    }
    /* Get flash properties*/
    FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflashBlockBaseAddr, &pflashBlockBase);
    FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflashTotalSize, &pflashTotalSize);
    FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflashSectorSize, &pflashSectorSize);

    /* print welcome message */
    PRINTF("\r\n PFlash Example Start \r\n");
    /* Print flash information - PFlash. */
    PRINTF("\r\n PFlash Information: ");
    PRINTF("\r\n Total Program Flash Size:\t%d KB, Hex: (0x%x)", (pflashTotalSize / 1024), pflashTotalSize);
    PRINTF("\r\n Program Flash Sector Size:\t%d KB, Hex: (0x%x) ", (pflashSectorSize / 1024), pflashSectorSize);

    /* Check security status. */
    result = FLASH_GetSecurityState(&s_flashDriver, &securityStatus);
    if (kStatus_FLASH_Success != result)
    {
        error_trap();
    }
    /* Print security status. */
    switch (securityStatus)
    {
        case kFLASH_SecurityStateNotSecure:
            PRINTF("\r\n Flash is UNSECURE!");
            break;
        case kFLASH_SecurityStateBackdoorEnabled:
            PRINTF("\r\n Flash is SECURE, BACKDOOR is ENABLED!");
            break;
        case kFLASH_SecurityStateBackdoorDisabled:
            PRINTF("\r\n Flash is SECURE, BACKDOOR is DISABLED!");
            break;
        default:
            break;
    }
    PRINTF("\r\n");

    /* Test pflash basic opeation only if flash is unsecure. */
    if (kFLASH_SecurityStateNotSecure == securityStatus)
    {
        /* Debug message for user. */
        /* Erase several sectors on upper pflash block where there is no code */
        PRINTF("\r\n Erase a sector of flash");

        /* Erase a sector from destAdrss. */
        destAdrss = pflashBlockBase + (pflashTotalSize - pflashSectorSize);
        result = FLASH_Erase(&s_flashDriver, destAdrss, pflashSectorSize, kFLASH_ApiEraseKey);
        if (kStatus_FLASH_Success != result)
        {
            error_trap();
        }

        /* Verify sector if it's been erased. */
        result = FLASH_VerifyErase(&s_flashDriver, destAdrss, pflashSectorSize, kFLASH_MarginValueUser);
        if (kStatus_FLASH_Success != result)
        {
            error_trap();
        }

        /* Print message for user. */
        PRINTF("\r\n Successfully Erased Sector 0x%x -> 0x%x\r\n", destAdrss, (destAdrss + pflashSectorSize));

        /* Print message for user. */
        PRINTF("\r\n Program a buffer to a sector of flash ");
        /* Prepare buffer. */
        for (i = 0; i < BUFFER_LEN; i++)
        {
            s_buffer[i] = i+1;
        }
        destAdrss = pflashBlockBase + (pflashTotalSize - pflashSectorSize);
        result = FLASH_Program(&s_flashDriver, destAdrss, s_buffer, sizeof(s_buffer));
        if (kStatus_FLASH_Success != result)
        {
            error_trap();
        }

        /* Program Check user margin levels */
        result = FLASH_VerifyProgram(&s_flashDriver, destAdrss, sizeof(s_buffer), s_buffer, kFLASH_MarginValueUser,
                                     &failAddr, &failDat);
        if (kStatus_FLASH_Success != result)
        {
            error_trap();
        }
        PRINTF("\r\n Successfully Programmed and Verified Location 0x%x -> 0x%x \r\n", destAdrss,
               (destAdrss + sizeof(s_buffer)));
    }
    else
    {
        PRINTF("\r\n Erase/Program opeation will not be executed, as Flash is SECURE!");
    }

    /* Print finished message. */
    PRINTF("\r\n End of PFlash Example \r\n");
//    while (1)
//    {
//    }
}

void BSP_UART0_Init(void)
{
    lpsci_config_t config;
    uint32_t instance;
    /*
     * config.parityMode = kLPSCI_ParityDisabled;
     * config.stopBitCount = kLPSCI_OneStopBit;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    LPSCI_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    LPSCI_Init(DEMO_LPSCI, &config, CLOCK_GetFreq(DEMO_LPSCI_CLKSRC));
    /* Enable interrupt in NVIC. */
    EnableIRQ(UART0_IRQn);

    LPSCI_EnableInterrupts(DEMO_LPSCI, kLPSCI_RxDataRegFullInterruptEnable | kLPSCI_RxOverrunInterruptEnable);

    g_Uart0Comm.rxBuf_readIdx = 0;
    g_Uart0Comm.rxBuf_writeIdx = 0;
    g_Uart0Comm.txBuf_readIdx = 0;
    g_Uart0Comm.txBuf_writeIdx = 0;
    g_Uart0Comm.rxFullCallback = BSP_UART0_RxBuffFullCallback;
}

uint16_t BSP_UART0_SendData(uint8_t *buf, uint16_t size)
{
    int i, flag = 0;

    if (BSP_UART0_GetTxBufDataSize() == 0)
        flag = 1;
    for (i=0; i<size; i++)
    {
        if ((g_Uart0Comm.txBuf_writeIdx + 1) % UART_TX_BUFF_SIZE == g_Uart0Comm.txBuf_readIdx)//tx buffer is full
        {
            break;
        }
        g_Uart0Comm.txBuff[g_Uart0Comm.txBuf_writeIdx] = buf[i];
        g_Uart0Comm.txBuf_writeIdx = (g_Uart0Comm.txBuf_writeIdx + 1) % UART_TX_BUFF_SIZE;
    }
    if (flag == 1)
    {
        /* Enable transmiter interrupt. */
        LPSCI_EnableInterrupts(DEMO_LPSCI, kLPSCI_TxDataRegEmptyInterruptEnable);
       //UART0_BWR_C2_TIE(UART0, 1U);/* Enable the transmitter data register empty interrupt.*/
    }
    return i;// the real size that put into the tx buffer
}

uint16_t BSP_UART0_GetTxBufDataSize(void)
{
    return (g_Uart0Comm.txBuf_writeIdx - g_Uart0Comm.txBuf_readIdx + UART_TX_BUFF_SIZE) % UART_TX_BUFF_SIZE;
}
uint16_t BSP_UART0_GetTxBufRemainSize(void)
{
    return UART_TX_BUFF_SIZE - BSP_UART0_GetTxBufDataSize() - 1;
}

uint16_t BSP_UART0_GetRxBufDataSize(void)
{
    return (g_Uart0Comm.rxBuf_writeIdx - g_Uart0Comm.rxBuf_readIdx + UART_RX_BUFF_SIZE) % UART_RX_BUFF_SIZE;
}

void BSP_UART0_RxBuffFullCallback(void *comm)
{
    UART_Comm_t *pComm = (UART_Comm_t *)comm;

}

uint16_t BSP_UART0_ReceiveData(uint8_t *buf, uint16_t bufSize)
{
    int i;
    for (i=0; i<bufSize; i++)
    {
        if (g_Uart0Comm.rxBuf_readIdx == g_Uart0Comm.rxBuf_writeIdx)// rx buffer is empty
        {
            break;
        }
        buf[i] = g_Uart0Comm.rxBuff[g_Uart0Comm.rxBuf_readIdx];
        g_Uart0Comm.rxBuf_readIdx = (g_Uart0Comm.rxBuf_readIdx + 1) % UART_RX_BUFF_SIZE;
    }
    return i; //received data size
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART0_IRQHandler
 * Description   : Interrupt handler for LPSCI.
 * This is not a public API as it is called whenever an interrupt occurs.
 *
 *END**************************************************************************/
void UART0_IRQHandler(void)
{
    /* Handle Rx Data Register Full interrupt */
    if((UART0_S1_RDRF_MASK & UART0->S1) && (UART0_C2_RIE_MASK & UART0->C2))
    {
        /* Get data and put in receive buffer */
        //LPSCI_HAL_Getchar(UART0, &);
        g_Uart0Comm.rxBuff[g_Uart0Comm.rxBuf_writeIdx] = UART0->D;
        g_Uart0Comm.rxBuf_writeIdx = (g_Uart0Comm.rxBuf_writeIdx + 1) % UART_RX_BUFF_SIZE;
        if ((g_Uart0Comm.rxBuf_writeIdx + 1) % UART_RX_BUFF_SIZE == g_Uart0Comm.rxBuf_readIdx)// the buffer is full
        {
            g_Uart0Comm.rxFullCallback(&g_Uart0Comm);
        }
    }

    /* Handle Tx Data Register Empty interrupt */
    if((UART0->S1 & UART0_S1_TDRE_MASK) && (UART0->C2 & UART0_C2_TIE_MASK))
    {
        if (g_Uart0Comm.txBuf_readIdx == g_Uart0Comm.txBuf_writeIdx)// tx  buffer is empty
        {
            /* Disable TX register empty interrupt. */
            UART0->C2 &= ~UART0_C2_TIE_MASK;
        }
        else
        {
            /* Transmit data and update tx size/buff. */
            UART0->D = g_Uart0Comm.txBuff[g_Uart0Comm.txBuf_readIdx];
            g_Uart0Comm.txBuf_readIdx = (g_Uart0Comm.txBuf_readIdx + 1) % UART_TX_BUFF_SIZE;
        }
    }

    /* Handle receive overrun interrupt */
    if (UART0_S1_OR_MASK & UART0->S1)
    {
        /* Clear the flag, OR the rxDataRegFull will not be set any more */
        UART0->S1 |= UART0_S1_OR_MASK;
    }
}

