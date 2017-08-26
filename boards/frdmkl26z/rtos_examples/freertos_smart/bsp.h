#ifndef _BSP_H
#define _BSP_H
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "fsl_flash.h"
#include "fsl_smc.h"
#include "fsl_adc16.h"

#define DEMO_ADC16_BASEADDR ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U

#define DEMO_ADC16_IRQ_ID ADC0_IRQn
#define DEMO_ADC16_IRQ_HANDLER_FUNC ADC0_IRQHandler

#define DEMO_LPTMR_BASE LPTMR0

/*
 * These values are used to get the temperature. DO NOT MODIFY
 * The method used in this demo to calculate temperature of chip is mapped to
 * Temperature Sensor for the HCS08 Microcontroller Family document (Document Number: AN3031)
 */
#define ADCR_VDD (65535U) /* Maximum value when use 16b resolution */
#define V_BG (1000U)      /* BANDGAP voltage in mV (trim to 1.0V) */
#define V_TEMP25 (716U)   /* Typical VTEMP25 in mV */
#define tempM (1620U)         /* Typical slope: (mV x 1000)/oC */
#define STANDARD_TEMP (25U)

#define LED_GPIO                   GPIOD
#define LED_GPIO_PIN               4U
#define WIFI_EN_GPIO               GPIOA
#define WIFI_EN_GPIO_PIN           4U
#define SW_GPIO                    GPIOC
#define SW_GPIO_PORT               PORTC
#define SW_GPIO_PIN                5U

#define LED_GREEN_INIT()                                   \
    GPIO_WritePinOutput(LED_GPIO, LED_GPIO_PIN, LOGIC_LED_OFF);\
    LED_GPIO->PDDR |= (1U << LED_GPIO_PIN)  /*!< Enable target LED_GREEN */
#define LED_GREEN_ON() \
    GPIO_ClearPinsOutput(LED_GPIO, 1U << LED_GPIO_PIN) /*!< Turn on target LED_GREEN */
#define LED_GREEN_OFF() \
    GPIO_SetPinsOutput(LED_GPIO, 1U << LED_GPIO_PIN) /*!< Turn off target LED_GREEN */
#define LED_GREEN_TOGGLE() \
    GPIO_TogglePinsOutput(LED_GPIO, 1U << LED_GPIO_PIN) /*!< Toggle on target LED_GREEN */


#define WIFI_EN_INIT()                                   \
    GPIO_WritePinOutput(WIFI_EN_GPIO, WIFI_EN_GPIO_PIN, 1);\
    WIFI_EN_GPIO->PDDR |= (1U << WIFI_EN_GPIO_PIN)  /*!< Enable target LED_GREEN */
#define WIFI_EN_OFF() \
    GPIO_ClearPinsOutput(WIFI_EN_GPIO, 1U << WIFI_EN_GPIO_PIN) /*!< Turn on target LED_GREEN */
#define WIFI_EN_ON() \
    GPIO_SetPinsOutput(WIFI_EN_GPIO, 1U << WIFI_EN_GPIO_PIN) /*!< Turn off target LED_GREEN */

#define UART_RX_BUFF_SIZE                500
#define UART_TX_BUFF_SIZE                500
#define SLEEP_DATA_NODE_SIZE             600

/*! @brief LPSCI receive callback function type. */
typedef void (* uart_rx_callback_t)(void * comm);


typedef struct _ee_setting{
    uint8_t heading_code[6];// fill with 0xAA,0xBB,0xCC,0xDD,0xEE,0xFF
    uint8_t device_id[12];
    uint8_t ssid[33];// ssid_string(32B)+\0
    uint8_t pwd[26];// pwd_string(25B)+\0
    uint8_t serverIP[16];// ip_string(15B)+\0
    uint8_t serverPort[6];// port_string(5B)+\0
    uint16_t count_time;   // time unit to static
    uint32_t send_interval;// time interval to send data
    uint8_t protect_code[12];// this code get from mcu unique id
    uint8_t pad[5];   // only used for pad, fill with random number
}EE_Setting_t;

typedef struct _data_format{
    uint8_t  heart_rate;
    uint8_t  breath_rate;
    uint8_t  turning_flag;
}SleepData_t;
typedef struct _datetime{
    uint8_t year;// compare from 1970
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minite;
    uint8_t second;
}SleepTime_t;
typedef struct _data_packet{
    uint16_t num;
    SleepTime_t time;
    SleepData_t nodeBuf[SLEEP_DATA_NODE_SIZE];
}SleepDataPacket_t;

typedef struct LpsciState {
    uint8_t txBuff[UART_TX_BUFF_SIZE];        /*!< The buffer of data being sent.*/
    uint8_t rxBuff[UART_RX_BUFF_SIZE];              /*!< The buffer of received data. */
    int16_t txBuf_readIdx;
    int16_t txBuf_writeIdx;
    int16_t rxBuf_readIdx;
    int16_t rxBuf_writeIdx;
    uart_rx_callback_t rxFullCallback; /*!< Callback to invoke after receiving byte.*/
}UART_Comm_t;

/*
*********************************************************************************************************
*                                            GLOBAL VARIABLES
*********************************************************************************************************
*/
extern UART_Comm_t  g_Uart0Comm;
extern UART_Comm_t  g_Uart1Comm;
extern UART_Comm_t  g_Uart2Comm;

extern volatile uint32_t adcValue; /*! ADC value */
extern volatile bool conversionCompleted; /*! Conversion is completed Flag */

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/
void LPTMR_InitTriggerSourceOfAdc(LPTMR_Type *base);
void ADC16_PauseConversion(ADC_Type *base);
void ADC16_CalibrateParams(ADC_Type *base);
bool ADC16_InitHardwareTrigger(ADC_Type *base);



//void pflash_test(void);
uint8_t init_eeprom(void);
uint8_t get_setting_from_eeprom(uint8_t *pBuf, uint32_t len);
uint8_t store_setting_to_eeprom(uint32_t *pBuf, uint32_t len);


void BSP_UART0_Init(void);
uint16_t BSP_UART0_SendData(uint8_t *buf, uint16_t size);
uint16_t BSP_UART0_GetTxBufDataSize(void);
uint16_t BSP_UART0_GetTxBufRemainSize(void);
void BSP_UART0_RxBuffFullCallback(void *comm);
uint16_t BSP_UART0_ReceiveData(uint8_t *buf, uint16_t bufSize);

#endif