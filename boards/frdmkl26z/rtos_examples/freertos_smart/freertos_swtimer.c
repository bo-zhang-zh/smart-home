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

/* Standard includes. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_lpsci.h"
#include "fsl_sim.h"

#include "board.h"
#include "app.h"
#include "bsp.h"
#include "esp8266.h"

#include "arm_math.h"
#include "arm_const_structs.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifdef PRINTF
#undef PRINTF
#define PRINTF
//#define PRINTF(...) do{                                                 \
//    sprt_rst = sprintf(commbuf, __VA_ARGS__);                           \
//    xfer.data = commbuf;                                                \
//    xfer.dataSize = strlen(commbuf);                                    \
//    while (txOnGoing);                                                  \
//    txOnGoing = true;                                                   \
//    LPSCI_TransferSendNonBlocking(DEMO_LPSCI, &g_lpsciHandle, &xfer);   \
//    /*while (txOnGoing);*/                                                  \
//}while(1)
#endif

/* Task priorities. */
#define parse_task_PRIORITY           (configMAX_PRIORITIES - 1) // the uart data must be parsed as soon as possible
#define adc_task_PRIORITY             (configMAX_PRIORITIES - 2)
#define setting_task_PRIORITY         (configMAX_PRIORITIES - 3)
#define uart_task_PRIORITY            (configMAX_PRIORITIES - 3)
#define gpio_task_PRIORITY            (configMAX_PRIORITIES - 4)

#define configUSER_STACK_SIZE         ((unsigned short)150)
/* The software timer period. */
#define SW_TIMER_PERIOD_MS (1000 / portTICK_PERIOD_MS)

#define RX_RING_BUFFER_SIZE 50U
#define TX_BUFFER_SIZE      250U
#define RX_BUFFER_SIZE      250U

#define CLIENT_REQUEST_MAX_LEN                   650

#define WORK_STATE_NORMAL                        0x01
#define WORK_STATE_SETTING                       0x02
#define WORK_STATE_CONNECTING                    0x03
#define WORK_STATE_ILLEGAL                       0x04

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint8_t g_workState = WORK_STATE_CONNECTING;

#pragma pack(1)
EE_Setting_t settingData = {
    .heading_code = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF},
    .device_id = {0x00},
    .ssid = {"soul_2.4"},
    .pwd = {"11111111"},
    .serverIP = {"192.168.1.68"},
    .serverPort = {"8234"},
    .count_time = 60,
    .send_interval = 1,
};
EE_Setting_t tmpFlashData;
#pragma pack()
sim_uid_t uid;

uint8_t *ssid_str;
uint8_t *pwd_str;
uint8_t *serverIP_str;
uint8_t *serverPort_str;


TaskHandle_t settingTaskHandle = NULL;
TaskHandle_t uartTaskHandle = NULL;
TaskHandle_t adcTaskHandle = NULL;
TaskHandle_t monitorTaskHandle = NULL;
SemaphoreHandle_t xSemSendTrigger = NULL;

SleepDataPacket_t sleep_data;
SleepTime_t       currentTime;
SleepTime_t       sleepTime;
uint8_t reqData[CLIENT_REQUEST_MAX_LEN];


#define TEST_LENGTH_SAMPLES          512
static q15_t testInput_q15[TEST_LENGTH_SAMPLES];
static q15_t testOutput_q15[TEST_LENGTH_SAMPLES];
uint32_t fftSize = TEST_LENGTH_SAMPLES;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* The callback function. */
static void SwTimerCallback(TimerHandle_t xTimer);

void error_trap(void);
//void pflash_test(void);

static void COMM_HardwareInit(void);
static void uart_task(void *pvParameters);
static void adc_task(void *pvParameters);
static void gpio_task(void *pvParameters);
static void setting_task(void *pvParameters);
static void monitor_task(void *pvParameters);
static void parse_task(void *pvParameters);// parse the uart received data
/* LPSCI user callback */
void LPSCI_UserCallback(UART0_Type *base, lpsci_handle_t *handle, status_t status, void *userData);

void uart_itoa(int value, uint8_t *str, uint8_t radix);// assume the string buffer is large enough

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    TimerHandle_t SwTimerHandle = NULL;

    /* Init board hardware. */
    BOARD_InitHardware();
#ifndef DISABLE_ALL_UART0_APP
    COMM_HardwareInit();
#endif
    /* Create the software timer. */
    SwTimerHandle = xTimerCreate("SwTimer",          /* Text name. */
                                 SW_TIMER_PERIOD_MS, /* Timer period. */
                                 pdTRUE,             /* Enable auto reload. */
                                 0,                  /* ID is not used. */
                                 SwTimerCallback);   /* The callback function. */
    /* Start timer. */
    xTimerStart(SwTimerHandle, 0);
    xSemSendTrigger = xSemaphoreCreateCounting( 1, 0 );
    if( xSemSendTrigger != NULL )
    {
        // The semaphore was created successfully.
        // The semaphore can now be used.
    }
#ifdef DISABLE_ALL_UART0_APP
    while(1);
#endif
    //pflash_test();
    SIM_GetUniqueId(&uid);// get unique id from MCU
    init_eeprom();//
#ifndef CODE_PROTECTION_INIT
    if (get_setting_from_eeprom((uint8_t *)&tmpFlashData, sizeof(tmpFlashData)) == 0)
#endif
    {
        settingData.device_id[0] = (uid.MH >> 24) & 0xFF;
        settingData.device_id[1] = (uid.MH >> 16) & 0xFF;
        settingData.device_id[2] = (uid.MH >> 8) & 0xFF;
        settingData.device_id[3] = (uid.MH) & 0xFF;
        settingData.device_id[4] = (uid.ML >> 24) & 0xFF;
        settingData.device_id[5] = (uid.ML >> 16) & 0xFF;
        settingData.device_id[6] = (uid.ML >> 8) & 0xFF;
        settingData.device_id[7] = (uid.ML) & 0xFF;
        settingData.device_id[8] = (uid.L >> 24) & 0xFF;
        settingData.device_id[9] = (uid.L >> 16) & 0xFF;
        settingData.device_id[10] = (uid.L >> 8) & 0xFF;
        settingData.device_id[11] = (uid.L) & 0xFF;

#ifdef CODE_PROTECTION_INIT
        for (int i=0; i<12; i++)
            settingData.protect_code[i] = settingData.device_id[i] ^ 'B';
        for (int i=0; i<12; i=i+2)
            settingData.protect_code[i] = settingData.protect_code[i] ^ 'o';
#endif
        store_setting_to_eeprom((uint32_t *)&settingData, sizeof(settingData));
    }
#ifndef CODE_PROTECTION_INIT
    else
    {
        for (int i=0; i<sizeof(settingData); i++)
        {
            ((uint8_t *)&settingData)[i] = ((uint8_t *)&tmpFlashData)[i];
        }
    }
#endif

    xTaskCreate(uart_task, "Uart_task", configUSER_STACK_SIZE, NULL, uart_task_PRIORITY, &uartTaskHandle);
    xTaskCreate(adc_task, "Adc_task", configUSER_STACK_SIZE, NULL, adc_task_PRIORITY, &adcTaskHandle);
    xTaskCreate(gpio_task, "Gpio_task", configMINIMAL_STACK_SIZE, NULL, gpio_task_PRIORITY, NULL);

    xTaskCreate(setting_task, "setting_task", configUSER_STACK_SIZE, NULL, setting_task_PRIORITY, &settingTaskHandle);
    xTaskCreate(parse_task, "parse_task", configUSER_STACK_SIZE, NULL, parse_task_PRIORITY, NULL);
    xTaskCreate(monitor_task, "monitor_task", configUSER_STACK_SIZE, NULL, gpio_task_PRIORITY, &monitorTaskHandle);
    /* Start scheduling. */
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Software timer callback.
 */
static void SwTimerCallback(TimerHandle_t xTimer)
{
    PRINTF("Tick.\r\n");
}


/*
* @brief Gets called when an error occurs.
*
* @details Print error message and trap forever.
*/
void error_trap(void)
{
    PRINTF("\r\n\r\n\r\n\t---- HALTED DUE TO FLASH ERROR! ----");
    while (1)
    {
    }
}

static void COMM_HardwareInit(void)
{
    BSP_UART0_Init();
}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
static void uart_task(void *pvParameters)
{
    int i, j;
    uint8_t len;
    int8_t get_ap_state = 0;
    uint8_t pcode[12];

    ssid_str = settingData.ssid;
    pwd_str = settingData.pwd;
    serverIP_str = settingData.serverIP;
    serverPort_str = settingData.serverPort;

    for (i=0; i<12; i++)
        pcode[i] = settingData.device_id[i] ^ 'B';
    for (i=0; i<12; i=i+2)
        pcode[i] = pcode[i] ^ 'o';
    for (i=0; i<12; i++)
        if (pcode[i] != settingData.protect_code[i])
            break;
    if (i < 12) // verify fail
    {
        g_workState = WORK_STATE_ILLEGAL;
        vTaskSuspend(NULL);// hold here
    }

    vTaskDelay(2000);
    if (GPIO_ReadPinInput(SW_GPIO, SW_GPIO_PIN))// normal mode
    {
    }
    else// enter wifi settingg mode
    {
        vTaskResume(settingTaskHandle);
        vTaskSuspend(0);
    }

    g_workState = WORK_STATE_CONNECTING;

    AT_NormalModeInit();
    AT_SearchAPBlock(ssid_str, 5000); // retry until find the AP(ssip)

    get_ap_state = AT_ConnectAP(ssid_str, pwd_str);
    if (get_ap_state < 0)// connect AP failed
    {
        vTaskSuspend(NULL);
    }

    AT_ConnectServerRetry(serverIP_str, serverPort_str, 5000);// connect the TCP server, try again after 5s if failed.

    reqData[0] = 0x03;
    for (i=0; i<12; i++)
        reqData[i+1] = settingData.device_id[i];
    AT_SendData(0, reqData, i+1);

    len = AT_GetTCPIPData(0, reqData, CLIENT_REQUEST_MAX_LEN);
    if (len > 0)
    {
        switch (reqData[0])
        {
        case 0x30:
            currentTime.year = reqData[1];
            currentTime.month = reqData[2];
            currentTime.day = reqData[3];
            currentTime.hour = reqData[4];
            currentTime.minite = reqData[5];
            currentTime.second = reqData[6];
            break;
        default:
            break;
        }
    }
    vTaskResume(adcTaskHandle);
    g_workState = WORK_STATE_NORMAL;
    for (i=0; i<sizeof(currentTime); i++)
        ((uint8_t *)&sleepTime)[i] = ((uint8_t *)&currentTime)[i];

#ifndef AT_SERVER_CONNECTING_ALWAYS_ON
    AT_TcpDisconnect();
    //BSP_UART0_SendData(atcmd_cipserver0, sizeof(atcmd_cipserver0));// close the TCPIP connection
    //AT_WaitResp(atresp_ok);
#endif

    while (1)
    {
        if( xSemaphoreTake( xSemSendTrigger, ( TickType_t )portMAX_DELAY ) == pdTRUE )// wait forever
        {
#ifndef AT_SERVER_CONNECTING_ALWAYS_ON
            AT_ConnectServerRetry(3000);// connect the TCP server, try again after 3s if failed.
#endif
            reqData[0] = 0x04; // special command for send sleep data
            reqData[3] = sleepTime.year;
            reqData[4] = sleepTime.month;
            reqData[5] = sleepTime.day;
            reqData[6] = sleepTime.hour;
            reqData[7] = sleepTime.minite;
            reqData[8] = sleepTime.second;
            reqData[9] = (sleep_data.num >> 8) & 0xFF;
            reqData[10] = sleep_data.num & 0xFF;
            for (i=11,j=0; (i<CLIENT_REQUEST_MAX_LEN) && (j<sleep_data.num);)
            {
                reqData[i++] = sleep_data.nodeBuf[j].heart_rate;
                reqData[i++] = sleep_data.nodeBuf[j].breath_rate;
                reqData[i++] = sleep_data.nodeBuf[j].turning_flag;
                j++;
            }
            reqData[1] = (i >> 8) & 0xFF;//MSB first
            reqData[2] = i & 0xFF;
            sleep_data.num = 0;
            AT_SendData(0, reqData, i);

            for (i=0; i<sizeof(currentTime); i++)
                ((uint8_t *)&sleepTime)[i] = ((uint8_t *)&currentTime)[i];
#ifndef AT_SERVER_CONNECTING_ALWAYS_ON
            AT_TcpDisconnect();
            //BSP_UART0_SendData(atcmd_cipserver0, sizeof(atcmd_cipserver0));// close the TCPIP connection
            //AT_WaitResp(atresp_ok);
#endif
        }
        else
        {
            // We could not obtain the semaphore and can therefore not access
            // the shared resource safely.
        }

        //LED_GREEN_TOGGLE();
        //vTaskDelay(1000);
    }
}

uint16_t adc_data;
uint32_t tmp_time = 0;

extern uint8_t heartbeat_cnt;
extern uint8_t breath_cnt;
extern uint8_t turning_cnt;
/*!
 * @brief main function
 */
void adc_task(void *pvParameters)
{
    arm_rfft_instance_q15 S;
//    uint16_t result;
    uint8_t  rst;
    uint16_t fft_data_idx = 0;
//    uint8_t  tmp = 0xFF;
//    uint8_t buf[5];
//    uint8_t idx = 0;
    static uint32_t time_idx = 0;
    uint8_t tick_cnt = 0;

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = ADC_TRIGGER_PERIOD;
    xLastWakeTime = xTaskGetTickCount ();

    /* Set to allow entering vlps mode */
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeVlp);

    /* Calibrate param Temperature sensor */
    ADC16_CalibrateParams(DEMO_ADC16_BASEADDR);

    /* Initialize Demo ADC */
    if (!ADC16_InitHardwareTrigger(DEMO_ADC16_BASEADDR))
    {
        vTaskSuspend(NULL);
    }
    /* setup the HW trigger source */
    LPTMR_InitTriggerSourceOfAdc(DEMO_LPTMR_BASE);
#if defined(FSL_FEATURE_ADC16_MAX_RESOLUTION) && (FSL_FEATURE_ADC16_MAX_RESOLUTION >= 16U)
    ADC16_EnableDMA(DEMO_ADC16_BASEADDR, false);
#endif
    NVIC_EnableIRQ(DEMO_ADC16_IRQ_ID);

    sleep_data.num = 0;
    vTaskSuspend(0); // suspend here, wait for resume


 	arm_rfft_init_q15(&S, fftSize, ifftFlag, doBitReverse);

    while (1)
    {
        /* Prevents the use of wrong values */
        //while (!conversionCompleted)
        {
        }

        time_idx++;

        portENTER_CRITICAL();
        rst = (uint8_t)(adcValue >> 8);
        portEXIT_CRITICAL();
        adc_data = rst;
        sleep_time_analysis(rst, time_idx);

        testInput_q15[fft_data_idx++] = rst;
        if (fft_data_idx >= TEST_LENGTH_SAMPLES)
        {
            fft_data_idx = 0;
            arm_rfft_q15(&S, testInput_q15, testOutput_q15);
            arm_cmplx_mag_q15(testOutput_q15, testInput_q15, fftSize);
        }

        tmp_time++;
        if (tmp_time >= settingData.count_time * configTICK_RATE_HZ/ADC_TRIGGER_PERIOD)// reach the time to count the data
        {
            tmp_time = 0;
            if (sleep_data.num < SLEEP_DATA_NODE_SIZE)
            {
                sleep_data.nodeBuf[sleep_data.num].heart_rate = heartbeat_cnt;
                sleep_data.nodeBuf[sleep_data.num].breath_rate = breath_cnt;
                sleep_data.nodeBuf[sleep_data.num].turning_flag = turning_cnt;
                sleep_data.num++;
                heartbeat_cnt = 0;
                breath_cnt = 0;
                turning_cnt = 0;
                if (sleep_data.num >= settingData.send_interval)
                {
                    xSemaphoreGive( xSemSendTrigger );
                }
            }
        }

        //vTaskDelay(20);;        /* Suspend the task execution for 20 milliseconds.          */
        vTaskDelayUntil( &xLastWakeTime, xFrequency );

        /* Clear conversionCompleted flag */
        conversionCompleted = false;

        //LED_GREEN_TOGGLE();
        /* Enter to Very Low Power Stop Mode */
        //SMC_SetPowerModeVlps(SMC);
        //vTaskSuspend(NULL);

        tick_cnt++;
        if (tick_cnt >= configTICK_RATE_HZ/xFrequency)
        {
            tick_cnt = 0;
            currentTime.second++;
            if (currentTime.second >= 60)
            {
                currentTime.second = 0;
                currentTime.minite++;
                if (currentTime.minite >= 60)
                {
                    currentTime.minite = 0;
                    currentTime.hour++;
                    if (currentTime.hour >= 24)
                    {
                        currentTime.hour = 0;
                        currentTime.day++;
                        if ((currentTime.day > 28) && (currentTime.month == 2))
                        {
                            currentTime.day = 1;
                            currentTime.month++;
                        }
                        else if ((currentTime.day > 30) && ((currentTime.month == 4) || (currentTime.month == 6)
                                 || (currentTime.month == 9) || (currentTime.month == 11)))
                        {
                            currentTime.day = 1;
                            currentTime.month++;
                        }
                        else if (currentTime.day > 31)
                        {
                            currentTime.day = 1;
                            currentTime.month++;
                        }

                        if (currentTime.month > 12)
                        {
                            currentTime.month = 1;
                            currentTime.year++;
                        }
                    }
                }
            }
        }
    }
}

static void gpio_task(void *pvParameters)
{
    uint32_t interval;

    while (1)
    {
        switch (g_workState)
        {
        case WORK_STATE_NORMAL:
            interval = 1000;//ms
            break;
        case WORK_STATE_SETTING:
            interval = 250;//ms
            break;
        case WORK_STATE_CONNECTING:
            interval = 100;//ms
            break;
        case WORK_STATE_ILLEGAL:
            interval = 50;//ms
            break;
        default:
            break;
        }
        LED_GREEN_TOGGLE();
        vTaskDelay(interval);
    }

    vTaskSuspend(NULL);
}

static void setting_task(void *pvParameters)
{

    int i,j;
    uint8_t dataLen;

    vTaskSuspend(0); // suspend here, wait for resume
    g_workState = WORK_STATE_SETTING;

    AT_SettingModeInit();

    while (1)
    {
        dataLen = AT_GetTCPIPData(1, reqData, CLIENT_REQUEST_MAX_LEN);// block here until get request data
        if (dataLen > 0)//(AT_GetTCPIPData(reqData, CLIENT_REQUEST_MAX_LEN))
        {
            switch (reqData[0])
            {
            case 0x01:
                reqData[0] = 0x10;
                i = 1;
                j = 0;
                do{
                    reqData[i++] = ssid_str[j];
                }while ((i<CLIENT_REQUEST_MAX_LEN) && (j<sizeof(settingData.ssid)) && (ssid_str[j++] != '\0'));
                j = 0;
                do{
                    reqData[i++] = pwd_str[j];
                }while ((i<CLIENT_REQUEST_MAX_LEN) && (j<sizeof(settingData.pwd)) && (pwd_str[j++] != '\0'));
                j = 0;
                do{
                    reqData[i++] = serverIP_str[j];
                }while ((i<CLIENT_REQUEST_MAX_LEN) && (j<sizeof(settingData.serverIP)) && (serverIP_str[j++] != '\0'));
                j = 0;
                do{
                    reqData[i++] = serverPort_str[j];
                }while ((i<CLIENT_REQUEST_MAX_LEN) && (j<sizeof(settingData.serverPort)) && (serverPort_str[j++] != '\0'));
                reqData[i++] = (settingData.send_interval >> 24) & 0xFF;
                reqData[i++] = (settingData.send_interval >> 16) & 0xFF;
                reqData[i++] = (settingData.send_interval >> 8) & 0xFF;
                reqData[i++] = settingData.send_interval & 0xFF;
                AT_SendData(1, reqData, i);
                break;
            case 0x02:
                i = 1;
                j = 0;
                do{
                    tmpFlashData.ssid[j++] = reqData[i];
                }while ((i<dataLen) && (j<sizeof(settingData.ssid)) && (reqData[i++] != '\0'));
                j = 0;
                do{
                    tmpFlashData.pwd[j++] = reqData[i];
                }while ((i<dataLen) && (j<sizeof(settingData.pwd)) && (reqData[i++] != '\0'));
                j = 0;
                do{
                    tmpFlashData.serverIP[j++] = reqData[i];
                }while ((i<dataLen) && (j<sizeof(settingData.serverIP)) && (reqData[i++] != '\0'));
                j = 0;
                do{
                    tmpFlashData.serverPort[j++] = reqData[i];
                }while ((i<dataLen) && (j<sizeof(settingData.serverPort)) && (reqData[i++] != '\0'));
                tmpFlashData.send_interval = (reqData[i] << 24) | (reqData[i+1] << 16) | (reqData[i+2] << 8) | (reqData[i+3]);
                for (int i=0; i<sizeof(settingData); i++)
                {
                    if (((uint8_t *)&settingData)[i] != ((uint8_t *)&tmpFlashData)[i])
                        break;
                }
                if (i<sizeof(settingData))
                {
                    for (int i=0; i<sizeof(settingData); i++)
                    {
                        ((uint8_t *)&settingData)[i] = ((uint8_t *)&tmpFlashData)[i];
                    }
                    portDISABLE_INTERRUPTS();
                    store_setting_to_eeprom((uint32_t *)&settingData, sizeof(settingData));
                    portENABLE_INTERRUPTS();
                }

                reqData[0] = 0x20;
                AT_SendData(1, reqData, 1);
                AT_TcpServerClose();
                AT_ModuleReset();
                //BSP_UART0_SendData(atcmd_cipclose, sizeof(atcmd_cipclose));
                //AT_WaitResp(atresp_ok);
                //BSP_UART0_SendData(atcmd_rst, sizeof(atcmd_rst));
                //AT_WaitMultiRespOR(atresp_ready, atresp_invalid);

                vTaskResume(uartTaskHandle);
                vTaskSuspend(0);
                break;
            default:
                break;
            }
        }
        LED_GREEN_TOGGLE();
        vTaskDelay(100);
    }

    vTaskSuspend(NULL);
}

static void monitor_task(void *pvParameters)// monitor the TCP connection state
{
    vTaskSuspend(NULL);
    while (1)
    {
        //LED_GREEN_TOGGLE();
        vTaskDelay(20);
    }
    vTaskSuspend(NULL);
}

#define SEGMENT_MAX_LEN              100   // the maximum value is 127
#define SEGMENT_BUF_SIZE             10

static uint8_t tempRxBuffer[20];
static uint8_t tempRxSize = 0;

uint8_t segBuf[SEGMENT_BUF_SIZE][SEGMENT_MAX_LEN];
uint8_t segBufWIdx = 0;
uint8_t segBufRIdx = 0;
uint8_t segBufUsedSize = 0;
uint8_t segBufDataIdx = 1;// the first byte is reserved for data length

static void parse_task(void *pvParameters)// parse the uart received data
{
    int i;
    while (1)
    {
        if (BSP_UART0_GetRxBufDataSize() >= 2)
        {
            tempRxSize = BSP_UART0_ReceiveData(tempRxBuffer, 20);
            for (i=0; i<tempRxSize-1; i++)
            {
                segBuf[segBufWIdx][segBufDataIdx++] = tempRxBuffer[i];
                if ((segBufDataIdx >= SEGMENT_MAX_LEN-1) || ((tempRxBuffer[i] == '\r') && (tempRxBuffer[i+1] == '\n')))// the segment buffer is full or meet "\r\n"
                {
                    if (segBufDataIdx > SEGMENT_MAX_LEN-1)// the segment buffer is full
                    {
                        segBuf[segBufWIdx][0] = segBufDataIdx-1;//data size in this segment buffer, the first byte is data size
                    }
                    else// segBufDataIdx == SEGMENT_MAX_LEN-1  or this segment contain string "\r\n"
                    {
                        segBuf[segBufWIdx][segBufDataIdx++] = tempRxBuffer[i+1];
                        if ((tempRxBuffer[i] == '\r') && (tempRxBuffer[i+1] == '\n'))// this segment contain string "\r\n"
                            segBuf[segBufWIdx][0] = 0x80 | (segBufDataIdx-1);//data size in this segment buffer, the first byte is data size
                        else//
                            segBuf[segBufWIdx][0] = segBufDataIdx-1;//data size in this segment buffer, the first byte is data size
                        i++;
                    }
                    segBufDataIdx = 1;

                    if (segBufUsedSize >= SEGMENT_BUF_SIZE)// buffer array is use up
                    {//discard the oldest data
                        if (segBufRIdx + 1 >= SEGMENT_BUF_SIZE)
                            segBufRIdx = 0;
                        else
                            segBufRIdx++;
                        segBufUsedSize--;
                    }
                    if (segBufWIdx + 1 >= SEGMENT_BUF_SIZE)
                        segBufWIdx = 0;
                    else
                        segBufWIdx++;
                    segBuf[segBufWIdx][0] = 0;
                    segBufUsedSize++;
                }
            }
            if (i == tempRxSize-1)
                segBuf[segBufWIdx][segBufDataIdx++] = tempRxBuffer[i];
        }
        //LED_GREEN_TOGGLE();
        vTaskDelay(20);
    }
}

uint8_t GetNextATRespItem(uint8_t **pBuf)
{
    uint8_t len;
    static uint8_t unchange_cnt = 0;
    static uint8_t preDataIdx = 0;
    if (segBufUsedSize == 0)// the buffer is empty
    {
        if ((preDataIdx == segBufDataIdx) && (segBufDataIdx > 1))// received data has unchanged for some time, regard it as a segment
        {
            unchange_cnt++;
            if (unchange_cnt > 1)
            {
                *pBuf = &(segBuf[segBufWIdx][1]);// the first byte used as segment length
                len = segBufDataIdx - 1;
                segBuf[segBufWIdx][0] = len;
                segBufDataIdx = 1;

                if (segBufWIdx + 1 >= SEGMENT_BUF_SIZE)
                    segBufWIdx = 0;
                else
                    segBufWIdx++;
                segBuf[segBufWIdx][0] = 0;
                if (segBufRIdx + 1 >= SEGMENT_BUF_SIZE)
                    segBufRIdx = 0;
                else
                    segBufRIdx++;
                return len;
            }
        }
        preDataIdx = segBufDataIdx;
        return 0;
    }
    else
    {
        preDataIdx = 0;
        unchange_cnt = 0;
    }

    *pBuf = &(segBuf[segBufRIdx][1]);// the first byte used as segment length
    len = segBuf[segBufRIdx][0];

    if (segBufRIdx + 1 >= SEGMENT_BUF_SIZE)
        segBufRIdx = 0;
    else
        segBufRIdx++;
    segBufUsedSize--;

    return len;// return the length of the AT response information
}

uint8_t IsRecvATRespComplete(void)
{
    uint8_t idx;
    uint8_t result = 0;
    static uint8_t unchange_cnt = 0;
    static uint8_t preDataIdx = 0;
    if (segBufUsedSize > 0)
    {
        if (segBufWIdx == 0)
            idx = SEGMENT_BUF_SIZE-1;
        else
            idx = segBufWIdx - 1;
        if ((segBuf[segBufWIdx][0] == 0) && (segBuf[idx][0] & 0x80))//get response, current write index is not completed and the previous item is ok
            result = 1;
    }
    else
    {
        if ((preDataIdx == segBufDataIdx) && (segBufDataIdx > 1))// the latest received data unchanged for some time
        {
            unchange_cnt++;
            if (unchange_cnt > 0)
            {
                result = 1;
            }
        }
        preDataIdx = segBufDataIdx;
    }
    return result;
}

void AT_ClearRespBuffer(void)
{
    while (segBufUsedSize > 0)
    {
        if (segBufRIdx + 1 >= SEGMENT_BUF_SIZE)
            segBufRIdx = 0;
        else
            segBufRIdx++;
        segBufUsedSize--;
    }
}
