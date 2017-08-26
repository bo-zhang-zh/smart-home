//#include "includes.h"
#include "stdint.h"
#include "esp8266.h"

#define SEGMENT_MAX_LEN              100
#define SEGMENT_BUF_SIZE             10

uint8_t atcmd_ate0[] = "ATE0\r\n";                   // echo or not, 0=no, 1=yes
uint8_t atcmd_cwautoconn[] = "AT+CWAUTOCONN=0\r\n";  // auto connect to wifi ap, 0=no, 1=yes
uint8_t atcmd_cipdinfo[] = "AT+CIPDINFO=0\r\n";      // whether response opposite IP and port when "+IPD", 0=no, 1=yes
uint8_t atcmd_cwmode_def[] = "AT+CWMODE_DEF=1\r\n";  // set wifi mode and save to device flash
uint8_t atcmd_cwmode_cur[] = "AT+CWMODE_CUR=2\r\n";  // set current wifi mode, 1=station, 2=softAP, 3=softAP+station
// set softAP IP address(only support C type address): AT+CIPAP_DEF=<ip>[,<gateway>,<netmask>]
uint8_t atcmd_cipap_def[] = "AT+CIPAP_DEF=\"192.168.6.1\",\"192.168.6.1\",\"255.255.255.0\"\r\n";
// set softAP parameters: AT+CWSAP_DEF=<ssid>,<pwd>,<chl>,<ecn>, <max conn>
uint8_t atcmd_cwsap_def[] = "AT+CWSAP_DEF=\"smart-home\",\"987654321\",5,3,1\r\n";

uint8_t atcmd_rst[] = "AT+RST\r\n";             // reset esp8266 module

uint8_t atcmd_cipmode[] = "AT+CIPMODE=0\r\n";   // set transfer mode, 0=general mode, 1=
uint8_t atcmd_cipmux1[] = "AT+CIPMUX=1\r\n";    // set multi link, 1=multiLink, 0=sigleLink
uint8_t atcmd_cipmux0[] = "AT+CIPMUX=0\r\n";
// setup TCP server, only when CIPMUX=1 available. AT+CIPSERVER=<mode>[,<port>], close_server:mode=0, setup_server:mode=1
uint8_t atcmd_cipserver1[] = "AT+CIPSERVER=1,5000\r\n";
uint8_t atcmd_cipserver0[] = "AT+CIPSERVER=0,5000\r\n";
// send data, AT+CIPSEND=<link ID when CIPMUX=1>,<length>, send data when receive ">", max length=2048
uint8_t atcmd_cipsend[] = "AT+CIPSEND=";

uint8_t atcmd_cwlap[] = "AT+CWLAP=";            // list current available AP
uint8_t atcmd_cwjap_def[] = "AT+CWJAP_DEF=";    // connect to WIFI(AP), and save to flash

// AT+CIPSTART=<<link ID when CIPMUX=1>,><type>,<remote IP>,<remote port>[,(<UDP local port>),(<UDP mode>)][,(<TCP keep alive>)]
// type: "TCP" or "UDP",
uint8_t atcmd_cipstart[] = "AT+CIPSTART=";
uint8_t atcmd_cipclose[] = "AT+CIPCLOSE=0\r\n";// close tcp/udp transfer

uint8_t atresp_ok[] = "OK";
uint8_t atresp_sendok[] = "SEND OK";
uint8_t atresp_ready[] = "ready";
uint8_t atresp_invalid[] = "invalid";


uint8_t GetNextATRespItem(uint8_t **pBuf);
uint8_t IsRecvATRespComplete(void);
void AT_ClearRespBuffer(void);
uint8_t __MatchString(uint8_t *pInfo, uint8_t infoLen, uint8_t *target);


void AT_NormalModeInit(void)
{
    AT_ClearRespBuffer();
    BSP_UART0_SendData(atcmd_ate0, sizeof(atcmd_ate0));
    AT_WaitResp(atresp_ok);
    BSP_UART0_SendData(atcmd_cwmode_def, sizeof(atcmd_cwmode_def));
    AT_WaitResp(atresp_ok);
    BSP_UART0_SendData(atcmd_cipmux0, sizeof(atcmd_cipmux0));
    AT_WaitResp(atresp_ok);
    BSP_UART0_SendData(atcmd_cipmode, sizeof(atcmd_cipmode));
    AT_WaitResp(atresp_ok);
}

void AT_SettingModeInit(void)
{
    AT_ClearRespBuffer();
    //BSP_UART0_SendData(atcmd_rst, sizeof(atcmd_rst));
    //AT_WaitResp(atresp_ready);
    //AT_WaitMultiRespOR(atresp_ready, atresp_invalid);
    BSP_UART0_SendData(atcmd_ate0, sizeof(atcmd_ate0));
    AT_WaitResp(atresp_ok);
    BSP_UART0_SendData(atcmd_cwautoconn, sizeof(atcmd_cwautoconn));
    AT_WaitResp(atresp_ok);
    BSP_UART0_SendData(atcmd_cipdinfo, sizeof(atcmd_cipdinfo));
    AT_WaitResp(atresp_ok);
    BSP_UART0_SendData(atcmd_cwmode_def, sizeof(atcmd_cwmode_def));
    AT_WaitResp(atresp_ok);
    BSP_UART0_SendData(atcmd_cwmode_cur, sizeof(atcmd_cwmode_cur));
    AT_WaitResp(atresp_ok);
    BSP_UART0_SendData(atcmd_cipap_def, sizeof(atcmd_cipap_def));
    AT_WaitResp(atresp_ok);
    BSP_UART0_SendData(atcmd_cwsap_def, sizeof(atcmd_cwsap_def));
    AT_WaitResp(atresp_ok);

    BSP_UART0_SendData(atcmd_cipmode, sizeof(atcmd_cipmode));
    AT_WaitResp(atresp_ok);
    BSP_UART0_SendData(atcmd_cipmux1, sizeof(atcmd_cipmux1));
    AT_WaitResp(atresp_ok);
    BSP_UART0_SendData(atcmd_cipserver1, sizeof(atcmd_cipserver1));
    AT_WaitResp(atresp_ok);
}


uint8_t __MatchString(uint8_t *pInfo, uint8_t infoLen, uint8_t *target)
{
    uint8_t len;
    uint8_t rst = 0;
    int i;
    len = strlen(target);
    if (infoLen >= len)
    {
        for (i=0; i<len; i++)
        {
            if (pInfo[i] != target[i])
                break;
        }
        if (i >= len)
            rst = 1;// match successfully
    }

    return rst;
}
void AT_WaitResp(uint8_t *respStr)
{
    uint8_t get_ap_state = 0;
    uint8_t *pInfo;
    uint8_t len;
    while (get_ap_state == 0)
    {
        if (IsRecvATRespComplete())
        {
            len = GetNextATRespItem(&pInfo);
            if (__MatchString(pInfo, len, respStr) > 0)
            {
                get_ap_state = 1;
            }
        }
        else
            vTaskDelay(20);
    }

    return;
}
uint8_t AT_WaitMultiRespOR(uint8_t *respStr1, uint8_t *respStr2)
{
    uint8_t get_ap_state = 0;
    uint8_t *pInfo;
    uint8_t len;
    while (get_ap_state == 0)
    {
        if (IsRecvATRespComplete())
        {
            len = GetNextATRespItem(&pInfo);
            if (__MatchString(pInfo, len, respStr1) > 0)
            {
                get_ap_state = 1;
            }
            if (__MatchString(pInfo, len, respStr2) > 0)
            {
                get_ap_state = 1;
            }
        }
        else
            vTaskDelay(20);
    }

    return get_ap_state;
}

uint8_t AT_GetTCPIPData(uint8_t mux, uint8_t *tcpData, uint8_t bufLen)
{
    uint8_t get_ap_state = 0;
    uint8_t *pInfo;
    uint8_t len;
    uint8_t dataLen = 0;
    uint8_t commaCnt = 0;
    int i,j;
    while (get_ap_state == 0)
    {
        if (IsRecvATRespComplete())
        {
            len = GetNextATRespItem(&pInfo);
            if (__MatchString(pInfo, len, "+IPD") > 0)// get tcpip data, the format is:+IPD,0,len:xxxxx
            {
                for (i=0; i<len; i++)
                {
                    if (pInfo[i] == ',')
                    {
                        commaCnt++;
                        if ((mux == 0) || (commaCnt >= 2))
                        {
                            dataLen = atoi(&(pInfo[i+1]));// get the data length
                            break;
                        }
                    }
                }
                for (; i<len; i++)
                    if (pInfo[i] == ':')
                        break;
                for (i=i+1,j=0; (i<len)&&(j<bufLen)&&(j<dataLen); i++,j++)// copy data
                {
                    tcpData[j] = pInfo[i];
                }
                while ((j<dataLen) && (j<bufLen))// there is some data in the next segment
                {
                    len = GetNextATRespItem(&pInfo);
                    for (i=0; (i<len)&&(j<bufLen)&&(j<dataLen); i++,j++)// copy data
                    {
                        tcpData[j] = pInfo[i];
                    }
                }
                get_ap_state = 1;
            }
        }
        else
            vTaskDelay(20);
    }

    return dataLen;
}
void uart_itoa(int value, uint8_t *str, uint8_t radix)// assume the string buffer is large enough
{
    uint8_t temp[10];
    uint8_t reminder;
    int j;
    int i = 0;
    do{
        reminder = value%radix;
        temp[i++] = reminder + (reminder <= 9 ? '0' : 'A');
        value = value / radix;
    }while(value);
    for (j=0,i--; i>=0; i--,j++)
    {
        str[j] = temp[i];
    }
    str[j] = '\0';
    return;
}
void AT_WaitDeviceInputReady(void)
{
    uint8_t get_ap_state = 0;
    uint8_t *pInfo;
    uint8_t len;
    while (get_ap_state == 0)
    {
        if (IsRecvATRespComplete())
        {
            len = GetNextATRespItem(&pInfo);
            if (__MatchString(pInfo, len, ">") > 0)
            {
                get_ap_state = 1;
            }
        }
        else
            vTaskDelay(20);
    }

    return;
}
void AT_SendData(uint8_t mux, uint8_t *data, uint16_t len)
{
    uint8_t num_str[10];
    int idx = 0;
    uint16_t remain = BSP_UART0_GetTxBufRemainSize();

    while (remain < len)
    {
        BSP_UART0_SendData(atcmd_cipsend, sizeof(atcmd_cipsend)-1);
        if (mux == 1) //multi-link
            BSP_UART0_SendData("0,", 2);
        uart_itoa(remain, num_str, 10);
        BSP_UART0_SendData(num_str, strlen(num_str));
        BSP_UART0_SendData("\r\n", 2);
        AT_WaitDeviceInputReady();
        BSP_UART0_SendData(&(data[idx]), remain);// send the data
        AT_WaitResp(atresp_sendok);
        idx += remain;
        len -= remain;
        remain = BSP_UART0_GetTxBufRemainSize();
    }
    BSP_UART0_SendData(atcmd_cipsend, sizeof(atcmd_cipsend)-1);
    if (mux == 1) //multi-link
        BSP_UART0_SendData("0,", 2);
    uart_itoa(len, num_str, 10);
    BSP_UART0_SendData(num_str, strlen(num_str));
    BSP_UART0_SendData("\r\n", 2);
    AT_WaitDeviceInputReady();
    BSP_UART0_SendData(&(data[idx]), len);// send the data
    AT_WaitResp(atresp_sendok);

    return;
}

void AT_ConnectServerRetry(uint8_t *pServerIP, uint8_t *pPort, uint16_t retryTime)
{
    uint8_t state;
    uint8_t len;
    AT_ClearRespBuffer();
    while (1)
    {
        BSP_UART0_SendData(atcmd_cipstart, sizeof(atcmd_cipstart)-1);
        BSP_UART0_SendData("\"TCP\",\"", 7);
        len = strlen(pServerIP);
        BSP_UART0_SendData(pServerIP, len);
        BSP_UART0_SendData("\",", 2);
        len = strlen(pPort);
        BSP_UART0_SendData(pPort, len);
        BSP_UART0_SendData("\r\n", 2);
        vTaskDelay(1000);
        state = AT_WaitMultiRespOR("ERROR", "OK");
        if (state == 2)// get OK response, jump out the while (1)
            break;
        else
            vTaskDelay(retryTime);// wait 5s and then try again
    }
}

uint8_t AT_SearchAPBlock(uint8_t *pSsip, uint32_t retryMs)
{
    int8_t state = 0;
    uint16_t len = 0;
    uint8_t *pResp;
    AT_ClearRespBuffer();
    while (1)
    {
        BSP_UART0_SendData(atcmd_cwlap, sizeof(atcmd_cwlap)-1);
        BSP_UART0_SendData("\"", 1);
        len = strlen(pSsip);
        BSP_UART0_SendData(pSsip, len);
        BSP_UART0_SendData("\"\r\n", 3);
        vTaskDelay(2000);
        while ((state == 0) || (state == 1))
        {
            if (IsRecvATRespComplete())
            {
                len = GetNextATRespItem(&pResp);
                if (__MatchString(pResp, len, "+CWLAP") > 0)
                {
                    state = 1;
                }
                else if (__MatchString(pResp, len, "OK") > 0)
                {
                    if (state == 1)
                        state = 2;
                    else
                        state = 3;
                }
            }
            else
                vTaskDelay(200);
        }
        if (state == 2)// jump out the while (1)
            break;
        else if (retryMs > 0)
            vTaskDelay(retryMs);// wait 5s and then try again
    }
    return state;
}

uint8_t AT_ConnectAP(uint8_t *pSsip, uint8_t *pPwd)
{
    int8_t state = 0;
    uint16_t len;
    uint8_t *pResp;

    AT_ClearRespBuffer();
    BSP_UART0_SendData(atcmd_cwjap_def, sizeof(atcmd_cwjap_def)-1);
    BSP_UART0_SendData("\"", 1);
    len = strlen(pSsip);
    BSP_UART0_SendData(pSsip, len);
    BSP_UART0_SendData("\",\"", 3);
    len = strlen(pPwd);
    BSP_UART0_SendData(pPwd, len);
    BSP_UART0_SendData("\"\r\n", 3);
    vTaskDelay(2000);
    while (state == 0)
    {
        if (IsRecvATRespComplete())
        {
            len = GetNextATRespItem(&pResp);
            if (__MatchString(pResp, len, "FAIL") > 0)
            {
                state = -1;
            }
            if (__MatchString(pResp, len, "OK") > 0)
            {
                state = 1;
            }
        }
        else
            vTaskDelay(200);
    }
    return state;
}

void AT_TcpServerClose(void)
{
    BSP_UART0_SendData(atcmd_cipclose, sizeof(atcmd_cipclose));// close the TCP/UDP transfer
    AT_WaitResp(atresp_ok);
}

void AT_ModuleReset(void)
{
    BSP_UART0_SendData(atcmd_rst, sizeof(atcmd_rst));
    AT_WaitMultiRespOR(atresp_ready, atresp_invalid);
}

void AT_TcpDisconnect(void)
{
    BSP_UART0_SendData(atcmd_cipserver0, sizeof(atcmd_cipserver0));// close the TCP/UDP transfer
    AT_WaitResp(atresp_ok);
}
