#ifndef _ESP8266_H
#define _ESP8266_H

//#include  <includes.h>

#define SEGMENT_MAX_LEN              100
#define SEGMENT_BUF_SIZE             10

extern  uint8_t atcmd_ate0[];                   // echo or not, 0=no, 1=yes
extern  uint8_t atcmd_cwautoconn[];  // auto connect to wifi ap, 0=no, 1=yes
extern  uint8_t atcmd_cipdinfo[];      // whether response opposite IP and port when "+IPD", 0=no, 1=yes
extern  uint8_t atcmd_cwmode_def[];  // set wifi mode and save to device flash
extern  uint8_t atcmd_cwmode_cur[];  // set current wifi mode, 1=station, 2=softAP, 3=softAP+station
// set softAP IP address(only support C type address): AT+CIPAP_DEF=<ip>[,<gateway>,<netmask>]
extern  uint8_t atcmd_cipap_def[];
// set softAP parameters: AT+CWSAP_DEF=<ssid>,<pwd>,<chl>,<ecn>, <max conn>
extern  uint8_t atcmd_cwsap_def[];

extern  uint8_t atcmd_rst[];             // reset esp8266 module

extern  uint8_t atcmd_cipmode[];   // set transfer mode, 0=general mode, 1=
extern  uint8_t atcmd_cipmux1[];    // set multi link, 1=multiLink, 0=sigleLink
extern  uint8_t atcmd_cipmux0[];
// setup TCP server, only when CIPMUX=1 available. AT+CIPSERVER=<mode>[,<port>], close_server:mode=0, setup_server:mode=1
extern  uint8_t atcmd_cipserver1[];
extern  uint8_t atcmd_cipserver0[];
// send data, AT+CIPSEND=<link ID when CIPMUX=1>,<length>, send data when receive ">", max length=2048
extern  uint8_t atcmd_cipsend[];

extern  uint8_t atcmd_cwlap[];            // list current available AP
extern  uint8_t atcmd_cwjap_def[];    // connect to WIFI(AP), and save to flash

// AT+CIPSTART=<<link ID when CIPMUX=1>,><type>,<remote IP>,<remote port>[,(<UDP local port>),(<UDP mode>)][,(<TCP keep alive>)]
// type: "TCP" or "UDP",
extern  uint8_t atcmd_cipstart[];
extern  uint8_t atcmd_cipclose[];// close tcp/udp transfer

extern  uint8_t atresp_ok[];
extern  uint8_t atresp_sendok[];
extern  uint8_t atresp_ready[];
extern  uint8_t atresp_invalid[];

extern uint16_t BSP_UART0_SendData(uint8_t *buf, uint16_t size);
extern uint8_t GetNextATRespItem(uint8_t **pBuf);
extern uint8_t IsRecvATRespComplete(void);
extern void AT_ClearRespBuffer(void);

void    AT_NormalModeInit(void);
void    AT_SettingModeInit(void);
void    AT_TcpServerClose(void);
void    AT_ModuleReset(void);

void    AT_WaitResp(uint8_t *respStr);
uint8_t AT_WaitMultiRespOR(uint8_t *respStr1, uint8_t *respStr2);
void    AT_WaitDeviceInputReady(void);
void    AT_ConnectServerRetry(uint8_t *pServerIP, uint8_t *pPort, uint16_t retryTime);
uint8_t AT_SearchAPBlock(uint8_t *pSsip, uint32_t retryMs);
uint8_t AT_ConnectAP(uint8_t *pSsip, uint8_t *pPwd);
uint8_t AT_GetTCPIPData(uint8_t mux, uint8_t *tcpData, uint8_t bufLen);
void    AT_SendData(uint8_t mux, uint8_t *data, uint16_t len);

#endif