#ifndef _ESP8266_H
#define _ESP8266_H

#include "sys.h"


/*
1.开始操作wifi之前要设置工作模式(setMode)为STA或AP
(1).STA模式用joinAP连接WIFI,用quitAP退出WIFI
(2).AP模式用wifi_setAP设置ap参数
2.模块可TCP或UDP通信,通过CIPSTART命令的type参数实现,程序进行了封装,以下只讨论TCP模式
(1).TCP可作为Client或Server
(2).Client
	调用startClient,输入服务器IP,和端口号
	closeClient关闭该连接
	发送数据
		只有单链接模式下可以透传(setMUX(0)),调用setMode(1)可设置为透传模式


			
(3).Server
		必须先调用setMUX(1)设置为多路模式,多路模式不能透传(setMode(0))
		tcp_setServer(1,port),输入端口号,启动Server监听,当有TCPClient接入时按顺序占用一个连接,最大5个,id=0-5
		用closeClient()输入id号关闭对应Client,
		通过CIFSR获取本地IP
		发送数据:
			使用send发送,输入id号和数据大小			
			接收数据:如果Client发来数据,形式为+IPD,id,长度:data


		setMode(0);
		setMUX(1);
		setServer(1,port)
		
		
	
(1).TCP Server要用多连接方式(setMUX(1)),因为他要连接多个Client,此时不能为透传模式
(2).TCP Client要用单连接(setMUX(0)),只与Server建立连接
*/


//setMode用,wifi工作模式
#define WF_MODE_STA			1
#define WF_MODE_AP			2
#define WF_MODE_STA_AP		3
//setAP用,加密方式
#define WF_ECN_OPEN			0
#define WF_ECN_WPA_PSK		2
#define WF_ECN_WPA2_PSK 	3
#define WF_ECN_WPA_WPA2_PSK	4
//DHCP设置模式				
#define DHCP_AP				0
#define DHCP_STA			1
#define DHCP_STA_AP			2
//应答类型
#define ESP8266_RES_OK			0
#define ESP8266_RES_ERROR		1
#define ESP8266_RES_FAIL		2
#define ESP8266_RES_TIMEOUT	3
#define ESP8266_RES_NONE		 4



typedef enum
{
	ESP_STATUS_NONE = 0,
	ESP_STATUS_ATE,
	ESP_STATUS_ATE_RES,
	ESP_STATUS_WIFIMODE,
	ESP_STATUS_WIFIMODE_RES,
	ESP_STATUS_JOINAP,
	ESP_STATUS_JOINAP_RES,
	ESP_STATUS_MUX,
	ESP_STATUS_MUX_RES,
	ESP_STATUS_IPMODE,
	ESP_STATUS_IPMODE_RES,	
	ESP_STATUS_TCPSERVER,
	ESP_STATUS_TCPSERVER_RES,
}esp_status_t;

typedef struct
{
	uint8_t hwconnected:1;	//硬件连接
	uint8_t getip:1;				//获取到ip(STA模式下连接到AP)
	uint8_t getserver:1;		//链接到Server(TCP Client模式
	uint8_t getwriter:1;		//链接到Server(TCP Client模式
	uint16_t timeout;				//超时时间(ms)
	uint8_t last_res;
	esp_status_t status;

}esp_info_t;



extern esp_info_t esp_info;

void esp8266_init(void);
//基础命令
void esp8266_test(void);
uint8_t esp8266_reset(void);
void esp8266_ate(uint8_t enable);		//设置回显
uint8_t esp8266_setUART(uint32_t baudrate,uint8_t databits,uint8_t stopbits,uint8_t parity,uint8_t flowcontrol);


//wifi参数设置指令
void wifi_setMode(uint8_t mode);//设置STA或AP模式
void wifi_joinAP(char *ssid,char *pwd);
uint8_t wifi_quitAP(void);
uint8_t wifi_setAP(char *ssid,char *pwd,uint8_t chl,uint8_t ecn);
uint8_t wifi_setDHCP(uint8_t mode, uint8_t enable);
uint8_t wifi_autoConnect(uint8_t enable);


uint8_t wifi_askAP(void);
uint8_t wifi_askIP(void);

//TCP/IP工具
void ip_setMUX(uint8_t enable);				
void ip_setMode(uint8_t enable);						
void ip_tcpClient(char *addr, uint16_t port);
uint8_t ip_close(void);							


uint8_t ip_tcpServer(uint16_t port, uint8_t enable);//启用tcp server
uint8_t tcp_closeServer(void);
uint8_t tcp_setOverTime(uint16_t ns);					//设置tcp server等待超时时间 0-7200
uint8_t tcp_send(uint8_t *dat,uint16_t len);
void tcp_startSend(void);
uint8_t tcp_stopSend(void);
void esp8266_task_readRespond(void);
void esp8266_comm(void);

#endif


