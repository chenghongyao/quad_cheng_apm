#include "esp8266.h"
#include "sys.h"
#include "stdarg.h"
#include "string.h"
#include "usart.h"


#define esp8266_putc(_c) H_USART_PUT8_F(3,_c)


static const char resOK[] = "OK";
static const char  resERROR[] = "ERROR";
//static const char  resFAIL[] = "Fail";

static char espbuf[50];			//一行数据
esp_info_t esp_info;
extern queue_t qUart;				//队列,在串口更新


char *itoa(uint16_t num, char buf[])
{
	char t;
	char *p = buf;
	char *b = buf;
	do
	{
		t = num % 10;
		num = num / 10;
		*p++ = t + '0';
	} while (num != 0);
	*p = '\0';
	p--;

	do
	{
		
		t = *p;
		*p-- = *b;
		*b++ = t;
	} while (p > b);
	return buf;
}




void esp8266_puts(char *s)
{
	while(*s!='\0')
	{
		esp8266_putc(*s);
		s++;
	}
}

void esp8266_at(const char *fmt,...)
{
	va_list ap;	
	char datbuf[10];
	esp8266_putc('A');	esp8266_putc('T');	esp8266_putc('+');
	va_start(ap,fmt);
	for(;*fmt;fmt++)
	{
		if(*fmt != '%')
		{
			esp8266_putc(*fmt);
		}
		else	
		{
			switch(*++fmt)
			{
				case 'd':
					esp8266_puts(itoa(va_arg(ap,int),datbuf));
					break;
				case 's':
					esp8266_putc('\"');
					esp8266_puts(va_arg(ap,char*));
					esp8266_putc('\"');
					break;
				case '%':
					esp8266_putc('%');
					break;
				default :
					break;
			}
		}
	}
	va_end(ap);
	esp8266_putc('\r');
	esp8266_putc('\n');
}



//mswait:检测前的等待时间,ms
//timeout:检测超时时间,ms
uint8_t esp8266_check(uint16_t mswait,uint16_t timeout)
{
	uint8_t fres = 0;		//空行标志
	uint32_t tNow;
	delay_ms(mswait);
	tNow = millis();
	while ((millis() - tNow)<timeout)
	{
		while (queue_gets(&qUart, (uint8_t*)espbuf, sizeof(espbuf)))
		{
			//printf("res:%s\r\n", espbuf);
			if (fres)																
			{
				if (!strncmp(resOK, espbuf, 2))
				{
					puts("CHECK:OK");
					return ESP8266_RES_OK;
				}
				else if (!strncmp(resERROR, espbuf, 5))
				{
					puts("CHECK:ERROR");
					return ESP8266_RES_ERROR;
				}
				else if (!strncmp("busy p...", espbuf, 5))
				{
					fres = 0;
				}
				else 
				{
					//puts("CHECK:FAIL");
					return ESP8266_RES_FAIL;
				}
			}
			else
			{
				if (*espbuf == '\0')		//空行
				{
					//puts("res start");
					fres = 1;
				}
				else if(!strncmp("No AP", espbuf, 5))
				{
					esp_info.getip = 0;
				}
			}
		}
	}
	return ESP8266_RES_TIMEOUT;
}

uint8_t esp8266_waitsend(uint16_t mswait,uint16_t timeout)
{
	uint8_t fres = 0;		//空行标志
	uint32_t tNow;
	delay_ms(mswait);
	tNow = millis();
	while ((millis() - tNow)<timeout)
	{
		while (queue_gets(&qUart, (uint8_t*)espbuf, sizeof(espbuf)))
		{
			if (fres)																
			{
				if(*espbuf == '>')
					return ESP8266_RES_OK;
			}
			else
			{
				if (*espbuf == '\0')		//空行
				{
					fres = 1;
				}
			}
		}
	}
	return ESP8266_RES_TIMEOUT;
}



///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

uint8_t esp8266_reset()
{
	esp8266_at("RST");
	return esp8266_check(80,10);
}
void esp8266_test()
{
	esp8266_puts("AT\r\n");
}

void esp8266_ate(uint8_t enable)
{
	if(enable)
		esp8266_puts("ATE1\r\n");
	else 
		esp8266_puts("ATE0\r\n");
}
/******************************************************************************* 
功能：设置串口属性
参数：
stopbits:1/1bit,2/1.5bit,3/2bit
parity:0/none,1/odd,2/even
flow control:0/disable ,2/RTS,3/CTS,4/RTS&CTS
时间：1/17/2017
注意： 
*******************************************************************************/  
uint8_t esp8266_setUART(uint32_t baudrate,uint8_t databits,uint8_t stopbits,uint8_t parity,uint8_t flowcontrol)
{
	esp8266_at("UART_CUR=%d,%d,%d,%d,%d",baudrate,databits,stopbits,parity,flowcontrol);
	return esp8266_check(80,10);
}

///////////////////////////////////////////////////////////////////////////////////////////
/******************************************************************************* 
功能：设置工作模式
参数：
mode:wifi工作模式
	WF_MODE_STA
	WF_MODE_AP
	WF_MODE_STAAP
时间：1/17/2017
注意： 
*******************************************************************************/  
void wifi_setMode(uint8_t mode)
{
	esp8266_at("CWMODE_CUR=%d", mode);
}

/******************************************************************************* 
功能：加入热点
参数：无 
时间：1/17/2017
注意： STA模式下使用
*******************************************************************************/  
void wifi_joinAP(char *ssid,char *pwd)
{
	esp8266_at("CWJAP_CUR=%s,%s",ssid,pwd);
}

uint8_t wifi_askAP()
{
	esp8266_at("CWJAP?");
	return esp8266_check(100, 10);
}
uint8_t wifi_askIP()
{
	esp8266_at("CIFSR");
	return esp8266_check(100, 10);
}
/******************************************************************************* 
功能：退出热点
参数：无 
时间：1/17/2017
注意： STA模式下使用
*******************************************************************************/  
uint8_t wifi_quitAP()
{
	
	esp8266_at("CWQAP");
	return esp8266_check(80,10);
}

/******************************************************************************* 
功能：设置softAP属性
参数：
chl:通道号
ecn:加密方式
WF_ECN_OPEN
WF_ECN_WPA_PSK
WF_ECN_WPA2_PSK
WF_ECN_WPA_WPA2_PSK
时间：1/17/2017
注意： 在AP模式下使用
*******************************************************************************/  
uint8_t wifi_setAP(char *ssid,char *pwd,uint8_t chl,uint8_t ecn)
{
	esp8266_at("CWSAP_CUR=%s,%s,%d,%d",ssid,pwd,chl,ecn);
	return esp8266_check(80,10);
}

/*******************************************************************************
功能：使能DHCP
参数：
enable:
mode:设置的模式
	DHCP_STA
	DHCP_AP
	DHCP_STA_AP
时间：1/17/2017
注意：
*******************************************************************************/
uint8_t wifi_setDHCP(uint8_t mode ,uint8_t enable)
{
	esp8266_at("CWDHCP_CUR=%d,%d", mode,enable);
	return esp8266_check(80,10);
}


/******************************************************************************* 
功能：使能上电自动连接热点
参数：无 
时间：1/17/2017
注意： 
*******************************************************************************/  
uint8_t wifi_autoConnect(uint8_t enable)
{
	esp8266_at("CWAUTOCONN=%d", enable);
	return esp8266_check(80,10);
}

//////////////////////////////////////////////////////////////////////////


/******************************************************************************* 
功能：设置多通道
参数：
时间：1/17/2017
注意： 
tcpServer只支持多链接,需要关闭服务器后设置
*******************************************************************************/  
void ip_setMUX(uint8_t enable)
{
	if(enable)
		esp8266_at("CIPMUX=1\r\n");
	else
		esp8266_at("CIPMUX=0\r\n");
}

/******************************************************************************* 
功能：设置透传
参数：
时间：1/17/2017
注意： 单连接模式下才能设置透传
*******************************************************************************/  
void ip_setMode(uint8_t enable)
{
	if(enable)
		esp8266_at("CIPMODE=1");
	else 
		esp8266_at("CIPMODE=0");
}


/******************************************************************************* 
功能：设置为TCP Client
参数：
ip_remote:远程ip地址
remote_port:远程端口
时间：1/17/2017
注意： 函数为单连接模式
*******************************************************************************/  
void ip_tcpClient(char *remote_ip, uint16_t remote_port)
{
	esp8266_at("CIPSTART=\"TCP\",%s,%d", remote_ip, remote_port);
}

/******************************************************************************* 
功能：关闭ip连接
参数：无 
时间：1/17/2017
注意：包括TcpClient,TcpServier,UDP,单连接模式
*******************************************************************************/  
uint8_t ip_close()
{
	esp8266_at("CIPCLOSE");
	return esp8266_check(80,10);
}


/******************************************************************************* 
功能：设置为TCP Server
参数：
port:监视端口号
enable:是否打开
时间：1/17/2017
注意：单连接模式

*******************************************************************************/  
uint8_t ip_tcpServer(uint16_t port, uint8_t enable)
{
	if (enable)
	{
		esp8266_at("CIPSERVER=1,%d", port);
		return esp8266_check(80,10);
	}
	else
	{
		esp8266_at("CIPSERVER=0");
	}
	return 0;

}


/******************************************************************************* 
功能：非透传模式发送
参数：
dat:
len:数据长度
时间：1/17/2017
注意： 单连接模式
*******************************************************************************/  
uint8_t tcp_send(uint8_t *dat,uint16_t len)
{
	if(len>2048)len=2048;
	esp8266_at("CIPSEND=%d",len);
	delay_ms(100);
	usart3_putbuf(dat,len);
	return 0;
}


/******************************************************************************* 
功能：透传模式下启动发送
参数：无 
时间：1/17/2017
注意： 
*******************************************************************************/  
void tcp_startSend()
{
	esp8266_at("CIPSEND");
}

/******************************************************************************* 
功能：透传模式下停止发送
参数：无 
时间：1/17/2017
注意： 下一包数据必须在1s以后
*******************************************************************************/  
uint8_t tcp_stopSend()
{
	esp8266_puts("+++");
	return 0;
}

uint8_t tcp_setOverTime(uint16_t ns)
{
	if (ns > 7200)ns = 7200;
	esp8266_at("CIPSTO=%d", ns);
	return esp8266_check(80,10);
}


uint8_t wifi_setAPMAC(char *mac)
{
	esp8266_at("CIPAPMAC_CUR=%s",mac);
	return esp8266_check(80,10);
}
uint8_t wifi_setSTAMAC(char *mac)
{
	esp8266_at("CIPSTAMAC_CUR=%s",mac);
	return esp8266_check(80,10);
}
uint8_t wifi_setAPIP(char *ip)
{
	esp8266_at("CIPAP=%s",ip);
	return esp8266_check(80,10);
}
uint8_t wifi_setSTAIP(char *ip)
{
	esp8266_at("CIPSTA=%s",ip);
	return esp8266_check(80,10);
}




void esp8266_init()
{
	esp_info.getip = 0;
	esp_info.getwriter = 0;
	esp_info.hwconnected = 0;
	esp_info.status = ESP_STATUS_NONE;
	esp_info.last_res = ESP8266_RES_NONE;					//上条命令成功
	
//	esp8266_test();		//测试指令
//	if(esp8266_check(80,10) == ESP8266_RES_OK)
//	{
//		puts("test ok");
//		esp_info.hwconnected = 1;
//		
//	}
//	else
//	{
//		puts("test fail");
//	}
}


static void esp8266_dealRespond(char *buf,uint16_t len)
{
	static uint8_t fres = 0;
	//puts(buf);
	if(*buf=='\0')				//等待接受结果
	{
		fres=1;
		return;
	}
	else if(fres==1)								//响应结果
	{
		fres = 0;
		if (!strncmp(resOK,buf,2))
		{
			//puts("CHECK:OK");
			esp_info.last_res = ESP8266_RES_OK;
			return;
		}
		else if (!strncmp(resERROR,buf, 5))
		{
			//puts("CHECK:ERROR");
			esp_info.last_res = ESP8266_RES_ERROR;
			return;
		}
		else if (!strncmp("busy p...", espbuf, 4))
		{
			fres = 0;
		}
		else if (*espbuf=='>')
		{
			
			if(esp_info.getserver == 1)
			{
				esp_info.last_res = ESP8266_RES_OK;
			}
			fres = 0;
		}
		else 
		{
			puts("CHECK:FAIL");
			esp_info.last_res = ESP8266_RES_ERROR;
			return;
		}
	}
	else
	{
		puts(buf);
		return;
	}
}


void esp8266_task_readRespond()
{
	queue_ansyg_gets(&qUart,(uint8_t*)espbuf,sizeof(espbuf),sizeof(espbuf),esp8266_dealRespond);
}

void esp8266_comm()
{
	static uint8_t step=ESP_STATUS_ATE;
	switch(step)
	{
		case ESP_STATUS_ATE:	//关闭回显
			esp8266_ate(0);
			esp_info.last_res = ESP8266_RES_NONE;
			step=ESP_STATUS_ATE_RES;
		break;
		case ESP_STATUS_ATE_RES:
				if(esp_info.last_res == ESP8266_RES_OK)
				{
					puts("ate0 ok");
					step=ESP_STATUS_WIFIMODE;
				}
		break;
		case ESP_STATUS_WIFIMODE://WIFI STA模式
			wifi_setMode(WF_MODE_STA);
			esp_info.last_res = ESP8266_RES_NONE;
			step=ESP_STATUS_WIFIMODE_RES;
			break;
		case ESP_STATUS_WIFIMODE_RES:
				if(esp_info.last_res == ESP8266_RES_OK)
				{
					puts("sta mode ok");
					step=ESP_STATUS_JOINAP;
				}
				break;
		case ESP_STATUS_JOINAP:	//连接热点
			wifi_joinAP("cheng", "a1234567");
			esp_info.last_res = ESP8266_RES_NONE;
			step=ESP_STATUS_JOINAP_RES;
			break;
		case ESP_STATUS_JOINAP_RES:
				if(esp_info.last_res == ESP8266_RES_OK)
				{
					puts("join ap ok");
					esp_info.getip = 1;
					step=ESP_STATUS_MUX;
				}
				else if((esp_info.last_res == ESP8266_RES_FAIL) || (esp_info.last_res == ESP8266_RES_ERROR))
				{
					step = ESP_STATUS_JOINAP;									 			//重新尝试加入
					beepMode = BEEP_SHORT_TWICE;		//加入错误警告
				}
				break;
		case ESP_STATUS_MUX://单通道
			ip_setMUX(0);
			esp_info.last_res = ESP8266_RES_NONE;
			step=ESP_STATUS_MUX_RES;
			break;
		case ESP_STATUS_MUX_RES:
				if(esp_info.last_res == ESP8266_RES_OK)
				{
					puts("mux0 ok");
					step=ESP_STATUS_IPMODE;
				}
				break;
		case ESP_STATUS_IPMODE://非透传
			ip_setMode(0);
			esp_info.last_res = ESP8266_RES_NONE;
			step=ESP_STATUS_IPMODE_RES;
			break;
		case ESP_STATUS_IPMODE_RES:
				if(esp_info.last_res == ESP8266_RES_OK)
				{
					puts("mode1 ok");
					step=ESP_STATUS_TCPSERVER;
				}
				break;
		case ESP_STATUS_TCPSERVER://连接TCPServer
			ip_tcpClient("192.168.137.1",777);
			esp_info.last_res = ESP8266_RES_NONE;
			step=ESP_STATUS_TCPSERVER_RES;
			break;
		case ESP_STATUS_TCPSERVER_RES:
				if(esp_info.last_res == ESP8266_RES_OK)
				{
					puts("tcp client ok");
					esp_info.getserver = 1;
					step=ESP_STATUS_NONE;
				}
				else if((esp_info.last_res == ESP8266_RES_FAIL) || (esp_info.last_res == ESP8266_RES_ERROR))
				{
					step = ESP_STATUS_TCPSERVER;									 			//重新尝试加入
					beepMode = BEEP_SHORT_TWICE;		//加入错误警告
				}
				break;
		default:
			
		break;
	}
}

