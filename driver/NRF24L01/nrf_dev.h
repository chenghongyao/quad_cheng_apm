#ifndef _NRF_DEV_H
#define _NRF_DEV_H


// nRF24L01 指令
#define NRF_READ_REG    		0x00  // 读配置寄存器+寄存器地址送出	
#define NRF_WRITE_REG  			0x20  // 写配置寄存器 +寄存器地址送出
	
#define RD_RX_PLOAD 				0x61  //读RX有效数据,1-32字节,从字节0开始
#define WR_TX_PLOAD 				0xA0  //写TX有效数据,1-32字节,从字节0开始

#define FLUSH_TX    				0xE1  // 清除TX FIFO寄存器
#define FLUSH_RX    				0xE2  // 清除RX FIFO寄存器

#define R_RX_PL_WID					0x60					//读取接收数据的宽度(使用动态数据宽度功能),如果该值大于32要清空RX FIFO
#define W_ACK_PAYLOAD				0xA8			//写应答时的数据包,最多可以写三个应答包
#define W_TX_PAYLOAD_NOACK 	0xB0 	//设置不应答(设置后不会自动应答)

#define NRF_NOP         0xFF  //空操作,用来读状态寄存器

#define REUSE_TX_PL 		0xE3  // 重新发送上一包有效数据
#define ACTIVATE				0x50
//===================================================



// nRF24L01 寄存器地址
#define NRF_CONFIG    0x00  // 配置寄存器
#define EN_AA      		0x01  // 使能自动应答功能
#define EN_RXADDR  		0x02  // 接收通道允许(默认通道0与通道1 允许(1))
#define SETUP_AW    	0x03  // 设置地址宽度(3,4,5)
#define SETUP_RETR  	0x04  // 建立自动重发(设置重发延时(),重发次数上限(0-15))
#define RF_CH       	0x05  // 射频通道(设置工作频率(125个))
#define RF_SETUP    	0x06  // '射频寄存器(数据传输速率,发射功率,低噪声放大器增益)

#define STATUS      	0x07  // 状态寄存器(中断源,接受数据通道号,寄存器满标志)
#define OBSERVE_TX  	0x08  // '发送检测寄存器
#define CD          	0x09  // 载波检测

/**接收通道地址**/
#define RX_ADDR_P0  	0x0A  // 数据通道0接受地址(先写低字节,所写字节由SETUP_AW设定)
#define RX_ADDR_P1  	0x0B  // 数据通道1接受地址
#define RX_ADDR_P2  	0x0C  // 数据通道2接受地址
#define RX_ADDR_P3  	0x0D  // 数据通道3接受地址
#define RX_ADDR_P4  	0x0E  // 数据通道4接受地址
#define RX_ADDR_P5  	0x0F  // 数据通道5接受地址
/**发送地址**/
#define TX_ADDR     	0x10  // 发送地址(使用自动应答功能时 接受通道0的地址应与此相等)
/**接收通道有效数据长度**/
#define RX_PW_P0    	0x11  //  接收通道0有效数据长度(0-32字节)
#define RX_PW_P1    	0x12  //  接收通道2有效数据长度(0-32字节)
#define RX_PW_P2    	0x13  //  接收通道3有效数据长度(0-32字节)
#define RX_PW_P3    	0x14  //  接收通道3有效数据长度(0-32字节)
#define RX_PW_P4    	0x15  //  接收通道4有效数据长度(0-32字节)
#define RX_PW_P5    	0x16  //  接收通道5有效数据长度(0-32字节)

#define FIFO_STATUS 	0x17  // FIFI状态寄存器


//以下为NRF2401+ 才有的寄存器
#define DYNPD 				0x1C			//用于设置各通道是否为动态数据宽度 需要先使能FEATURE里的EN_DPL
#define FEATURE				0x1D		//用于使能动态数据宽度,带数据包应答,不应答发送




#endif
