#include "sys.h"
#if 1
#pragma import(__use_no_semihosting)                             
struct __FILE 
{ 
	int handle; 

}; 
FILE __stdout;     
  
//??_sys_exit()??????????    
_sys_exit(int x) 
{ 
	x = x; 
} 

//???fputc?? 
int fputc(int ch, FILE *f)
{      
	H_USART_PUT8_F(1,ch);
	return 0;
}
#endif 
