#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"
#include "rc.h"

extern __RC_Data RC_Data;

int main(void)
{
 
 delay_init();
 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

 uart_init(9600);
	
 RC_Init();

 while(1)
 {
	 printf("A:%d,%d\n",RC_Data.RC.ch0,RC_Data.RC.ch1);
	 delay_ms(5);
 }
 
}

