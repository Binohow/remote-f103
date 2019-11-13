#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"
#include "rc.h"

int main(void)
{
 
 delay_init();
 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
 uart_init(9600);
 RC_Init();

 while(1)
 {
	 printf("wawaw\n");
	 delay_ms(100);
 }
}

