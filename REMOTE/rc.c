#include "rc.h"

static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Ƕ�������жϿ�������ѡ�� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
  /* �������ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  /* �����ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* ʹ���ж� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* ��ʼ������NVIC */
	
  NVIC_Init(&NVIC_InitStructure);
}

void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// �򿪴���GPIO��ʱ��
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
	
	// �򿪴��������ʱ��
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

    // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	// ���ô��ڵĹ�������
	
	// ���ò�����
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	
	// ����У��λ
	USART_InitStructure.USART_Parity =USART_Parity_Even;
	
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
	
	// �����ж����ȼ�����
	NVIC_Configuration();
	
	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);	
	
	// ʹ�ܴ���
	USART_ITConfig(DEBUG_USARTx,USART_IT_RXNE,ENABLE);
	USART_ITConfig(DEBUG_USARTx,USART_IT_IDLE,ENABLE);
	
	USART_Cmd(DEBUG_USARTx, ENABLE);	    
}




__RC_Data	RC_Data;

static void RemoteDataProcess(volatile const uint8_t *sbus_buf, __RC_Data *rc_ctrl);
uint8_t num=0;

/*-----UART4_RX-----PC11----*/

void RC_Init(void)
{
	USART_Config();
}

void UART4_IRQHandler(void){
	
	
	if(USART_GetITStatus(UART4,USART_IT_RXNE)!= RESET){
		
		RC_Data.sbus_rx_buffer[num++] = UART4->DR; 

		
		if(num>Sbus_rx_length-1)
			num=0;
		USART_ClearFlag(UART4,USART_FLAG_RXNE);
		
	}
	else if(USART_GetITStatus(UART4,USART_IT_IDLE)!= RESET)
	{
		UART4->SR;
		UART4->DR;   //�����μĴ��� ���IDLE�жϱ�־λ
		
		RemoteDataProcess(RC_Data.sbus_rx_buffer,&RC_Data);
		
		USART_ClearFlag(UART4,USART_FLAG_IDLE);
		
	}
}

static void RemoteDataProcess(volatile const uint8_t *sbus_buf, __RC_Data *rc_ctrl) 
{    
	 if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
	
	RC_Data.RC.ch0 = ((int16_t)RC_Data.sbus_rx_buffer[0] | ((int16_t)RC_Data.sbus_rx_buffer[1] << 8)) & 0x07FF;     
	RC_Data.RC.ch1 = (((int16_t)RC_Data.sbus_rx_buffer[1] >> 3) | ((int16_t)RC_Data.sbus_rx_buffer[2] << 5)) & 0x07FF;  
	RC_Data.RC.ch2 = (((int16_t)RC_Data.sbus_rx_buffer[2] >> 6) | ((int16_t)RC_Data.sbus_rx_buffer[3] << 2) | ((int16_t)RC_Data.sbus_rx_buffer[4] << 10)) & 0x07FF;  
	RC_Data.RC.ch3 = (((int16_t)RC_Data.sbus_rx_buffer[4] >> 1) | ((int16_t)RC_Data.sbus_rx_buffer[5]<<7)) & 0x07FF;        
	
	RC_Data.RC.s1 = ((RC_Data.sbus_rx_buffer[5] >> 4) & 0x000C) >> 2;   
	RC_Data.RC.s2 = ((RC_Data.sbus_rx_buffer[5] >> 4) & 0x0003);  
	
	RC_Data.Mouse.x = ((int16_t)RC_Data.sbus_rx_buffer[6]) | ((int16_t)RC_Data.sbus_rx_buffer[7] << 8);   
	RC_Data.Mouse.y = ((int16_t)RC_Data.sbus_rx_buffer[8]) | ((int16_t)RC_Data.sbus_rx_buffer[9] << 8);   
	RC_Data.Mouse.z = ((int16_t)RC_Data.sbus_rx_buffer[10]) | ((int16_t)RC_Data.sbus_rx_buffer[11] << 8);     

	RC_Data.Mouse.press_L = RC_Data.sbus_rx_buffer[12];    
	RC_Data.Mouse.press_R = RC_Data.sbus_rx_buffer[13];      
	RC_Data.Keybord.key_num = ((int16_t)RC_Data.sbus_rx_buffer[14])| ((int16_t)RC_Data.sbus_rx_buffer[15] << 8);
	
	RC_Data.RC.slip = ((int16_t)RC_Data.sbus_rx_buffer[16])| ((int16_t)RC_Data.sbus_rx_buffer[17] << 8);       
	
}

//�ж�ң���������Ƿ����
const _Bool RC_SW_DOWN=1;
const uint16_t RC_CHANNAL_ERROR_VALUE=2000;

uint8_t RC_data_is_error(void)
{
    //ʹ����go to��� �������ͳһ����ң�����������ݹ���
    if (RC_Data.RC.ch0 > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_Data.RC.ch1 > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_Data.RC.ch2 > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_Data.RC.ch3 > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_Data.RC.s1 == 0)
    {
        goto error;
    }
    if (RC_Data.RC.s2 == 0)
    {
        goto error;
    }
    return 0;

error:
	
    RC_Data.RC.ch0 = 0;
    RC_Data.RC.ch1 = 0;
    RC_Data.RC.ch2 = 0;
    RC_Data.RC.ch3 = 0;
    RC_Data.RC.s1 = RC_SW_DOWN;
    RC_Data.RC.s2 = RC_SW_DOWN;
    RC_Data.Mouse.x = 0;
    RC_Data.Mouse.y = 0;
    RC_Data.Mouse.z = 0;
    RC_Data.Mouse.press_L = 0;
    RC_Data.Mouse.press_R = 0;
    RC_Data.Keybord.key_num = 0;
	
    return 1;
}

/***************************************�쳣����************************************************/

void Slove_RC_lost(void)
{
    RC_restart();
}
void Slove_data_error(void)
{
    RC_restart();
}

void RC_restart(void)
{
        USART_Cmd(UART4, DISABLE);	
		USART_ClearFlag(UART4,USART_FLAG_IDLE);
		USART_ClearFlag(UART4,USART_FLAG_RXNE);
	
        USART_Cmd(UART4, ENABLE);
}























