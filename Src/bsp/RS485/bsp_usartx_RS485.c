/**
  ******************************************************************************
  * �ļ�����: bsp_usartx_RS485.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: ���ش��ڵײ���������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "RS485/bsp_usartx_RS485.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
UART_HandleTypeDef husartx_rs485;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: RS485ͨ�Ź�������GPIO��ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void RS485_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* ��������ʱ��ʹ�� */
  RS485_USART_RCC_CLK_ENABLE();
  RS485_USARTx_GPIO_ClK_ENABLE();
  RS485_REDE_GPIO_ClK_ENABLE();

  /* �������蹦��GPIO���� */
  GPIO_InitStruct.Pin = RS485_USARTx_Tx_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AFx_USARTx;
  HAL_GPIO_Init(RS485_USARTx_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RS485_USARTx_Rx_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AFx_USARTx;
  HAL_GPIO_Init(RS485_USARTx_PORT, &GPIO_InitStruct);
  
  /* SP3485E��������ʹ�ܿ������ų�ʼ�� */
  HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = RS485_REDE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RS485_REDE_PORT, &GPIO_InitStruct);
}

/**
  * ��������: ���ڲ�������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void RS485_USARTx_Init(void)
{ 
  /* RS485ͨ�Ź�������GPIO��ʼ�� */
  RS485_GPIO_Init();
  
  /* Modbus RTUģʽ��ÿ���ֽڵĸ�ʽ:
     8λ����������λ 
     1����ʼλ
     1����żУ��λ
     1��ֹͣλ
     Ĭ��Ҫ��ʹ��żУ��,ʹ����У��ʱҪ��2��ֹͣλ
  */
  husartx_rs485.Instance = RS485_USARTx;
  husartx_rs485.Init.BaudRate = RS485_USARTx_BAUDRATE;
  husartx_rs485.Init.WordLength = UART_WORDLENGTH_8B;
  husartx_rs485.Init.StopBits = UART_STOPBITS_1;
  husartx_rs485.Init.Parity = UART_PARITY_NONE;
  husartx_rs485.Init.Mode = UART_MODE_TX_RX;
  husartx_rs485.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husartx_rs485.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&husartx_rs485);
	
  HAL_NVIC_SetPriority(RS485_USART_IRQn,0,1);
  HAL_NVIC_EnableIRQ(RS485_USART_IRQn);
  
  /* �����ж�,�洢����ʱ������ */ 
  RS485_RX_MODE();
  HAL_UART_Receive_IT(&husartx_rs485,(uint8_t*)&tmp_Rx_Buf,1);	
	
	 
}

/**
  * ��������: ���ڷ��ͺ���
  * �������: Tx_Bul:�����ַ���,TxCount:���͵��ֽ���
  * �� �� ֵ: ��
  * ˵    ��: ���⺯�������װ.
  */
void UART_Tx(uint8_t *Tx_Buf,uint16_t TxCount)
{
  RS485_TX_MODE();
  HAL_UART_Transmit(&husartx_rs485, Tx_Buf, TxCount, 0xffff);
  RS485_RX_MODE();
}



/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
