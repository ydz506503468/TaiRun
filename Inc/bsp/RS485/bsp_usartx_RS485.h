#ifndef __BSP_USARTX_RS485_H__
#define __BSP_USARTX_RS485_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
#define RS485_USARTx                                 USART3
#define RS485_USARTx_BAUDRATE                        115200  // modbus Ĭ��ͨ�Ų�������19.2Kbit/s 
#define RS485_USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART3_CLK_ENABLE()
#define RS485_USART_RCC_CLK_DISABLE()                __HAL_RCC_USART3_CLK_DISABLE()

#define RS485_USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define RS485_USARTx_PORT                            GPIOB
#define RS485_USARTx_Tx_PIN                          GPIO_PIN_10
#define RS485_USARTx_Rx_PIN                          GPIO_PIN_11
#define GPIO_AFx_USARTx                              GPIO_AF7_USART3

/* ʹ��485ͨ�ŵ�ʱ��Ż��õ�ʹ��IO */
#define RS485_REDE_GPIO_ClK_ENABLE()                 __HAL_RCC_GPIOH_CLK_ENABLE()
#define RS485_REDE_PORT                              GPIOH
#define RS485_REDE_PIN                               GPIO_PIN_8
#define RS485_RX_MODE()                              HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_RESET)
#define RS485_TX_MODE()                              HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_SET)

#define RS485_USART_IRQn                             USART3_IRQn
#define RS485_USART_IRQHANDLER                       USART3_IRQHandler

/* RTUͨ����Ҫȷ����ʱʱ�� */
#if DEBUG_USARTx_BAUDRATE <= 19200
  /* 1.5���ַ��ĳ�ʱʱ�� T = BAUDRATE/11/1000*/
  #define OVERTIME_15CHAR             (((float)DEBUG_USARTx_BAUDRATE/11)*1.5f)
  /* 3���ַ��ĳ�ʱʱ�� */
  #define OVERTIME_35CHAR             (((float)DEBUG_USARTx_BAUDRATE/11)*3.5f)
#else 
  /* �����ʳ���19200bit/s������½���ĳ�ʱʱ�� */
  #define OVERTIME_15CHAR                750.0f    // 750us 
  #define OVERTIME_35CHAR               1750.0f  // 1.75ms  
#endif
	
	
/* ��չ���� ------------------------------------------------------------------*/
extern UART_HandleTypeDef husartx_rs485;

extern __IO uint8_t Rx_Buf[256];    // ���ջ���,���256�ֽ�
extern __IO uint8_t Tx_Buf[256];    // ���ջ���,���256�ֽ�
extern __IO uint8_t tmp_Rx_Buf;     // ���ջ���
extern  __IO uint16_t RxCount;      // �����ַ�����
  
/* �������� ------------------------------------------------------------------*/
void RS485_USARTx_Init(void);
void UART_Tx(uint8_t *Tx_Buf,uint16_t TxCount);


#endif  /* __BSP_USARTX_RS485_H__ */

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
