#ifndef __GENERAL_TIM_H__
#define __GENERAL_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
/********************ͨ�ö�ʱ��TIM�������壬TIM2~TIM5************/

#define GENERAL_TIMx                     TIM5
#define GENERAL_TIM_RCC_CLK_ENABLE()     __HAL_RCC_TIM5_CLK_ENABLE()
#define GENERAL_TIM_RCC_CLK_DISABLE()    __HAL_RCC_TIM5_CLK_DISABLE()
#define GENERAL_TIM_IRQ                  TIM5_IRQn
#define GENERAL_TIM_INT_FUN              TIM5_IRQHandler

#define STEPMOTOR_TIM_CHANNEL_1          TIM_CHANNEL_1
#define STEPMOTOR_TIM_CHANNEL_2          TIM_CHANNEL_2
// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��84MHz/��GENERAL_TIM_PRESCALER+1��
#define GENERAL_TIM_PRESCALER           83  // ʵ��ʱ��Ƶ��Ϊ��1MHz
// ���嶨ʱ�����ڣ�����ʱ����ʼ������BASIC_TIMx_PERIODֵ�Ǹ��¶�ʱ�������ɶ�Ӧ�¼����ж�
#define GENERAL_TIM_PERIOD              0xFFFF 

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx;

/* �������� ------------------------------------------------------------------*/

void GENERAL_TIMx_Init(void);

#endif	/* __GENERAL_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/

