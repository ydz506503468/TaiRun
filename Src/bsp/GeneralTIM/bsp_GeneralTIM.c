/**
  ******************************************************************************
  * �ļ�����: bsp_BasicTIM.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: ������ʱ��TIM6 & TIM7�ײ���������
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
#include "GeneralTIM/bsp_GeneralTIM.h"
#include "RS485/bsp_usartx_RS485.h"
#include "led/bsp_led.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void GENERAL_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sOCConfig;

  htimx.Instance = GENERAL_TIMx;
  htimx.Init.Prescaler = GENERAL_TIM_PRESCALER;
  htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx.Init.Period = GENERAL_TIM_PERIOD;
  htimx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_OnePulse_Init(&htimx,TIM_OPMODE_SINGLE);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htimx, &sClockSourceConfig);

  /* ��ʱ���Ƚ�������� */
  sOCConfig.OCMode = TIM_OCMODE_TOGGLE;                // �Ƚ����ģʽ����ת���
  sOCConfig.Pulse = 0xFFFF;                      // ������
  sOCConfig.OCPolarity = TIM_OCPOLARITY_HIGH;          // �������
  sOCConfig.OCNPolarity = TIM_OCNPOLARITY_LOW;         // ����ͨ���������
  sOCConfig.OCFastMode = TIM_OCFAST_DISABLE;           // ����ģʽ
  sOCConfig.OCIdleState = TIM_OCIDLESTATE_RESET;       // ���е�ƽ
  sOCConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // ����ͨ�����е�ƽ
  HAL_TIM_OC_ConfigChannel(&htimx, &sOCConfig, STEPMOTOR_TIM_CHANNEL_1);
  HAL_TIM_OC_ConfigChannel(&htimx, &sOCConfig, STEPMOTOR_TIM_CHANNEL_2);
  
}

/**
  * ��������: ������ʱ��Ӳ����ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==GENERAL_TIMx)
  {
    /* ������ʱ������ʱ��ʹ�� */
    GENERAL_TIM_RCC_CLK_ENABLE();

    /* �����ж����� */
    HAL_NVIC_SetPriority(GENERAL_TIM_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(GENERAL_TIM_IRQ);
  }
}


// ����һ��ȫ�ֱ�����Ϊ������
volatile uint32_t counter = 0;

// ��ʱ���жϷ������
void GENERAL_TIMx_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htimx, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htimx, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htimx, TIM_IT_UPDATE);
            
            // ÿ���1
            counter++;
            
            // ÿ10��ִ��һ��
            if (counter >= 3)
            {
						LED3_TOGGLE;
                // ���UART_FLAG_ORE��־
                if (__HAL_UART_GET_FLAG(&husartx_rs485  , UART_FLAG_ORE) != RESET)
                {
                    // ���³�ʼ����ʱ��
                    GENERAL_TIMx_Init();
                }
                
                // ���ü�����
                counter = 0;
            }
        }
    }
}




///////////////////////////////////////////////////////////////////////////////////////////////////////


/**
  * ��������: ������ʱ��Ӳ������ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
/*
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

 if(htim_base->Instance==GENERAL_TIMx)
  {
//������ʱ������ʱ�ӽ��� 
    GENERAL_TIM_RCC_CLK_DISABLE();

   //�ر������ж� 
    HAL_NVIC_DisableIRQ(GENERAL_TIM_IRQ);
  }
} 
*/
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
