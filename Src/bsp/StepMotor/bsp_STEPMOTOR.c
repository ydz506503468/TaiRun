/**
  ******************************************************************************
  * �ļ�����: bsp_STEPMOTOR.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.1
  * ��д����: 2017-06-01
  * ��    ��: ���Ჽ���������������ʵ��
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "StepMotor/bsp_STEPMOTOR.h" 
#include "GeneralTIM/bsp_GeneralTIM.h"
#include "led/bsp_led.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_STEPMOTOR;
__IO uint16_t Toggle_Pulse[4] = {800,1200,1600,2000};         // �Ƚ�������ڣ�ֵԽС���Ƶ��Խ��

/* ��չ���� ------------------------------------------------------------------*/
int stop=0; // ����ȫ�ֱ���
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ���������GPIO��ʼ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void STEPMOTOR_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct; 
  
  /* ���Ŷ˿�ʱ��ʹ�� */
  STEPMOTOR_TIM_PUL1_GPIO_CLK_ENABLE();
  STEPMOTOR_DIR1_GPIO_CLK_ENABLE();
  STEPMOTOR_ENA1_GPIO_CLK_ENABLE();
  
  STEPMOTOR_TIM_PUL2_GPIO_CLK_ENABLE();
  STEPMOTOR_DIR2_GPIO_CLK_ENABLE();
  STEPMOTOR_ENA2_GPIO_CLK_ENABLE();
  
  STEPMOTOR_TIM_PUL3_GPIO_CLK_ENABLE();
  STEPMOTOR_DIR3_GPIO_CLK_ENABLE();
  STEPMOTOR_ENA3_GPIO_CLK_ENABLE();
  
  STEPMOTOR_TIM_PUL4_GPIO_CLK_ENABLE();
  STEPMOTOR_DIR4_GPIO_CLK_ENABLE();
  STEPMOTOR_ENA4_GPIO_CLK_ENABLE();
  
  /* ��1��*/
  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_TIM_PUL1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AFx_TIMx;        // GPIO��������TIM���ù���
  HAL_GPIO_Init(STEPMOTOR_TIM_PUL1_PORT, &GPIO_InitStruct);
  
  /* �����������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_DIR1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_DIR1_PORT, &GPIO_InitStruct);
  
  /* �������ѻ�ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_ENA1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_ENA1_PORT, &GPIO_InitStruct);
  
  /* ��2��*/
  /* �����������������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_TIM_PUL2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AFx_TIMx;        // GPIO��������TIM���ù���
  HAL_GPIO_Init(STEPMOTOR_TIM_PUL2_PORT, &GPIO_InitStruct);
  
  /* ��������������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_DIR2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_DIR2_PORT, &GPIO_InitStruct);
  
  /* ���������ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_ENA2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_ENA2_PORT, &GPIO_InitStruct);
  
  /* ��3��*/
  /* �����������������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_TIM_PUL3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AFx_TIMx;        // GPIO��������TIM���ù���
  HAL_GPIO_Init(STEPMOTOR_TIM_PUL3_PORT, &GPIO_InitStruct);
  
  /* ��������������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_DIR3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_DIR3_PORT, &GPIO_InitStruct);
  
  /* ���������ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_ENA3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_ENA3_PORT, &GPIO_InitStruct);
  
  /* ��4��*/
  /* �����������������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_TIM_PUL4_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AFx_TIMx;        // GPIO��������TIM���ù���
  HAL_GPIO_Init(STEPMOTOR_TIM_PUL4_PORT, &GPIO_InitStruct);
  
  /* ��������������������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_DIR4_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_DIR4_PORT, &GPIO_InitStruct);
  
  /* ���������ʹ�ܿ�������IO��ʼ�� */
  GPIO_InitStruct.Pin = STEPMOTOR_ENA4_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(STEPMOTOR_ENA4_PORT, &GPIO_InitStruct);
  
  STEPMOTOR_DIR1_FORWARD();
  STEPMOTOR_OUTPUT1_ENABLE();
  
  STEPMOTOR_DIR2_FORWARD();
  STEPMOTOR_OUTPUT2_ENABLE();
  
  STEPMOTOR_DIR3_FORWARD();
  STEPMOTOR_OUTPUT3_ENABLE();
  
  STEPMOTOR_DIR4_FORWARD();
  STEPMOTOR_OUTPUT4_ENABLE();
	
	
	
	/* �����������������*/
  GPIO_InitStruct.Pin = LIMPOS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(LIMPOS_PORT, &GPIO_InitStruct);
	
	/* �����������������*/
  GPIO_InitStruct.Pin = LIMNEG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
  HAL_GPIO_Init(LIMNEG_PORT, &GPIO_InitStruct);
	/* �������ԭ��������*/
  GPIO_InitStruct.Pin = ORIGIN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;       // GPIO��������ϵͳĬ�Ϲ���
	HAL_GPIO_Init(ORIGIN_PORT, &GPIO_InitStruct);
  
	HAL_NVIC_SetPriority(LIMPOS_EXTI_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(LIMPOS_EXTI_IRQn);
  
  HAL_NVIC_SetPriority(LIMNEG_EXTI_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(LIMNEG_EXTI_IRQn);
  
  HAL_NVIC_SetPriority(ORIGIN_EXTI_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(ORIGIN_EXTI_IRQn);
}

/**
  * ��������: ��������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void STEPMOTOR_TIMx_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;             // ��ʱ��ʱ��
  TIM_MasterConfigTypeDef sMasterConfig;                 // ��ʱ����ģʽ����
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;   // ɲ��������ʱ������
  TIM_OC_InitTypeDef sConfigOC;                          // ��ʱ��ͨ���Ƚ����
  
  /* ��ʱ�������������� */
  htimx_STEPMOTOR.Instance = STEPMOTOR_TIMx;                                 // ��ʱ�����
  htimx_STEPMOTOR.Init.Prescaler = STEPMOTOR_TIM_PRESCALER;                  // ��ʱ��Ԥ��Ƶ��
  htimx_STEPMOTOR.Init.CounterMode = TIM_COUNTERMODE_UP;                     // �����������ϼ���
  htimx_STEPMOTOR.Init.Period = STEPMOTOR_TIM_PERIOD;                        // ��ʱ������
  htimx_STEPMOTOR.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;                 // ʱ�ӷ�Ƶ
  htimx_STEPMOTOR.Init.RepetitionCounter = STEPMOTOR_TIM_REPETITIONCOUNTER;  // �ظ�������
  HAL_TIM_Base_Init(&htimx_STEPMOTOR);

  /* ��ʱ��ʱ��Դ���� */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       // ʹ���ڲ�ʱ��Դ
  HAL_TIM_ConfigClockSource(&htimx_STEPMOTOR, &sClockSourceConfig);

  /* ��ʼ����ʱ���Ƚ�������� */
  HAL_TIM_OC_Init(&htimx_STEPMOTOR);
  
  /* ��ʱ�������ģʽ */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htimx_STEPMOTOR, &sMasterConfig);
  
  /* ɲ��������ʱ������ */
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htimx_STEPMOTOR, &sBreakDeadTimeConfig);

  /* ��ʱ���Ƚ�������� */
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;                // �Ƚ����ģʽ����ת���
  sConfigOC.Pulse = Toggle_Pulse[0];                   // ������
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;          // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;         // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;       // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;     // ����ͨ�����е�ƽ
  HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, STEPMOTOR_NO1_TIM_CHANNEL_x);

  sConfigOC.Pulse = Toggle_Pulse[1];                   // ������
  HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, STEPMOTOR_NO2_TIM_CHANNEL_x);
  
  sConfigOC.Pulse = Toggle_Pulse[2];                   // ������
  HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, STEPMOTOR_NO3_TIM_CHANNEL_x);
  
  sConfigOC.Pulse = Toggle_Pulse[3];                   // ������
  HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, STEPMOTOR_NO4_TIM_CHANNEL_x);
  
  /* STEPMOTOR���GPIO��ʼ������ */
  STEPMOTOR_GPIO_Init();
  
  /* ���ö�ʱ���ж����ȼ���ʹ�� */
  HAL_NVIC_SetPriority(STEPMOTOR_TIMx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(STEPMOTOR_TIMx_IRQn);

}

/**
  * ��������: ������ʱ��Ӳ����ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==STEPMOTOR_TIMx)
  {
    /* ������ʱ������ʱ��ʹ�� */
    STEPMOTOR_TIM_RCC_CLK_ENABLE();
  }
}

/**
  * ��������: ������ʱ��Ӳ������ʼ������
  * �������: htim_base��������ʱ���������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
 if(htim_base->Instance==GENERAL_TIMx)
  {
    /* ������ʱ������ʱ�ӽ��� */
    GENERAL_TIM_RCC_CLK_DISABLE();

    /* �ر������ж� */
    HAL_NVIC_DisableIRQ(GENERAL_TIM_IRQ);
  }
  if(htim_base->Instance==STEPMOTOR_TIMx)
  {
    /* ������ʱ������ʱ�ӽ��� */
    STEPMOTOR_TIM_RCC_CLK_DISABLE();
    
    HAL_GPIO_DeInit(STEPMOTOR_TIM_PUL1_PORT,STEPMOTOR_TIM_PUL1_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_DIR1_PORT,STEPMOTOR_DIR1_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_ENA1_PORT,STEPMOTOR_ENA1_PIN);
    
    HAL_GPIO_DeInit(STEPMOTOR_TIM_PUL2_PORT,STEPMOTOR_TIM_PUL2_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_DIR2_PORT,STEPMOTOR_DIR2_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_ENA2_PORT,STEPMOTOR_ENA2_PIN);
    
    HAL_GPIO_DeInit(STEPMOTOR_TIM_PUL3_PORT,STEPMOTOR_TIM_PUL3_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_DIR3_PORT,STEPMOTOR_DIR3_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_ENA3_PORT,STEPMOTOR_ENA3_PIN);
    
    HAL_GPIO_DeInit(STEPMOTOR_TIM_PUL4_PORT,STEPMOTOR_TIM_PUL4_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_DIR4_PORT,STEPMOTOR_DIR4_PIN);
    HAL_GPIO_DeInit(STEPMOTOR_ENA4_PORT,STEPMOTOR_ENA4_PIN);
    
    HAL_NVIC_DisableIRQ(STEPMOTOR_TIMx_IRQn);
  }
} 





/**
  * ��������: �ⲿ�жϷ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʵ���ж��Ƿ񵽴Ｋ�޺ͼ��ԭ���ź�
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == LIMPOS_PIN)	//������λ�ļ�������
	{
		__HAL_GPIO_EXTI_CLEAR_IT(LIMPOS_PIN);
		if(HAL_GPIO_ReadPin(LIMPOS_PORT,LIMPOS_PIN) == LIM_POS_LEVEL)
		{	
			LED2_ON;
				stop=1;
		}
		
	}
}
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
