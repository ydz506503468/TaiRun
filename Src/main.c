/**
 ******************************************************************************
 * �ļ�����: main.c
 * ��    ��: ӲʯǶ��ʽ�����Ŷ�
 * ��    ��: V1.0
 * ��д����: 2017-5-31
 * ��    ��: ����57&42���������ת����ʵ��
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "StepMotor/bsp_STEPMOTOR.h"
#include "key/bsp_key.h"

#include "usart/bsp_debug_usart.h"
#include "key/bsp_key.h"
#include "led/bsp_led.h"
#include "GeneralTIM/bsp_GeneralTIM.h"
#include "MB-host/bsp_MB_host.h"
#include "RS485/bsp_usartx_RS485.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
__IO uint8_t Motor_En = 0; // ���ʹ��ת��, 0:��ֹ���ת����1��ʹ�ܵ��ת��
/* ˽�к궨�� ----------------------------------------------------------------*/
#define STEPMOTOR_MICRO_STEP 32 // �������������ϸ�֣�������������ʵ�����ö�Ӧ
#define MSG_ERR_FLAG 0xFFFF     // ���մ��� �ַ��䳬ʱ
#define MSG_IDLE 0x0000         // ����״̬
#define MSG_RXING 0x0001        // ���ڽ�������
#define MSG_COM 0x0002          // �������
#define MSG_INC 0x8000          // ����֡������(���ַ���Ŀ��м������1.5���ַ�ʱ��)
#define TIME_OVERRUN 100        // ���峬ʱʱ�� ms
/* ˽�б��� ------------------------------------------------------------------*/
typedef struct
{
  uint8_t DATA_10H[64];

} REG_DATA;

union FloatConverter
{
  uint32_t intValue;
  float floatValue;
};
typedef union
{
  float f;
  uint32_t i;
} FloatUnion;
unsigned int result;
union FloatConverter converter;
__IO uint16_t Rx_MSG = MSG_IDLE; // ���ձ���״̬
__IO uint8_t rx_flag = 0, check_flag = 0;
__IO uint8_t MB_SLAVEADDR = 0x01; // Ĭ�ϴӻ���ַ
REG_DATA reg_value;
// KEY key1,key2,key3,key4,key5;

uint32_t PWM_Data = 10000; // PWMƵ�� 10KHz
uint16_t Duty_Data = 634;  // ռ�ձ�63.4��

uint8_t dir = 0; // 0 ��˳ʱ��   1����ʱ��
uint8_t ena = 0; // 0 ���������� 1��ͣ��
int circle;
int savecir;
int flage = 0;
int startflage = 0;
float data;
// int stop; // ����ȫ�ֱ���
int date_array[16];
int state = 0;
int circlespin;
int sportflage=0;
/* ��չ���� ------------------------------------------------------------------*/
extern __IO uint16_t Toggle_Pulse[4]; /* ��������ٶȿ��ƣ��ɵ��ڷ�ΧΪ 650 -- 3500 ��ֵԽС�ٶ�Խ�� */
/*
 *    ���������������ϸ������Ϊ1ʱ��ÿ200�����岽�������תһ��
 *                          Ϊ32ʱ��ÿ6400�����岽�������תһ��
 *    ����������Ϊ32ʱΪ�����⣺
 *    pulse_count���ڼ�¼�������������pulse_countΪ��������������
 *    ���統pulse_count=12800ʱ��ʵ�����6400���������塣
 *    �������Էǳ����㲽�������ʵ��ת��Ȧ����������Ƕȶ��а취���������
 *    ������������������ϸ������Ϊ����ֵ��pulse_countҲҪ����Ӧ����
 *
 */
__IO uint32_t pulse_count[4]; /*  ���������һ�����������������2 */

/* ˽�к���ԭ�� --------------------------------------------------------------*/
void Check_TimeOut(uint16_t over_time);
void Wait_TimeOut(void);
void read485Data();
void READ485TEST2();
void GENERAL_TIMx_IRQHandler(void);
/* ������ --------------------------------------------------------------------*/
/**
 * ��������: ϵͳʱ������
 * �������: ��
 * �� �� ֵ: ��
 * ˵    ��: ��
 */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE(); // ʹ��PWRʱ��

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1); // ���õ�ѹ�������ѹ����1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                   // ��HSE
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // ��PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;       // PLLʱ��Դѡ��HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                            // 8��ƵMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                          // 336��Ƶ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                // 2��Ƶ���õ�168MHz��ʱ��
  RCC_OscInitStruct.PLL.PLLQ = 7;                            // USB/SDIO/������������ȵ���PLL��Ƶϵ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // ϵͳʱ�ӣ�168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        // AHBʱ�ӣ� 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;         // APB1ʱ�ӣ�42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;         // APB2ʱ�ӣ�84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS(); // ʹ��CSS���ܣ�����ʹ���ⲿ�����ڲ�ʱ��ԴΪ����

  // HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
  // HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
  // HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000); // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
 * ��������: ������.
 * �������: ��
 * �� �� ֵ: ��
 * ˵    ��: ��
 */
int main(void)
{
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();

  /* ����ϵͳʱ�� */
  SystemClock_Config();
  /* ��ʼ��GPIO��Ϊ���� */
  KEY_GPIO_Init();
  /* ��ʼ����ʱ������ */
  STEPMOTOR_TIMx_Init();

  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  MX_DEBUG_USART_Init();
  /* ��ʱ����ʼ�� */
  GENERAL_TIMx_Init();
  /* Modbusʹ��RS485��ʼ�� */
  RS485_USARTx_Init();
  /* LED��ʼ�� */
  LED_GPIO_Init();
  /* ��ʼ����ʱ������ */


  Rx_MSG = MSG_IDLE;// ���ձ���״̬
	check_flag=1;	// ��ȡID--



  /* ȷ����ʱ�� */
  HAL_TIM_Base_Start(&htimx_STEPMOTOR);
  /* ʹ���ж� �رձȽ����*/
  HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR, STEPMOTOR_NO1_TIM_CHANNEL_x);
  HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR, STEPMOTOR_NO2_TIM_CHANNEL_x);
  HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR, STEPMOTOR_NO3_TIM_CHANNEL_x);
  HAL_TIM_OC_Start_IT(&htimx_STEPMOTOR, STEPMOTOR_NO4_TIM_CHANNEL_x);
  TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO2_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO3_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO4_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  while (1)
  {
		   if (KEY2_StateRead() == KEY_DOWN) // ����ѡ��
		{
			
			MB_ReadHoldingReg_QID();
		}
		
		/*
		 if(__HAL_UART_GET_FLAG(&husartx_rs485, UART_FLAG_ORE) != RESET)
  {
    __HAL_UART_CLEAR_FLAG(&husartx_rs485, UART_FLAG_ORE); // ��� ORE ��־λ
    __HAL_UART_FLUSH_DRREGISTER(&husartx_rs485); // ����������ݼĴ���
		
		 HAL_UART_Receive_IT(&husartx_rs485, Rx_Buf, 1);

  }
	*/
		
			
      	if(check_flag==1)
		{
			Rx_MSG = MSG_IDLE;
       //GENERAL_TIMx_IRQHandler();
			
   switch (startflage)
    {
    case 1:
      /* code */
      MB_ReadHoldingReg_03H(0x01, 0x02, 0x02); // ��ȡ�豸ID-��ѹ������1
      HAL_Delay(100);
      break;

    case 2:

      MB_ReadHoldingReg_03H(0x02, 0x02, 0x02); // ��ȡ�豸ID-��ѹ������2
      HAL_Delay(100);
      break;
    case 3:
      MB_ReadHoldingReg_03H(0x03, 0x02, 0x02); // ��ȡ�豸ID-��ѹ������3
      HAL_Delay(100);
      break;

    default:
      // ����Ĭ�����������ѡ��ִ���κβ�������ִ����������
      break;
    }
		rx_flag=1;
			Check_TimeOut(100); //��ʱ���
			READ485TEST2();
		HAL_Delay(100);
    MB_ReadHoldingReg_BATTERY(); // ��ȡ��ص���
      READ485TEST2();
		HAL_Delay(100);
    MB_ReadHoldingReg_DATE(0xb1, date_array[1], date_array[2], date_array[8], date_array[8]);
   	READ485TEST2();
    	HAL_Delay(100);
			if (KEY1_StateRead() == KEY_DOWN) // ����ѡ��
			{
					  /* Modbusʹ��RS485��ʼ�� */
  RS485_USARTx_Init();
			}
	    rx_flag=1;
			Check_TimeOut(50); //��ʱ���
		}	
 
    if (state == 1&&sportflage==0)
    {
      flage = 0;
      STEPMOTOR_DIR1_FORWARD(); // ��ת
      STEPMOTOR_DIR2_FORWARD(); // ��ת
      STEPMOTOR_DIR3_FORWARD(); // ��ת
      STEPMOTOR_DIR4_FORWARD(); // ��ת
      pulse_count[0] = 0;       // ���¼���
      pulse_count[1] = 0;       // ���¼���
      pulse_count[2] = 0;       // ���¼���
      ena = 0;
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO2_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO3_TIM_CHANNEL_x, TIM_CCx_ENABLE);
		  sportflage=1;
    }
    // if (KEY2_StateRead() == KEY_DOWN) // ����ѡ��
    if (state == 2&&sportflage==0)
    {
      flage = 1;
      STEPMOTOR_DIR1_REVERSAL(); // ��ת
      STEPMOTOR_DIR2_REVERSAL(); // ��ת
      STEPMOTOR_DIR3_REVERSAL(); // ��ת
      STEPMOTOR_DIR4_REVERSAL(); // ��ת
      pulse_count[0] = 0;        // ���¼���
      pulse_count[1] = 0;        // ���¼���
      pulse_count[2] = 0;        // ���¼���
      ena = 0;
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO2_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO3_TIM_CHANNEL_x, TIM_CCx_ENABLE);
			 sportflage=1;
    }
    // if (KEY3_StateRead() == KEY_DOWN) // ����ѡ��
    if (state == 3&&sportflage==0)
    {
      flage = 2;
      STEPMOTOR_DIR1_FORWARD(); // ��ת
      STEPMOTOR_DIR2_FORWARD(); // ��ת
      STEPMOTOR_DIR3_FORWARD(); // ��ת
      STEPMOTOR_DIR4_FORWARD(); // ��ת
      pulse_count[0] = 0;       // ���¼���
      pulse_count[1] = 0;       // ���¼���
      pulse_count[2] = 0;       // ���¼���
      ena = 0;
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO2_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO3_TIM_CHANNEL_x, TIM_CCx_ENABLE);
			 sportflage=1;
    }
    // if (KEY4_StateRead() == KEY_DOWN) // ����ѡ��
    if (state == 4&&sportflage==0)
    {
      flage = 3;
      STEPMOTOR_DIR1_REVERSAL(); // ��ת
      STEPMOTOR_DIR2_REVERSAL(); // ��ת
      STEPMOTOR_DIR3_REVERSAL(); // ��ת
      STEPMOTOR_DIR4_REVERSAL(); // ��ת
      pulse_count[0] = 0;        // ���¼���
      pulse_count[1] = 0;        // ���¼���
      pulse_count[2] = 0;        // ���¼���
      ena = 0;
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO2_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO3_TIM_CHANNEL_x, TIM_CCx_ENABLE);
			 sportflage=1;
    }
    state = 0;
    //read485Data(); // ��ȡ����
  
  }

  if (pulse_count[0] >= STEPMOTOR_MICRO_STEP * 200 * 2 * 100000) // ��1��ת��40Ȧ��ͣ��
  {
    // ͣ��
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  }
  if (pulse_count[1] >= STEPMOTOR_MICRO_STEP * 200 * 2 * 30) // ��2��ת��30Ȧ��ͣ��
  {
    // ͣ��
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO2_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  }
  if (pulse_count[2] >= STEPMOTOR_MICRO_STEP * 200 * 2 * 20) // ��3��ת��20Ȧ��ͣ��
  {
    // ͣ��
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO3_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  }
  if (pulse_count[3] >= STEPMOTOR_MICRO_STEP * 200 * 2 * 10) // ��4��ת��10Ȧ��ͣ��
  {
    // ͣ��
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO4_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  }
}

/**
 * ��������: ��ʱ���Ƚ�����жϻص�����
 * �������: htim����ʱ�����ָ��
 * �� �� ֵ: ��
 * ˵    ��: ��
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t count;
  uint32_t tmp;
  count = __HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    tmp = STEPMOTOR_TIM_PERIOD & (count + Toggle_Pulse[0]);
    __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR, STEPMOTOR_NO1_TIM_CHANNEL_x, tmp);
    pulse_count[0]++;
    if (flage == 0 && data > 100) // ��1��ת��circleȦ��ͣ�� 1Ȧ=6400  1Ȧ=200*32 ����ﵽȦ��֮��ֹͣ
    {
      // ͣ��
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_DISABLE);
			 sportflage=0;
    }

    if (flage == 1 && stop == 1)
    {
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_DISABLE);
			stop=0;
			 sportflage=0;
    }
    if (flage == 2 && pulse_count[0] >= STEPMOTOR_MICRO_STEP * 200 * 2 * 1)
    {
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_DISABLE);
			 sportflage=0;
    }
    if (flage == 3 && pulse_count[0] >= STEPMOTOR_MICRO_STEP * 200 * 2 * 1)
    {
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_DISABLE);
			sportflage=0;
    }
  }
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    tmp = STEPMOTOR_TIM_PERIOD & (count + Toggle_Pulse[1]);
    __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR, STEPMOTOR_NO2_TIM_CHANNEL_x, tmp);
    pulse_count[1]++;
  }
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    tmp = STEPMOTOR_TIM_PERIOD & (count + Toggle_Pulse[2]);
    __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR, STEPMOTOR_NO3_TIM_CHANNEL_x, tmp);
    pulse_count[2]++;
  }
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    tmp = STEPMOTOR_TIM_PERIOD & (count + Toggle_Pulse[3]);
    __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR, STEPMOTOR_NO4_TIM_CHANNEL_x, tmp);
    pulse_count[3]++;
  }
}

////////////////////////////////////////////////////////////////////////

/**
 * ��������: ��ַ��ʱ�ȴ�����
 * �������: ��
 * �� �� ֵ: ��
 * ˵    ��: ��������֮��,�ȴ��ӻ���Ӧ,200ms֮������Ϊ��ʱ
 */
void Check_TimeOut(uint16_t over_time)
{
  uint16_t TimeOut = 0; // ͨѶ��ʱ ��λ:ms
  TimeOut = over_time;  //
  while (Rx_MSG != MSG_COM)
  {
    HAL_Delay(1);
    if (TimeOut-- == 0)
    {
      if (Rx_MSG != MSG_COM) // 200ms����û�н������ݣ�����Ϊ��ʱ
      {
        MB_SLAVEADDR++;
        printf("MB_SLAVEADDR: 0x%02x\n", MB_SLAVEADDR);
        if (MB_SLAVEADDR == 0x64)
        {
          MB_SLAVEADDR = 0x00;
        }
      }
      return;
    }
  }
}

/**
 * ��������: ��ʱ�ȴ�����
 * �������: ��
 * �� �� ֵ: ��
 * ˵    ��: ��������֮��,�ȴ��ӻ���Ӧ,200ms֮������Ϊ��ʱ
 */
void Wait_TimeOut(void)
{
  uint16_t TimeOut = 0;   // ͨѶ��ʱ ��λ:ms
  TimeOut = TIME_OVERRUN; // ���峬ʱʱ��Ϊ100ms,��ʵ�ʲ���ʱ��Ϊ200ms
  while (Rx_MSG != MSG_COM)
  {
    HAL_Delay(1);
    if (TimeOut-- == 0)
    {
      if (Rx_MSG != MSG_COM) // 200ms����û�н������ݣ�����Ϊ��ʱ
      {
        printf("����ָ��ʧ�ܣ�������\n");
      }
      return;
    }
  }
}

/**
 * ��������: ���ڽ����жϻص�����
 * �������: ���ھ��
 * �� �� ֵ: ��
 * ˵    ��: ʹ��һ����ʱ���ıȽ��жϺ͸����ж���Ϊ���ճ�ʱ�ж�
 *           ֻҪ���յ����ݾͽ���ʱ����������0,�������Ƚ��жϵ�ʱ��
 *           ˵���Ѿ���ʱ1.5���ַ���ʱ��,�϶�Ϊ֡����,����Ǹ����ж�
 *           ����Ϊ�ǽ������
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &husartx_rs485)
  {
    switch (Rx_MSG)
    {
    /* ���յ���һ���ַ�֮��ʼ��ʱ1.5/3.5���ַ���ʱ�� */
    case MSG_IDLE:
      Rx_MSG = MSG_RXING;
      RxCount = 0;
      HAL_TIM_Base_Start(&htimx);
      break;

    /* ������һ�ν��յ������Ѿ�����1.5���ַ���ʱ����,�϶�Ϊ����֡������ */
    case MSG_ERR_FLAG:
      LED1_TOGGLE;
      Rx_MSG = MSG_INC; // ����֡������
      break;
    }

    /* ʹ�ܼ������� */
    Rx_Buf[RxCount] = tmp_Rx_Buf;
    RxCount++;
    __HAL_TIM_SET_COUNTER(&htimx, 0); // �������ʱ��
    HAL_UART_Receive_IT(&husartx_rs485, (uint8_t *)&tmp_Rx_Buf, 1);
  }
}

/**
 * ��������: ��ʱ���Ƚ��жϻص�����
 * �������: ��ʱ�����
 * �� �� ֵ: ��
 * ˵    ��: ����Ѿ���ʱ1.5���ַ���ʱ��û�н��յ�����
 */
void HAL_TIM_OC_DelayElapsedCallback2(TIM_HandleTypeDef *htim)
{
  /* ����ǵ�һ�η����Ƚ��ж�,���ݶ�Ϊ������ */
  if (Rx_MSG != MSG_INC)
    Rx_MSG = MSG_ERR_FLAG;

  /* ����ǵڶ��ν���Ƚ��ж�,���϶�Ϊ���Ĳ����� */
  else
    Rx_MSG = MSG_INC;
}

/**
 * ��������: ��ʱ�������жϻص�����
 * �������: ��ʱ�����
 * �� �� ֵ: ��
 * ˵    ��: ��ʱ3.5���ַ���ʱ��û�н��յ�����,��Ϊ�ǿ���״̬
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* ����Ѿ�����˽��յ�������������֡,��������Ϊ������������֡ */
  if (Rx_MSG == MSG_INC)
  {
    Rx_MSG = MSG_INC;
  }
  /* �����������ʱ������� */
  else
  {
    Rx_MSG = MSG_COM;
  }
}

void read485Data()
{

  Rx_MSG = MSG_IDLE;
  MB_ReadHoldingReg_03H(0x01, 0x02, 0x02); // ��ȡ�豸ID
  rx_flag = 1;
  printf("��������");
  Check_TimeOut(50); // ��ʱ���
  rx_flag = 1;
  // LED3_TOGGLE;
  //  if (rx_flag == 1)
  //  {
  // LED1_TOGGLE;

  // unsigned char Rx_Buf[] = {0x01, 0x03, 0x04, 0x38, 0x15, 0x42, 0xDE, 0x97, 0xF2};
  date_array[2] = Rx_Buf[5];
  date_array[3] = Rx_Buf[6];
  date_array[4] = Rx_Buf[3];
  date_array[5] = Rx_Buf[4];
  unsigned int result = ((unsigned int)Rx_Buf[5] << 24) | ((unsigned int)Rx_Buf[6] << 16) | ((unsigned int)Rx_Buf[3] << 8) | (unsigned int)Rx_Buf[4];
  if (Rx_Buf[0] == 0xA1)
  {
    if (Rx_Buf[1] == 01)
    {
      state = 1;
    }
    if (Rx_Buf[1] == 02)
    {
      state = 2;
    }
    if (Rx_Buf[1] == 03)
    {
      state = 3;
    }
    if (Rx_Buf[1] == 04)
    {
      state = 4;
    }
  }

  union
  {
    unsigned int intValue;
    float floatValue;
  } converter;

  converter.intValue = result;
  data = converter.floatValue;
  Check_TimeOut(50); // ��ʱ���
  printf("Result: 0x%08X\n", result);
  printf("IEEE 754 ʮ����: %f\n", converter.floatValue);
  printf("\n");
  rx_flag = 0;
  memset(Rx_Buf, 0, sizeof Rx_Buf);
  // }
  // else
  // {
  //   LED2_TOGGLE;
  //   for (uint8_t x = 0; x < 9; x++)
  //   {
  //     printf("Rx_Buf[%d]=%x\n", x, Rx_Buf[x]);
  //   }
  //   printf("Result: 0x%08X\n", result);
  //   printf("IEEE 754 ʮ����: %f\n", converter.floatValue);
  //   memset(Rx_Buf, 0, sizeof Rx_Buf);
  // }

  static uint16_t crc_check = 0;
  crc_check = ((Rx_Buf[RxCount - 1] << 8) | Rx_Buf[RxCount - 2]);

  if (crc_check == MB_CRC16((uint8_t *)&Rx_Buf, RxCount - 2))
  {
    if (Rx_Buf[1] & 0x80)
    {
      while (1)
      {
        LED1_TOGGLE;
      }
    }
  }
  Rx_MSG = MSG_IDLE;
}

void READ485TEST2()
{
  /* ��ȡ�ӵ�ַID����֤�Ƿ�ͨ������ */
 // if (check_flag == 1) // ��ȡID
 // {
  //  Rx_MSG = MSG_IDLE;                               // ������ձ�־
  //  MB_ReadHoldingReg_03H(MB_SLAVEADDR, 1200, 0x01); // ��ȡ�豸ID
  //  rx_flag = 1;                                     // ���ձ�־
  //  Check_TimeOut(100);                              // ��ʱ���
 // }
  Rx_MSG = MSG_COM;
  if (Rx_MSG == MSG_COM)
  {
    if (rx_flag == 1) // ���յ�����--��ȡID--0x04 0x19��0x04 0x20
    {
			if(Rx_Buf[0] == 0x01)
			{
			 unsigned int result = ((unsigned int)Rx_Buf[5] << 24) | ((unsigned int)Rx_Buf[6] << 16) | ((unsigned int)Rx_Buf[3] << 8) | (unsigned int)Rx_Buf[4];

    union
    {
      unsigned int intValue;
      float floatValue;
    } converter;

    converter.intValue = result;
    data = converter.floatValue;
    Check_TimeOut(50); // ��ʱ���
    printf("Result: 0x%08X\n", result);
    printf("IEEE 754 ʮ����: %f\n", converter.floatValue);
    printf("\n");
    rx_flag = 0;
  }

      if (Rx_Buf[0] == 0xA1)
      {
				startflage = 1;
        if (Rx_Buf[1] == 01)
        {
          state = 1;
        }
        if (Rx_Buf[1] == 02)
        {
          state = 2;
        }
        if (Rx_Buf[1] == 03)
        {
          state = 3;
        }
        if (Rx_Buf[1] == 04)
        {
          state = 4;
        }
      }
      printf("\n");
			HAL_UART_Receive_IT(&husartx_rs485, Rx_Buf, 1);
			//memset(Rx_Buf, 0, sizeof Rx_Buf);
      rx_flag = 0; // ������ձ�־
    }
    else // δ��������
    {
      for (uint8_t x = 0; x < 8; x++) // ��ӡ���յ�������
      {
       // LED3_TOGGLE;
        printf("Rx_Buf[%d]=%x\n", x, Rx_Buf[x]);
      }
    }
    static uint16_t crc_check = 0;                                  // CRCУ��
    crc_check = ((Rx_Buf[RxCount - 1] << 8) | Rx_Buf[RxCount - 2]); // CRCУ��
    /* CRC У����ȷ */
    if (crc_check == MB_CRC16((uint8_t *)&Rx_Buf, RxCount - 2)) // CRCУ��
    {
      /* ͨ�Ŵ��� */
      if (Rx_Buf[1] & 0x80)
      {
        while (1)
        {
          LED1_TOGGLE;
          HAL_Delay(500);
        }
      }
    }
    Rx_MSG = MSG_IDLE;
  }
	
	
	
	

}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
