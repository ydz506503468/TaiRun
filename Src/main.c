/**
 ******************************************************************************
 * 文件名程: main.c
 * 作    者: 硬石嵌入式开发团队
 * 版    本: V1.0
 * 编写日期: 2017-5-31
 * 功    能: 四轴57&42步进电机旋转控制实现
 ******************************************************************************
 * 说明：
 * 本例程配套硬石stm32开发板YS-F4Pro使用。
 *
 * 淘宝：
 * 论坛：http://www.ing10bbs.com
 * 版权归硬石嵌入式开发团队所有，请勿商用。
 ******************************************************************************
 */
/* 包含头文件 ----------------------------------------------------------------*/
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
/* 私有类型定义 --------------------------------------------------------------*/
__IO uint8_t Motor_En = 0; // 电机使能转动, 0:禁止电机转动，1：使能电机转动
/* 私有宏定义 ----------------------------------------------------------------*/
#define STEPMOTOR_MICRO_STEP 32 // 步进电机驱动器细分，必须与驱动器实际设置对应
#define MSG_ERR_FLAG 0xFFFF     // 接收错误 字符间超时
#define MSG_IDLE 0x0000         // 空闲状态
#define MSG_RXING 0x0001        // 正在接收数据
#define MSG_COM 0x0002          // 接收完成
#define MSG_INC 0x8000          // 数据帧不完整(两字符间的空闲间隔大于1.5个字符时间)
#define TIME_OVERRUN 100        // 定义超时时间 ms
/* 私有变量 ------------------------------------------------------------------*/
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
__IO uint16_t Rx_MSG = MSG_IDLE; // 接收报文状态
__IO uint8_t rx_flag = 0, check_flag = 0;
__IO uint8_t MB_SLAVEADDR = 0x01; // 默认从机地址
REG_DATA reg_value;
// KEY key1,key2,key3,key4,key5;

uint32_t PWM_Data = 10000; // PWM频率 10KHz
uint16_t Duty_Data = 634;  // 占空比63.4％

uint8_t dir = 0; // 0 ：顺时针   1：逆时针
uint8_t ena = 0; // 0 ：正常运行 1：停机
int circle;
int savecir;
int flage = 0;
int startflage = 0;
float data;
// int stop; // 声明全局变量
int date_array[16];
int state = 0;
int circlespin;
int sportflage=0;
/* 扩展变量 ------------------------------------------------------------------*/
extern __IO uint16_t Toggle_Pulse[4]; /* 步进电机速度控制，可调节范围为 650 -- 3500 ，值越小速度越快 */
/*
 *    当步进电机驱动器细分设置为1时，每200个脉冲步进电机旋转一周
 *                          为32时，每6400个脉冲步进电机旋转一周
 *    下面以设置为32时为例讲解：
 *    pulse_count用于记录输出脉冲数量，pulse_count为脉冲数的两倍，
 *    比如当pulse_count=12800时，实际输出6400个完整脉冲。
 *    这样可以非常方便步进电机的实际转动圈数，就任意角度都有办法控制输出。
 *    如果步进电机驱动器的细分设置为其它值，pulse_count也要做相应处理
 *
 */
__IO uint32_t pulse_count[4]; /*  脉冲计数，一个完整的脉冲会增加2 */

/* 私有函数原形 --------------------------------------------------------------*/
void Check_TimeOut(uint16_t over_time);
void Wait_TimeOut(void);
void read485Data();
void READ485TEST2();
void GENERAL_TIMx_IRQHandler(void);
/* 函数体 --------------------------------------------------------------------*/
/**
 * 函数功能: 系统时钟配置
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 无
 */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE(); // 使能PWR时钟

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1); // 设置调压器输出电压级别1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                   // 打开HSE
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // 打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;       // PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                            // 8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                          // 336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                // 2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 7;                            // USB/SDIO/随机数产生器等的主PLL分频系数
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // 系统时钟：168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        // AHB时钟： 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;         // APB1时钟：42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;         // APB2时钟：84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS(); // 使能CSS功能，优先使用外部晶振，内部时钟源为备用

  // HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
  // HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
  // HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000); // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
 * 函数功能: 主函数.
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 无
 */
int main(void)
{
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();

  /* 配置系统时钟 */
  SystemClock_Config();
  /* 初始化GPIO作为按键 */
  KEY_GPIO_Init();
  /* 初始化定时器配置 */
  STEPMOTOR_TIMx_Init();

  /* 初始化串口并配置串口中断优先级 */
  MX_DEBUG_USART_Init();
  /* 定时器初始化 */
  GENERAL_TIMx_Init();
  /* Modbus使用RS485初始化 */
  RS485_USARTx_Init();
  /* LED初始化 */
  LED_GPIO_Init();
  /* 初始化定时器配置 */


  Rx_MSG = MSG_IDLE;// 接收报文状态
	check_flag=1;	// 读取ID--



  /* 确定定时器 */
  HAL_TIM_Base_Start(&htimx_STEPMOTOR);
  /* 使能中断 关闭比较输出*/
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
		   if (KEY2_StateRead() == KEY_DOWN) // 功能选择
		{
			
			MB_ReadHoldingReg_QID();
		}
		
		/*
		 if(__HAL_UART_GET_FLAG(&husartx_rs485, UART_FLAG_ORE) != RESET)
  {
    __HAL_UART_CLEAR_FLAG(&husartx_rs485, UART_FLAG_ORE); // 清除 ORE 标志位
    __HAL_UART_FLUSH_DRREGISTER(&husartx_rs485); // 清除接收数据寄存器
		
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
      MB_ReadHoldingReg_03H(0x01, 0x02, 0x02); // 读取设备ID-气压传感器1
      HAL_Delay(100);
      break;

    case 2:

      MB_ReadHoldingReg_03H(0x02, 0x02, 0x02); // 读取设备ID-气压传感器2
      HAL_Delay(100);
      break;
    case 3:
      MB_ReadHoldingReg_03H(0x03, 0x02, 0x02); // 读取设备ID-气压传感器3
      HAL_Delay(100);
      break;

    default:
      // 处理默认情况，可以选择不执行任何操作或者执行其他操作
      break;
    }
		rx_flag=1;
			Check_TimeOut(100); //超时检测
			READ485TEST2();
		HAL_Delay(100);
    MB_ReadHoldingReg_BATTERY(); // 读取电池电量
      READ485TEST2();
		HAL_Delay(100);
    MB_ReadHoldingReg_DATE(0xb1, date_array[1], date_array[2], date_array[8], date_array[8]);
   	READ485TEST2();
    	HAL_Delay(100);
			if (KEY1_StateRead() == KEY_DOWN) // 功能选择
			{
					  /* Modbus使用RS485初始化 */
  RS485_USARTx_Init();
			}
	    rx_flag=1;
			Check_TimeOut(50); //超时检测
		}	
 
    if (state == 1&&sportflage==0)
    {
      flage = 0;
      STEPMOTOR_DIR1_FORWARD(); // 正转
      STEPMOTOR_DIR2_FORWARD(); // 正转
      STEPMOTOR_DIR3_FORWARD(); // 正转
      STEPMOTOR_DIR4_FORWARD(); // 正转
      pulse_count[0] = 0;       // 重新计数
      pulse_count[1] = 0;       // 重新计数
      pulse_count[2] = 0;       // 重新计数
      ena = 0;
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO2_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO3_TIM_CHANNEL_x, TIM_CCx_ENABLE);
		  sportflage=1;
    }
    // if (KEY2_StateRead() == KEY_DOWN) // 功能选择
    if (state == 2&&sportflage==0)
    {
      flage = 1;
      STEPMOTOR_DIR1_REVERSAL(); // 反转
      STEPMOTOR_DIR2_REVERSAL(); // 反转
      STEPMOTOR_DIR3_REVERSAL(); // 反转
      STEPMOTOR_DIR4_REVERSAL(); // 反转
      pulse_count[0] = 0;        // 重新计数
      pulse_count[1] = 0;        // 重新计数
      pulse_count[2] = 0;        // 重新计数
      ena = 0;
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO2_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO3_TIM_CHANNEL_x, TIM_CCx_ENABLE);
			 sportflage=1;
    }
    // if (KEY3_StateRead() == KEY_DOWN) // 功能选择
    if (state == 3&&sportflage==0)
    {
      flage = 2;
      STEPMOTOR_DIR1_FORWARD(); // 正转
      STEPMOTOR_DIR2_FORWARD(); // 正转
      STEPMOTOR_DIR3_FORWARD(); // 正转
      STEPMOTOR_DIR4_FORWARD(); // 正转
      pulse_count[0] = 0;       // 重新计数
      pulse_count[1] = 0;       // 重新计数
      pulse_count[2] = 0;       // 重新计数
      ena = 0;
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO2_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO3_TIM_CHANNEL_x, TIM_CCx_ENABLE);
			 sportflage=1;
    }
    // if (KEY4_StateRead() == KEY_DOWN) // 功能选择
    if (state == 4&&sportflage==0)
    {
      flage = 3;
      STEPMOTOR_DIR1_REVERSAL(); // 反转
      STEPMOTOR_DIR2_REVERSAL(); // 反转
      STEPMOTOR_DIR3_REVERSAL(); // 反转
      STEPMOTOR_DIR4_REVERSAL(); // 反转
      pulse_count[0] = 0;        // 重新计数
      pulse_count[1] = 0;        // 重新计数
      pulse_count[2] = 0;        // 重新计数
      ena = 0;
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO2_TIM_CHANNEL_x, TIM_CCx_ENABLE);
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO3_TIM_CHANNEL_x, TIM_CCx_ENABLE);
			 sportflage=1;
    }
    state = 0;
    //read485Data(); // 读取数据
  
  }

  if (pulse_count[0] >= STEPMOTOR_MICRO_STEP * 200 * 2 * 100000) // 第1轴转动40圈后停机
  {
    // 停机
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO1_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  }
  if (pulse_count[1] >= STEPMOTOR_MICRO_STEP * 200 * 2 * 30) // 第2轴转动30圈后停机
  {
    // 停机
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO2_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  }
  if (pulse_count[2] >= STEPMOTOR_MICRO_STEP * 200 * 2 * 20) // 第3轴转动20圈后停机
  {
    // 停机
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO3_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  }
  if (pulse_count[3] >= STEPMOTOR_MICRO_STEP * 200 * 2 * 10) // 第4轴转动10圈后停机
  {
    // 停机
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_NO4_TIM_CHANNEL_x, TIM_CCx_DISABLE);
  }
}

/**
 * 函数功能: 定时器比较输出中断回调函数
 * 输入参数: htim：定时器句柄指针
 * 返 回 值: 无
 * 说    明: 无
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
    if (flage == 0 && data > 100) // 第1轴转动circle圈后停机 1圈=6400  1圈=200*32 脉冲达到圈数之后停止
    {
      // 停机
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
 * 函数功能: 地址超时等待函数
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 发送数据之后,等待从机响应,200ms之后则认为超时
 */
void Check_TimeOut(uint16_t over_time)
{
  uint16_t TimeOut = 0; // 通讯超时 单位:ms
  TimeOut = over_time;  //
  while (Rx_MSG != MSG_COM)
  {
    HAL_Delay(1);
    if (TimeOut-- == 0)
    {
      if (Rx_MSG != MSG_COM) // 200ms后还是没有接受数据，则认为超时
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
 * 函数功能: 超时等待函数
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 发送数据之后,等待从机响应,200ms之后则认为超时
 */
void Wait_TimeOut(void)
{
  uint16_t TimeOut = 0;   // 通讯超时 单位:ms
  TimeOut = TIME_OVERRUN; // 定义超时时间为100ms,但实际测试时间为200ms
  while (Rx_MSG != MSG_COM)
  {
    HAL_Delay(1);
    if (TimeOut-- == 0)
    {
      if (Rx_MSG != MSG_COM) // 200ms后还是没有接受数据，则认为超时
      {
        printf("发送指令失败，请重试\n");
      }
      return;
    }
  }
}

/**
 * 函数功能: 串口接收中断回调函数
 * 输入参数: 串口句柄
 * 返 回 值: 无
 * 说    明: 使用一个定时器的比较中断和更新中断作为接收超时判断
 *           只要接收到数据就将定时器计数器清0,当发生比较中断的时候
 *           说明已经超时1.5个字符的时间,认定为帧错误,如果是更新中断
 *           则认为是接受完成
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &husartx_rs485)
  {
    switch (Rx_MSG)
    {
    /* 接收到第一个字符之后开始计时1.5/3.5个字符的时间 */
    case MSG_IDLE:
      Rx_MSG = MSG_RXING;
      RxCount = 0;
      HAL_TIM_Base_Start(&htimx);
      break;

    /* 距离上一次接收到数据已经超过1.5个字符的时间间隔,认定为数据帧不完整 */
    case MSG_ERR_FLAG:
      LED1_TOGGLE;
      Rx_MSG = MSG_INC; // 数据帧不完整
      break;
    }

    /* 使能继续接收 */
    Rx_Buf[RxCount] = tmp_Rx_Buf;
    RxCount++;
    __HAL_TIM_SET_COUNTER(&htimx, 0); // 重设计数时间
    HAL_UART_Receive_IT(&husartx_rs485, (uint8_t *)&tmp_Rx_Buf, 1);
  }
}

/**
 * 函数功能: 定时器比较中断回调函数
 * 输入参数: 定时器句柄
 * 返 回 值: 无
 * 说    明: 标记已经超时1.5个字符的时间没有接收到数据
 */
void HAL_TIM_OC_DelayElapsedCallback2(TIM_HandleTypeDef *htim)
{
  /* 如果是第一次发生比较中断,则暂定为错误标记 */
  if (Rx_MSG != MSG_INC)
    Rx_MSG = MSG_ERR_FLAG;

  /* 如果是第二次进入比较中断,则认定为报文不完整 */
  else
    Rx_MSG = MSG_INC;
}

/**
 * 函数功能: 定时器更新中断回调函数
 * 输入参数: 定时器句柄
 * 返 回 值: 无
 * 说    明: 超时3.5个字符的时间没有接收到数据,认为是空闲状态
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 如果已经标记了接收到不完整的数据帧,则继续标记为不完整的数据帧 */
  if (Rx_MSG == MSG_INC)
  {
    Rx_MSG = MSG_INC;
  }
  /* 在正常情况下时接收完成 */
  else
  {
    Rx_MSG = MSG_COM;
  }
}

void read485Data()
{

  Rx_MSG = MSG_IDLE;
  MB_ReadHoldingReg_03H(0x01, 0x02, 0x02); // 读取设备ID
  rx_flag = 1;
  printf("发送数据");
  Check_TimeOut(50); // 超时检测
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
  Check_TimeOut(50); // 超时检测
  printf("Result: 0x%08X\n", result);
  printf("IEEE 754 十进制: %f\n", converter.floatValue);
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
  //   printf("IEEE 754 十进制: %f\n", converter.floatValue);
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
  /* 读取从地址ID，验证是否通信正常 */
 // if (check_flag == 1) // 读取ID
 // {
  //  Rx_MSG = MSG_IDLE;                               // 清除接收标志
  //  MB_ReadHoldingReg_03H(MB_SLAVEADDR, 1200, 0x01); // 读取设备ID
  //  rx_flag = 1;                                     // 接收标志
  //  Check_TimeOut(100);                              // 超时检测
 // }
  Rx_MSG = MSG_COM;
  if (Rx_MSG == MSG_COM)
  {
    if (rx_flag == 1) // 接收到数据--读取ID--0x04 0x19或0x04 0x20
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
    Check_TimeOut(50); // 超时检测
    printf("Result: 0x%08X\n", result);
    printf("IEEE 754 十进制: %f\n", converter.floatValue);
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
      rx_flag = 0; // 清除接收标志
    }
    else // 未接受数据
    {
      for (uint8_t x = 0; x < 8; x++) // 打印接收到的数据
      {
       // LED3_TOGGLE;
        printf("Rx_Buf[%d]=%x\n", x, Rx_Buf[x]);
      }
    }
    static uint16_t crc_check = 0;                                  // CRC校验
    crc_check = ((Rx_Buf[RxCount - 1] << 8) | Rx_Buf[RxCount - 2]); // CRC校验
    /* CRC 校验正确 */
    if (crc_check == MB_CRC16((uint8_t *)&Rx_Buf, RxCount - 2)) // CRC校验
    {
      /* 通信错误 */
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

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
