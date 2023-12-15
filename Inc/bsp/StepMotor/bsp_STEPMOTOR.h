#ifndef __STEPMOTOR_TIM_H__
#define __STEPMOTOR_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define STEPMOTOR_TIMx                        TIM8
#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM8_CLK_ENABLE()
#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM8_CLK_DISABLE()
#define STEPMOTOR_TIMx_IRQn                   TIM8_CC_IRQn
#define STEPMOTOR_TIMx_IRQHandler             TIM8_CC_IRQHandler

/* 第1轴 */
#define STEPMOTOR_NO1_TIM_CHANNEL_x            TIM_CHANNEL_1
#define STEPMOTOR_TIM_PUL1_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOI_CLK_ENABLE()     // 输出控制脉冲给电机驱动器
#define STEPMOTOR_TIM_PUL1_PORT                GPIOI                            // 对应驱动器的PUL-（驱动器使用共阳接法）
#define STEPMOTOR_TIM_PUL1_PIN                 GPIO_PIN_5                       // 而PLU+直接接开发板的VCC

#define STEPMOTOR_DIR1_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()     // 电机旋转方向控制，如果悬空不接默认正转
#define STEPMOTOR_DIR1_PORT                    GPIOD                            // 对应驱动器的DIR-（驱动器使用共阳接法）
#define STEPMOTOR_DIR1_PIN                     GPIO_PIN_3                       // 而DIR+直接接开发板的VCC

#define STEPMOTOR_ENA1_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()     // 电机脱机使能控制，如果悬空不接默认使能电机
#define STEPMOTOR_ENA1_PORT                    GPIOD                            // 对应驱动器的ENA-（驱动器使用共阳接法）
#define STEPMOTOR_ENA1_PIN                     GPIO_PIN_7                       // 而ENA+直接接开发板的VCC

#define STEPMOTOR_DIR1_FORWARD()               HAL_GPIO_WritePin(STEPMOTOR_DIR1_PORT,STEPMOTOR_DIR1_PIN,GPIO_PIN_SET)
#define STEPMOTOR_DIR1_REVERSAL()              HAL_GPIO_WritePin(STEPMOTOR_DIR1_PORT,STEPMOTOR_DIR1_PIN,GPIO_PIN_RESET)

#define STEPMOTOR_OUTPUT1_ENABLE()             HAL_GPIO_WritePin(STEPMOTOR_ENA1_PORT,STEPMOTOR_ENA1_PIN,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT1_DISABLE()            HAL_GPIO_WritePin(STEPMOTOR_ENA1_PORT,STEPMOTOR_ENA1_PIN,GPIO_PIN_SET)

/* 第2轴 */
#define STEPMOTOR_NO2_TIM_CHANNEL_x            TIM_CHANNEL_2
#define STEPMOTOR_TIM_PUL2_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOI_CLK_ENABLE()     // 输出控制脉冲给电机驱动器
#define STEPMOTOR_TIM_PUL2_PORT                GPIOI                            // 对应驱动器的PUL-（驱动器使用共阳接法）
#define STEPMOTOR_TIM_PUL2_PIN                 GPIO_PIN_6                       // 而PLU+直接接开发板的VCC

#define STEPMOTOR_DIR2_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()     // 电机旋转方向控制，如果悬空不接默认正转
#define STEPMOTOR_DIR2_PORT                    GPIOD                            // 对应驱动器的DIR-（驱动器使用共阳接法）
#define STEPMOTOR_DIR2_PIN                     GPIO_PIN_11                       // 而DIR+直接接开发板的VCC

#define STEPMOTOR_ENA2_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()     // 电机脱机使能控制，如果悬空不接默认使能电机
#define STEPMOTOR_ENA2_PORT                    GPIOF                            // 对应驱动器的ENA-（驱动器使用共阳接法）
#define STEPMOTOR_ENA2_PIN                     GPIO_PIN_11                       // 而ENA+直接接开发板的VCC

#define STEPMOTOR_DIR2_FORWARD()               HAL_GPIO_WritePin(STEPMOTOR_DIR2_PORT,STEPMOTOR_DIR2_PIN,GPIO_PIN_SET)
#define STEPMOTOR_DIR2_REVERSAL()              HAL_GPIO_WritePin(STEPMOTOR_DIR2_PORT,STEPMOTOR_DIR2_PIN,GPIO_PIN_RESET)

#define STEPMOTOR_OUTPUT2_ENABLE()             HAL_GPIO_WritePin(STEPMOTOR_ENA2_PORT,STEPMOTOR_ENA2_PIN,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT2_DISABLE()            HAL_GPIO_WritePin(STEPMOTOR_ENA2_PORT,STEPMOTOR_ENA2_PIN,GPIO_PIN_SET)

/* 第3轴*/
#define STEPMOTOR_NO3_TIM_CHANNEL_x            TIM_CHANNEL_3
#define STEPMOTOR_TIM_PUL3_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOI_CLK_ENABLE()     // 输出控制脉冲给电机控制器
#define STEPMOTOR_TIM_PUL3_PORT                GPIOI                            // 对应电机驱动器的PUL-（控制器使用共阳接法）
#define STEPMOTOR_TIM_PUL3_PIN                 GPIO_PIN_7                       // 而PLU+直接接开发板的5V(或者3.3V)

#define STEPMOTOR_DIR3_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()     // 电机旋转方向控制，如果悬空不接默认正转
#define STEPMOTOR_DIR3_PORT                    GPIOF                            // 对应电机驱动器的DIR-（控制器使用共阳接法）
#define STEPMOTOR_DIR3_PIN                     GPIO_PIN_1                      // 而DIR+直接接开发板的5V(或者3.3V)

#define STEPMOTOR_ENA3_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()     // 电机使能控制，如果悬空不接默认使能电机
#define STEPMOTOR_ENA3_PORT                    GPIOF                            // 对应电机驱动器的ENA-（控制器使用共阳接法）
#define STEPMOTOR_ENA3_PIN                     GPIO_PIN_2                       // 而ENA+直接接开发板的5V(或者3.3V) 

#define STEPMOTOR_DIR3_FORWARD()               HAL_GPIO_WritePin(STEPMOTOR_DIR3_PORT,STEPMOTOR_DIR3_PIN,GPIO_PIN_SET)
#define STEPMOTOR_DIR3_REVERSAL()              HAL_GPIO_WritePin(STEPMOTOR_DIR3_PORT,STEPMOTOR_DIR3_PIN,GPIO_PIN_RESET)

#define STEPMOTOR_OUTPUT3_ENABLE()             HAL_GPIO_WritePin(STEPMOTOR_ENA3_PORT,STEPMOTOR_ENA3_PIN,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT3_DISABLE()            HAL_GPIO_WritePin(STEPMOTOR_ENA3_PORT,STEPMOTOR_ENA3_PIN,GPIO_PIN_SET)

/* 第4轴*/
#define STEPMOTOR_NO4_TIM_CHANNEL_x            TIM_CHANNEL_4
#define STEPMOTOR_TIM_PUL4_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOI_CLK_ENABLE()     // 输出控制脉冲给电机控制器
#define STEPMOTOR_TIM_PUL4_PORT                GPIOI                            // 对应电机驱动器的PUL-（控制器使用共阳接法）
#define STEPMOTOR_TIM_PUL4_PIN                 GPIO_PIN_2                       // 而PLU+直接接开发板的5V(或者3.3V)

#define STEPMOTOR_DIR4_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOC_CLK_ENABLE()     // 电机旋转方向控制，如果悬空不接默认正转
#define STEPMOTOR_DIR4_PORT                    GPIOC                            // 对应电机驱动器的DIR-（控制器使用共阳接法）
#define STEPMOTOR_DIR4_PIN                     GPIO_PIN_8                       // 而DIR+直接接开发板的5V(或者3.3V)

#define STEPMOTOR_ENA4_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOH_CLK_ENABLE()     // 电机使能控制，如果悬空不接默认使能电机
#define STEPMOTOR_ENA4_PORT                    GPIOH                            // 对应电机驱动器的ENA-（控制器使用共阳接法）
#define STEPMOTOR_ENA4_PIN                     GPIO_PIN_7                       // 而ENA+直接接开发板的5V(或者3.3V) 

#define STEPMOTOR_DIR4_FORWARD()               HAL_GPIO_WritePin(STEPMOTOR_DIR4_PORT,STEPMOTOR_DIR4_PIN,GPIO_PIN_SET)
#define STEPMOTOR_DIR4_REVERSAL()              HAL_GPIO_WritePin(STEPMOTOR_DIR4_PORT,STEPMOTOR_DIR4_PIN,GPIO_PIN_RESET)

#define STEPMOTOR_OUTPUT4_ENABLE()             HAL_GPIO_WritePin(STEPMOTOR_ENA4_PORT,STEPMOTOR_ENA4_PIN,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT4_DISABLE()            HAL_GPIO_WritePin(STEPMOTOR_ENA4_PORT,STEPMOTOR_ENA4_PIN,GPIO_PIN_SET)



//私有变量定义
#define ORIGIN_GPIO_CLK_ENABLE()        	  	__HAL_RCC_GPIOG_CLK_ENABLE()     // 原点检测输入引脚
#define ORIGIN_PORT                       		GPIOG                            // 
#define ORIGIN_PIN                      		  GPIO_PIN_0  
#define ORIGIN_EXTI_IRQn			    						EXTI0_IRQn
#define ORIGIN_EXTI_IRQHandler                EXTI0_IRQHandler 

#define LIMPOS_GPIO_CLK_ENABLE()      				__HAL_RCC_GPIOG_CLK_ENABLE()     // 正转限位输入引脚
#define LIMPOS_PORT                   				GPIOG                            // 有效电平是低电平,与GND短接即可
#define LIMPOS_PIN                    				GPIO_PIN_1                       // 
#define LIMPOS_EXTI_IRQn											EXTI1_IRQn
#define LIMPOS_EXTI_IRQHandler                EXTI1_IRQHandler                 // 中断服务函数入口

#define LIMNEG_GPIO_CLK_ENABLE()      				__HAL_RCC_GPIOG_CLK_ENABLE()     // 反转限位输入引脚
#define LIMNEG_PORT                   				GPIOG                            // 有效电平是低电平,与GND短接即可
#define LIMNEG_PIN                    				GPIO_PIN_2                       // 
#define LIMNEG_EXTI_IRQn											EXTI2_IRQn
#define LIMNEG_EXTI_IRQHandler                EXTI2_IRQHandler                 // 中断服务函数入口




//接近开关读取
#define ORIGIN_GPIO_CLK_ENABLE()        	  	__HAL_RCC_GPIOG_CLK_ENABLE()     // 原点检测输入引脚
#define ORIGIN_PORT                       		GPIOG                            // 
#define ORIGIN_PIN                      		  GPIO_PIN_0  
#define ORIGIN_EXTI_IRQn			    						EXTI0_IRQn
#define ORIGIN_EXTI_IRQHandler                EXTI0_IRQHandler 

#define LIMPOS_GPIO_CLK_ENABLE()      				__HAL_RCC_GPIOG_CLK_ENABLE()     // 正转限位输入引脚
#define LIMPOS_PORT                   				GPIOG                            // 有效电平是低电平,与GND短接即可
#define LIMPOS_PIN                    				GPIO_PIN_1                       // 
#define LIMPOS_EXTI_IRQn											EXTI1_IRQn
#define LIMPOS_EXTI_IRQHandler                EXTI1_IRQHandler                 // 中断服务函数入口

#define LIMNEG_GPIO_CLK_ENABLE()      				__HAL_RCC_GPIOG_CLK_ENABLE()     // 反转限位输入引脚
#define LIMNEG_PORT                   				GPIOG                            // 有效电平是低电平,与GND短接即可
#define LIMNEG_PIN                    				GPIO_PIN_2                       // 
#define LIMNEG_EXTI_IRQn											EXTI2_IRQn
#define LIMNEG_EXTI_IRQHandler                EXTI2_IRQHandler                 // 中断服务函数入口



// 定义高级定时器重复计数寄存器值
#define STEPMOTOR_TIM_REPETITIONCOUNTER       0

#define FALSE                                 0
#define TRUE                                  1
#define CW                                    1  // 顺时针:正方向
#define CCW                                   -1 // 逆时针:反方向
#define LIM_POS_LEVEL						              0	 // 正极限引脚有效电平
#define LIM_NEG_LEVEL						              0	 // 负极限引脚有效电平
#define ORIGIN_LEVEL                          0
#define HOME_POSITION	                        0  // 原点坐标
#define STOP                                  0  // 加减速曲线状态：停止
#define ACCEL                                 1  // 加减速曲线状态：加速阶段
#define DECEL                                 2  // 加减速曲线状态：减速阶段
#define RUN                                   3  // 加减速曲线状态：匀速阶段
#define IDLE	   						                  0	 // 搜索原点状态:空闲
#define FASTSEEK   							              1  // 搜索原点状态:快速搜索
#define SLOWSEEK 							                2  // 搜索原点状态:慢速搜索
#define MOVETOZERO 							              3  // 搜索原点状态:捕获原点
#define T1_FREQ                               (SystemCoreClock/(STEPMOTOR_TIM_PRESCALER+1)) // 频率ft值
#define FSPR                                  200// 步进电机单圈步数  步距角:1.8° 360/1.8 = 200 正常情况下需要200步转一圈
#define MICRO_STEP                            32 // 步进电机驱动器细分数
#define SPR                                   (FSPR*MICRO_STEP)   // 旋转一圈需要的脉冲数

// 数学常数
#define ALPHA                                 ((float)(2*3.14159/SPR))       // α= 2*pi/spr步距角
#define A_T_x10                               ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148                           ((float)((T1_FREQ*0.676)/10))  // 0.676为误差修正值
#define A_SQ                                  ((float)(2*100000*ALPHA)) 
#define A_x200                                ((float)(200*ALPHA))
#define MAX_NUM_LAP 						              INT32_MAX
#define MAX_NUM_STEP 						              UINT32_MAX

#define UNIT_STEP_MM                          (SPR/UNITS_DISTANCE)//步进1mm需要的步数
#define MAX_STEP_MM                           (MAX_DISTANCE/UNITS_DISTANCE)*UNIT_STEP_MM //

#define UNITS_DISTANCE                        5   // 步进电机转一圈,导轨前进5mm
#define MAX_DISTANCE                          400 // 导轨可以移动的最长距离400mm



extern int stop;
extern int date_array[16];

#define GPIO_AFx_TIMx                          GPIO_AF3_TIM8
// 定义定时器预分频，定时器实际时钟频率为：168MHz/（STEPMOTOR_TIMx_PRESCALER+1）
#define STEPMOTOR_TIM_PRESCALER               5  // 步进电机驱动器细分设置为：   32  细分
//#define STEPMOTOR_TIM_PRESCALER               9  // 步进电机驱动器细分设置为：   16  细分
//#define STEPMOTOR_TIM_PRESCALER               19  // 步进电机驱动器细分设置为：   8  细分
//#define STEPMOTOR_TIM_PRESCALER               39  // 步进电机驱动器细分设置为：   4  细分
//#define STEPMOTOR_TIM_PRESCALER               79  // 步进电机驱动器细分设置为：   2  细分
//#define STEPMOTOR_TIM_PRESCALER               159 // 步进电机驱动器细分设置为：   1  细分


// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define STEPMOTOR_TIM_PERIOD                   0xFFFF
// 定义高级定时器重复计数寄存器值
#define STEPMOTOR_TIM_REPETITIONCOUNTER       0

/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_STEPMOTOR;
extern __IO uint16_t Toggle_Pulse[4];
/* 函数声明 ------------------------------------------------------------------*/

void STEPMOTOR_TIMx_Init(void);

#endif	/* __STEPMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
