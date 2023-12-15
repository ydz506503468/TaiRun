#ifndef __STEPMOTOR_TIM_H__
#define __STEPMOTOR_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
#define STEPMOTOR_TIMx                        TIM8
#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM8_CLK_ENABLE()
#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM8_CLK_DISABLE()
#define STEPMOTOR_TIMx_IRQn                   TIM8_CC_IRQn
#define STEPMOTOR_TIMx_IRQHandler             TIM8_CC_IRQHandler

/* ��1�� */
#define STEPMOTOR_NO1_TIM_CHANNEL_x            TIM_CHANNEL_1
#define STEPMOTOR_TIM_PUL1_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOI_CLK_ENABLE()     // ���������������������
#define STEPMOTOR_TIM_PUL1_PORT                GPIOI                            // ��Ӧ��������PUL-��������ʹ�ù����ӷ���
#define STEPMOTOR_TIM_PUL1_PIN                 GPIO_PIN_5                       // ��PLU+ֱ�ӽӿ������VCC

#define STEPMOTOR_DIR1_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()     // �����ת������ƣ�������ղ���Ĭ����ת
#define STEPMOTOR_DIR1_PORT                    GPIOD                            // ��Ӧ��������DIR-��������ʹ�ù����ӷ���
#define STEPMOTOR_DIR1_PIN                     GPIO_PIN_3                       // ��DIR+ֱ�ӽӿ������VCC

#define STEPMOTOR_ENA1_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()     // ����ѻ�ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define STEPMOTOR_ENA1_PORT                    GPIOD                            // ��Ӧ��������ENA-��������ʹ�ù����ӷ���
#define STEPMOTOR_ENA1_PIN                     GPIO_PIN_7                       // ��ENA+ֱ�ӽӿ������VCC

#define STEPMOTOR_DIR1_FORWARD()               HAL_GPIO_WritePin(STEPMOTOR_DIR1_PORT,STEPMOTOR_DIR1_PIN,GPIO_PIN_SET)
#define STEPMOTOR_DIR1_REVERSAL()              HAL_GPIO_WritePin(STEPMOTOR_DIR1_PORT,STEPMOTOR_DIR1_PIN,GPIO_PIN_RESET)

#define STEPMOTOR_OUTPUT1_ENABLE()             HAL_GPIO_WritePin(STEPMOTOR_ENA1_PORT,STEPMOTOR_ENA1_PIN,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT1_DISABLE()            HAL_GPIO_WritePin(STEPMOTOR_ENA1_PORT,STEPMOTOR_ENA1_PIN,GPIO_PIN_SET)

/* ��2�� */
#define STEPMOTOR_NO2_TIM_CHANNEL_x            TIM_CHANNEL_2
#define STEPMOTOR_TIM_PUL2_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOI_CLK_ENABLE()     // ���������������������
#define STEPMOTOR_TIM_PUL2_PORT                GPIOI                            // ��Ӧ��������PUL-��������ʹ�ù����ӷ���
#define STEPMOTOR_TIM_PUL2_PIN                 GPIO_PIN_6                       // ��PLU+ֱ�ӽӿ������VCC

#define STEPMOTOR_DIR2_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()     // �����ת������ƣ�������ղ���Ĭ����ת
#define STEPMOTOR_DIR2_PORT                    GPIOD                            // ��Ӧ��������DIR-��������ʹ�ù����ӷ���
#define STEPMOTOR_DIR2_PIN                     GPIO_PIN_11                       // ��DIR+ֱ�ӽӿ������VCC

#define STEPMOTOR_ENA2_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()     // ����ѻ�ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define STEPMOTOR_ENA2_PORT                    GPIOF                            // ��Ӧ��������ENA-��������ʹ�ù����ӷ���
#define STEPMOTOR_ENA2_PIN                     GPIO_PIN_11                       // ��ENA+ֱ�ӽӿ������VCC

#define STEPMOTOR_DIR2_FORWARD()               HAL_GPIO_WritePin(STEPMOTOR_DIR2_PORT,STEPMOTOR_DIR2_PIN,GPIO_PIN_SET)
#define STEPMOTOR_DIR2_REVERSAL()              HAL_GPIO_WritePin(STEPMOTOR_DIR2_PORT,STEPMOTOR_DIR2_PIN,GPIO_PIN_RESET)

#define STEPMOTOR_OUTPUT2_ENABLE()             HAL_GPIO_WritePin(STEPMOTOR_ENA2_PORT,STEPMOTOR_ENA2_PIN,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT2_DISABLE()            HAL_GPIO_WritePin(STEPMOTOR_ENA2_PORT,STEPMOTOR_ENA2_PIN,GPIO_PIN_SET)

/* ��3��*/
#define STEPMOTOR_NO3_TIM_CHANNEL_x            TIM_CHANNEL_3
#define STEPMOTOR_TIM_PUL3_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOI_CLK_ENABLE()     // ���������������������
#define STEPMOTOR_TIM_PUL3_PORT                GPIOI                            // ��Ӧ�����������PUL-��������ʹ�ù����ӷ���
#define STEPMOTOR_TIM_PUL3_PIN                 GPIO_PIN_7                       // ��PLU+ֱ�ӽӿ������5V(����3.3V)

#define STEPMOTOR_DIR3_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()     // �����ת������ƣ�������ղ���Ĭ����ת
#define STEPMOTOR_DIR3_PORT                    GPIOF                            // ��Ӧ�����������DIR-��������ʹ�ù����ӷ���
#define STEPMOTOR_DIR3_PIN                     GPIO_PIN_1                      // ��DIR+ֱ�ӽӿ������5V(����3.3V)

#define STEPMOTOR_ENA3_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE()     // ���ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define STEPMOTOR_ENA3_PORT                    GPIOF                            // ��Ӧ�����������ENA-��������ʹ�ù����ӷ���
#define STEPMOTOR_ENA3_PIN                     GPIO_PIN_2                       // ��ENA+ֱ�ӽӿ������5V(����3.3V) 

#define STEPMOTOR_DIR3_FORWARD()               HAL_GPIO_WritePin(STEPMOTOR_DIR3_PORT,STEPMOTOR_DIR3_PIN,GPIO_PIN_SET)
#define STEPMOTOR_DIR3_REVERSAL()              HAL_GPIO_WritePin(STEPMOTOR_DIR3_PORT,STEPMOTOR_DIR3_PIN,GPIO_PIN_RESET)

#define STEPMOTOR_OUTPUT3_ENABLE()             HAL_GPIO_WritePin(STEPMOTOR_ENA3_PORT,STEPMOTOR_ENA3_PIN,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT3_DISABLE()            HAL_GPIO_WritePin(STEPMOTOR_ENA3_PORT,STEPMOTOR_ENA3_PIN,GPIO_PIN_SET)

/* ��4��*/
#define STEPMOTOR_NO4_TIM_CHANNEL_x            TIM_CHANNEL_4
#define STEPMOTOR_TIM_PUL4_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOI_CLK_ENABLE()     // ���������������������
#define STEPMOTOR_TIM_PUL4_PORT                GPIOI                            // ��Ӧ�����������PUL-��������ʹ�ù����ӷ���
#define STEPMOTOR_TIM_PUL4_PIN                 GPIO_PIN_2                       // ��PLU+ֱ�ӽӿ������5V(����3.3V)

#define STEPMOTOR_DIR4_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOC_CLK_ENABLE()     // �����ת������ƣ�������ղ���Ĭ����ת
#define STEPMOTOR_DIR4_PORT                    GPIOC                            // ��Ӧ�����������DIR-��������ʹ�ù����ӷ���
#define STEPMOTOR_DIR4_PIN                     GPIO_PIN_8                       // ��DIR+ֱ�ӽӿ������5V(����3.3V)

#define STEPMOTOR_ENA4_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOH_CLK_ENABLE()     // ���ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define STEPMOTOR_ENA4_PORT                    GPIOH                            // ��Ӧ�����������ENA-��������ʹ�ù����ӷ���
#define STEPMOTOR_ENA4_PIN                     GPIO_PIN_7                       // ��ENA+ֱ�ӽӿ������5V(����3.3V) 

#define STEPMOTOR_DIR4_FORWARD()               HAL_GPIO_WritePin(STEPMOTOR_DIR4_PORT,STEPMOTOR_DIR4_PIN,GPIO_PIN_SET)
#define STEPMOTOR_DIR4_REVERSAL()              HAL_GPIO_WritePin(STEPMOTOR_DIR4_PORT,STEPMOTOR_DIR4_PIN,GPIO_PIN_RESET)

#define STEPMOTOR_OUTPUT4_ENABLE()             HAL_GPIO_WritePin(STEPMOTOR_ENA4_PORT,STEPMOTOR_ENA4_PIN,GPIO_PIN_RESET)
#define STEPMOTOR_OUTPUT4_DISABLE()            HAL_GPIO_WritePin(STEPMOTOR_ENA4_PORT,STEPMOTOR_ENA4_PIN,GPIO_PIN_SET)



//˽�б�������
#define ORIGIN_GPIO_CLK_ENABLE()        	  	__HAL_RCC_GPIOG_CLK_ENABLE()     // ԭ������������
#define ORIGIN_PORT                       		GPIOG                            // 
#define ORIGIN_PIN                      		  GPIO_PIN_0  
#define ORIGIN_EXTI_IRQn			    						EXTI0_IRQn
#define ORIGIN_EXTI_IRQHandler                EXTI0_IRQHandler 

#define LIMPOS_GPIO_CLK_ENABLE()      				__HAL_RCC_GPIOG_CLK_ENABLE()     // ��ת��λ��������
#define LIMPOS_PORT                   				GPIOG                            // ��Ч��ƽ�ǵ͵�ƽ,��GND�̽Ӽ���
#define LIMPOS_PIN                    				GPIO_PIN_1                       // 
#define LIMPOS_EXTI_IRQn											EXTI1_IRQn
#define LIMPOS_EXTI_IRQHandler                EXTI1_IRQHandler                 // �жϷ��������

#define LIMNEG_GPIO_CLK_ENABLE()      				__HAL_RCC_GPIOG_CLK_ENABLE()     // ��ת��λ��������
#define LIMNEG_PORT                   				GPIOG                            // ��Ч��ƽ�ǵ͵�ƽ,��GND�̽Ӽ���
#define LIMNEG_PIN                    				GPIO_PIN_2                       // 
#define LIMNEG_EXTI_IRQn											EXTI2_IRQn
#define LIMNEG_EXTI_IRQHandler                EXTI2_IRQHandler                 // �жϷ��������




//�ӽ����ض�ȡ
#define ORIGIN_GPIO_CLK_ENABLE()        	  	__HAL_RCC_GPIOG_CLK_ENABLE()     // ԭ������������
#define ORIGIN_PORT                       		GPIOG                            // 
#define ORIGIN_PIN                      		  GPIO_PIN_0  
#define ORIGIN_EXTI_IRQn			    						EXTI0_IRQn
#define ORIGIN_EXTI_IRQHandler                EXTI0_IRQHandler 

#define LIMPOS_GPIO_CLK_ENABLE()      				__HAL_RCC_GPIOG_CLK_ENABLE()     // ��ת��λ��������
#define LIMPOS_PORT                   				GPIOG                            // ��Ч��ƽ�ǵ͵�ƽ,��GND�̽Ӽ���
#define LIMPOS_PIN                    				GPIO_PIN_1                       // 
#define LIMPOS_EXTI_IRQn											EXTI1_IRQn
#define LIMPOS_EXTI_IRQHandler                EXTI1_IRQHandler                 // �жϷ��������

#define LIMNEG_GPIO_CLK_ENABLE()      				__HAL_RCC_GPIOG_CLK_ENABLE()     // ��ת��λ��������
#define LIMNEG_PORT                   				GPIOG                            // ��Ч��ƽ�ǵ͵�ƽ,��GND�̽Ӽ���
#define LIMNEG_PIN                    				GPIO_PIN_2                       // 
#define LIMNEG_EXTI_IRQn											EXTI2_IRQn
#define LIMNEG_EXTI_IRQHandler                EXTI2_IRQHandler                 // �жϷ��������



// ����߼���ʱ���ظ������Ĵ���ֵ
#define STEPMOTOR_TIM_REPETITIONCOUNTER       0

#define FALSE                                 0
#define TRUE                                  1
#define CW                                    1  // ˳ʱ��:������
#define CCW                                   -1 // ��ʱ��:������
#define LIM_POS_LEVEL						              0	 // ������������Ч��ƽ
#define LIM_NEG_LEVEL						              0	 // ������������Ч��ƽ
#define ORIGIN_LEVEL                          0
#define HOME_POSITION	                        0  // ԭ������
#define STOP                                  0  // �Ӽ�������״̬��ֹͣ
#define ACCEL                                 1  // �Ӽ�������״̬�����ٽ׶�
#define DECEL                                 2  // �Ӽ�������״̬�����ٽ׶�
#define RUN                                   3  // �Ӽ�������״̬�����ٽ׶�
#define IDLE	   						                  0	 // ����ԭ��״̬:����
#define FASTSEEK   							              1  // ����ԭ��״̬:��������
#define SLOWSEEK 							                2  // ����ԭ��״̬:��������
#define MOVETOZERO 							              3  // ����ԭ��״̬:����ԭ��
#define T1_FREQ                               (SystemCoreClock/(STEPMOTOR_TIM_PRESCALER+1)) // Ƶ��ftֵ
#define FSPR                                  200// ���������Ȧ����  �����:1.8�� 360/1.8 = 200 �����������Ҫ200��תһȦ
#define MICRO_STEP                            32 // �������������ϸ����
#define SPR                                   (FSPR*MICRO_STEP)   // ��תһȦ��Ҫ��������

// ��ѧ����
#define ALPHA                                 ((float)(2*3.14159/SPR))       // ��= 2*pi/spr�����
#define A_T_x10                               ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148                           ((float)((T1_FREQ*0.676)/10))  // 0.676Ϊ�������ֵ
#define A_SQ                                  ((float)(2*100000*ALPHA)) 
#define A_x200                                ((float)(200*ALPHA))
#define MAX_NUM_LAP 						              INT32_MAX
#define MAX_NUM_STEP 						              UINT32_MAX

#define UNIT_STEP_MM                          (SPR/UNITS_DISTANCE)//����1mm��Ҫ�Ĳ���
#define MAX_STEP_MM                           (MAX_DISTANCE/UNITS_DISTANCE)*UNIT_STEP_MM //

#define UNITS_DISTANCE                        5   // �������תһȦ,����ǰ��5mm
#define MAX_DISTANCE                          400 // ��������ƶ��������400mm



extern int stop;
extern int date_array[16];

#define GPIO_AFx_TIMx                          GPIO_AF3_TIM8
// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/��STEPMOTOR_TIMx_PRESCALER+1��
#define STEPMOTOR_TIM_PRESCALER               5  // �������������ϸ������Ϊ��   32  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               9  // �������������ϸ������Ϊ��   16  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               19  // �������������ϸ������Ϊ��   8  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               39  // �������������ϸ������Ϊ��   4  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               79  // �������������ϸ������Ϊ��   2  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               159 // �������������ϸ������Ϊ��   1  ϸ��


// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define STEPMOTOR_TIM_PERIOD                   0xFFFF
// ����߼���ʱ���ظ������Ĵ���ֵ
#define STEPMOTOR_TIM_REPETITIONCOUNTER       0

/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_STEPMOTOR;
extern __IO uint16_t Toggle_Pulse[4];
/* �������� ------------------------------------------------------------------*/

void STEPMOTOR_TIMx_Init(void);

#endif	/* __STEPMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
