#ifndef __BSP_MB_HOST_H__
#define __BSP_MB_HOST_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* 类型定义 ------------------------------------------------------------------*/

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
uint16_t MB_CRC16(uint8_t *pushMsg,uint8_t usDataLen);
void MB_ReadCoil_01H(uint8_t _addr, uint16_t _reg, uint16_t _num);
void MB_WriteCoil_05H(uint8_t _addr, uint16_t _reg, uint16_t _num);
void MB_ReadInput_02H(uint8_t _addr, uint16_t _reg, uint16_t _num);
void MB_ReadHoldingReg_03H(uint8_t _addr, uint16_t _reg, uint16_t _num);
void MB_ReadInputReg_04H(uint8_t _addr, uint16_t _reg, uint16_t _num);
void MB_WriteHoldingReg_06H(uint8_t _addr, uint16_t _reg, uint16_t _data);
void MB_WriteNumHoldingReg_10H(uint8_t _addr, uint16_t _reg, uint16_t _num,uint8_t *_databuf);

#endif /* __BSP_MB_HOST_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
