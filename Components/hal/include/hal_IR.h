#ifndef HAL_IR_H
#define HAL_IR_H

//IR Interrupt option
#define HAL_IR_INTERRUPT_DISABLE 0x00
#define HAL_IR_INTERRUPT_ENABLE 0x01

#ifdef __cplusplus
extern "C"
{
#endif

#include "hal_board.h"

extern void Hal_IR_RCVD_Int_Config( bool interruptEnable );

extern void Hal_IR_RCVD_T1_Config( void );

extern void Hal_IR_RCVD_Init( void );

extern void initUARTtest( void );

extern void UartTX_Send_String(unsigned short *Data,int len);

#ifdef __cplusplus
}

#endif
#endif
