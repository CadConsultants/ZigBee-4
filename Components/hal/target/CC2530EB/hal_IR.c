#include "hal_IR.h"
#include "hal_mcu.h"

#if (defined HAL_IR) && (HAL_IR == TRUE)

bool Hal_IR_RCVD_IntEnable;            /* interrupt enable/disable flag */

void initUARTtest( void )
{
    PERCFG &= ~0x01;			//位置1 P0�?    
    P0SEL |= 0x0c;				//P0用作串口

    U0CSR |= 0x80;				//UART方式
    U0GCR |= 10;				//baud_e
    U0BAUD |= 216;				//波特率设�?7600
    UTX0IF = 0;
}

void Hal_IR_RCVD_Int_Config( bool interruptEnable)
{
    Hal_IR_RCVD_IntEnable = interruptEnable;

    if( Hal_IR_RCVD_IntEnable )
    {
        P0SEL &= ~0x10;  //P0.4 General I/O
        P0DIR &= ~0x10;  //P0.4 Input
        P2INP &= ~0x20;  //P0 Pullup
        P0INP &= ~0x10;  //P0.4 Pullup

        EA = 1;          //global IEN
        PICTL |= 0x01;   //P0 interruptEnable on falling edge
        P0IEN |= 0x10;   //P0.4 interruptEnable
        P0IE = 1;        //P0 IEN
        P0IFG &= ~0x10;  //clear P0.4 interrupt mask
    }
    else
    {
        P0IEN &= ~0x10;  //P0.4 interruptDisable
    }
}

void Hal_IR_RCVD_T1_Config( void )
{
    TIMIF |= 0x40;
    IRCON &= ~0x20;  //Clear T1 interrupt flag
    T1STAT &= ~0x04; //Clear T1 Channel 2 interrupt flag

    //P2SEL |= 0x18;  //T1 Highest Priority

    T1IE = 1;        //T1 INTEN

    T1CC0L = 0x08;
    T1CC0H = 0x52;   //Set T1CC0=21ms,If Tick.freq=1MHz;
    //T1CC0L = 0xFF;
    //T1CC0H = 0xFF;   //Set T1CC0=16ms,If Tick.freq=4MHz;
    T1CCTL0 |= 0x44; //Set T1.CH0 INTEN,Compare Mode

    T1CTL = 0x08;    //Tick frequency/32,No Operating Mode
    //T1CTL = 0x04;
    T1CTL = ( T1CTL & ~0x01 ) | 0x02;  //Start T1 on Modulo
}

void UartTX_Send_String(unsigned short *Data,int len)
{
  int j;
  for(j=0;j<len;j++)
  {
    U0DBUF = (*Data)>>8;
    while(UTX0IF == 0);
    UTX0IF = 0;

    U0DBUF = *Data++;
    while(UTX0IF == 0);
    UTX0IF = 0;
  }
}

extern unsigned short IR_Encoding[];
extern int T1_CNT1;
HAL_ISR_FUNCTION( hal_IR_RCVD_Isr, P0INT_VECTOR )
{
    if ( P0IFG & 0x10 )  //If P0.4 Interrupt
    {
        //PERCFG &= ~0x40; //P0 Alternative 0
        //P0SEL |= 0x10;  //P0.4 Peripheral

        //T1CCTL2 = 0x43;  //CH2,INTEN,Capture Mode on all Edges
        //T1CTL = ( T1CTL & ~0x01 ) | 0x02;  //Start T1 on Modulo

        //P0IFG = 0;       //clear P0 interrupt mask
        //P0IF = 0;        //clear P0 interrupt mask
        //Hal_IR_RCVD_Int_Config( HAL_IR_INTERRUPT_DISABLE );  //Disable IR_INT

        IR_Encoding[T1_CNT1] = T1CNTL;
        IR_Encoding[T1_CNT1++] += T1CNTH*0x100;
        T1CNTL = 0x00;        //Clear T1.CNT

        P0IFG = 0;       //clear P0 interrupt mask
        P0IF = 0;        //clear P0 interrupt mask
        PICTL ^= 0x01;   //PICTL_0 Flip Flop
        
        P1_0=!P1_0;
        P1_1=!P1_1;
    }
}

void Hal_IR_RCVD_Init( void )
{
    Hal_IR_RCVD_Int_Config( HAL_IR_INTERRUPT_ENABLE );
    Hal_IR_RCVD_T1_Config();
}

#endif
