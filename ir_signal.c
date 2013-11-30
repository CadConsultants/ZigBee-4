#include <ioCC2530.h>

#define uchar unsigned char

void main(void)
{
    PERCFG &= ~0x20;
    P1SEL |= 0x10;
    P2DIR = (P2DIR & ~0xC0) | 0x80; 

    T3CCTL1 = 0x2c;
    T3CC1 = 70;
    T3CTL = (T3CTL & 0x5f) | 0x40; //divide with 4
    T3CTL &= ~0x03; //free mode

    T3CTL |= 0x10; //start T3

    IRCTL |= 0x01;
}
