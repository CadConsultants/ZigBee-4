PERCFG &= ~0x40; // Select Timer 1 Alternative 0 location
P2DIR = (P2DIR & ~0xC0) | 0x80; // Give priority to Timer 1
P0SEL |= 0x08;  // Set P0_3 to peripheral

T1CC0L = 0xff;   // PWM signal period
T1CC0H = 0x7f;

T1CC1L = 0x78;  // PWM duty cycle
T1CC1H = 0x10;

T1CCTL1 = 0x1c;

T1CTL |= 0x0f; // divide with 128 and to do i up-down mode
