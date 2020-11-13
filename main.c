#include <stdio.h>
#include "msp432p401r.h"

void set_up_clock(void);
void Port_1_init(void);
void set_up_timer_A(void);


float timer_ms = 0;
float total_time = 0;

#define TRIGGER BIT6
#define ECHO    BIT7

void UART_Init(void){
  UCA0CTLW0 = 0x0001;                   // hold the USCI module in reset mode
  // bit15=0,      no parity bits
  // bit14=x,      not used when parity is disabled
  // bit13=0,      LSB first
  // bit12=0,      8-bit data length
  // bit11=0,      1 stop bit
  // bits10-8=000, asynchronous UART mode
  // bits7-6=11,   clock source to SMCLK
  // bit5=0,       reject erroneous characters and do not set flag
  // bit4=0,       do not set flag for break characters
  // bit3=0,       not dormant
  // bit2=0,       transmit data, not address (not used here)
  // bit1=0,       do not transmit break (not used here)
  // bit0=1,       hold logic in reset state while configuring
  UCA0CTLW0 = 0x00C1;
                                        // set the baud rate
                                        // N = clock/baud rate = 3,000,000/115,200 = 26.0417
  UCA0BRW = 13;                         // UCBR = baud rate = int(N) = 26
  UCA0MCTLW &= ~0xFFF1;                 // clear first and second modulation stage bit fields
                                        // configure second modulation stage select (from Table 22-4 on p731 of datasheet)
//  UCA0MCTLW |= (0<<8);                  // UCBRS = N - int(N) = 0.0417; plug this in Table 22-4
                                        // configure first modulation stage select (ignored when oversampling disabled)
//  UCA0MCTLW |= (10<<4);                 // UCBRF = int(((N/16) - int(N/16))*16) = 10
//  UCA0MCTLW |= 0x0001;                  // enable oversampling mode
  P1SEL0 |= 0x0C;
  P1SEL1 &= ~0x0C;                      // configure P1.3 and P1.2 as primary module function
  UCA0CTLW0 &= ~0x0001;                 // enable the USCI module
  UCA0IE &= ~0x000F;                    // disable interrupts (transmit ready, start received, transmit empty, receive full)
}

//------------UART_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
char UART_InChar(void){
  while((UCA0IFG&0x01) == 0);
  return((char)(UCA0RXBUF));
}

//------------UART_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART_OutChar(char data){
  while((UCA0IFG&0x02) == 0);
  UCA0TXBUF = data;
}

//------------UART_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART_OutString(char *pt){
  while(*pt){
    UART_OutChar(*pt);
    pt++;
  }
}

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	// set up
	set_up_clock();
	Port_1_init();
	set_up_timer_A();
	UART_Init();

	// Enable port 1 interrupt on NVIC
	NVIC->ISER[1] |= 1 << ((PORT1_IRQn) & 31); // <- Port 1
	NVIC->ISER[0] |= 1 << ((TA0_0_IRQn) & 31);  // <- Timer A0

	float distance = 0;
	char data[10] = "Hello\r\n";
	__enable_irq(); // enable interrupts

	while(1)
	{
	    P1->OUT |= TRIGGER;  // set trigger

	    __delay_cycles(20); // 10us delay

	    P1->OUT &= ~TRIGGER;    // make trigger low

	    P1->IES &= ~ECHO; // rise low-high transition
	    P1->IFG = 0;
	    P1->IE |= ECHO;  // enable interrupt for ECHO

	    __delay_cycles(50000);  // wait 30 ms

	    P1->IE &= ~ECHO;  // disable interrupt for ECHO

	    distance = (total_time/2)*0.0343;   // convert to cm
	    sprintf(data, "%d\r\n", (int)distance);

	    UART_OutString(data);

	    if(distance > 300)   // greater than 20 cm
	    {
	        P1->OUT |= BIT0;
	    }
	    else
	    {
	        P1->OUT &= ~BIT0;
	    }

	}
}

void PORT1_IRQHandler(void)
{
    if(P1->IFG & ECHO)  // if interrupt occured from ECHO
    {
        // rising edge
        // echo pulse received
        if(!(P1->IES & ECHO))
        {
            TIMER_A0->CTL |= TIMER_A_CTL_CLR;   // clear timer A_0
            timer_ms = 0;   // reset count
            P1->IES |= ECHO; // set for falling edge
        }
        else    // falling edge
        {
            total_time = timer_ms*1000; // convert to us
        }
        // clear IFG bit for correspnding bit
        P1->IFG &= ~ECHO;
    }
}

void TA0_0_IRQHandler(void)
{
    // Increment by 1ms the total
    timer_ms++;
    // clear interrupt flag
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
}

void set_up_clock(void)
{
    // enable clock registers to be modified
    CS->KEY = CS_KEY_VAL;
    // frequency select of DCO
    CS->CTL0 = CS_CTL0_DCORSEL_0;
    // set
    CS->CTL1 = CS_CTL1_SELM_3 | CS_CTL1_SELS_3 | CS_CTL1_DIVM_0 | CS_CTL1_DIVS_0;
    // lock clock
    CS->KEY = 0;
}

void Port_1_init(void)
{
    // make TRIGGER output
    // make P1.0 output
    // make ECHO input
    P1->DIR |= BIT0 | TRIGGER;
    P1->DIR &= ~ECHO;
}

void set_up_timer_A(void)
{
    // set count to 1499
    TIMER_A0->CCR[0] = 1499;    // 1ms at 1.5MHz
    // UP mode
    // /1 divider
    // SMCLK
    // clear TAR
    // enable interrupt
    TIMER_A0->CTL = TIMER_A_CTL_MC_1 | TIMER_A_CTL_ID_0 | TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_CLR;
    // enable interrupt in Timer A0
    TIMER_A0->CCTL[0] |= TIMER_A_CCTLN_CCIE;
}
