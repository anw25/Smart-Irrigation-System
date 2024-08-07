#include "TM4C123GH6PM.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
void delayms(int n)
{
    volatile int i, j; // volatile is important for variables incremented in code
    for (i = 0; i < n; i++)
        for (j = 0; j < 3180; j++) // delay for 1 msec
        {
        }
}

void init_portB()

{
    SYSCTL->RCGCGPIO |= (1);
    GPIOA->DEN |= 1<<2;
    GPIOA->DIR |= 1<<2;
    GPIOA->PUR |= 0x00;
	
	GPIOA->AMSEL &= ~(1<<2);
	GPIOA->AFSEL &= ~(1<<2);
	

}
void init_ports(){
	SYSCTL->RCGCGPIO |=(1<<5);
	GPIOF->DEN |=(1<<1) |(1<<2)|(1<<3);
	GPIOF->DIR |=(1<<1) |(1<<2)|(1<<3);
}
	
	
void uart_printstring(const char *string) {
    while (*string != '\0') {
        while (UART0->FR & (1 << 5)); // Wait until UART is not busy
        UART0->DR = *string; // Send the character
        string++; // Move to the next character in the string
    }
}

void  uart_init(){
	uint32_t time; /*stores pulse on time */
	uint32_t distance; /* stores measured distance value */
	char mesg[20];
	SYSCTL->RCGCUART |= (1<<0);
	SYSCTL->RCGCGPIO |= (1<<0);
	GPIOA->AFSEL |= (1<<0) | (1<<1);
	GPIOA->PCTL |= (1<<0) | (1<<4);
	GPIOA->DEN |= (1<<0) | (1<<1);

	UART0->CTL &= ~((1<<9)|(1<<8)|(1<<0));
	UART0->IBRD =104;
	UART0->FBRD=11;
	UART0->LCRH = (1<<6)| (1<<5);
	UART0->CC =0X5;
	UART0->CTL |= (1<<9)|(1<<8)|(1<<0);

    SYSCTL->RCGCGPIO = 0x20; 
	GPIOF->DEN = 0x0E;
	GPIOF->DIR = 0x0E;
	
}
void Delay_MicroSecond(int time);
uint32_t Measure_distance(void)
{
    int lastEdge, thisEdge;
	
	  /* Given 10us trigger pulse */
	  GPIOA->DATA &= ~(1<<4); /* make trigger  pin high */
	  Delay_MicroSecond(10); /*10 seconds delay */
	  GPIOA->DATA |= (1<<4); /* make trigger  pin high */
	  Delay_MicroSecond(10); /*10 seconds delay */
	  GPIOA->DATA &= ~(1<<4); /* make trigger  pin low */

 	while(1)
	{
    TIMER0->ICR = 4;            /* clear timer0A capture flag */
    while((TIMER0->RIS & 4) == 0) ;    /* wait till captured */
	  if(GPIOB->DATA & (1<<6)) /*check if rising edge occurs */
		{
    lastEdge = TIMER0->TAR;     /* save the timestamp */
		/* detect falling edge */
    TIMER0->ICR = 4;            /* clear timer0A capture flag */
    while((TIMER0->RIS & 4) == 0) ;    /* wait till captured */
    thisEdge = TIMER0->TAR;     /* save the timestamp */
		return (thisEdge - lastEdge); /* return the time difference */
		}
	}
}

void Timer0ACapture_init(void)
{
    SYSCTL->RCGCTIMER |= 1;     /* enable clock to Timer Block 0 */
    SYSCTL->RCGCGPIO |= 2;      /* enable clock to PORTB */
    
    GPIOB->DIR &= ~0x40;        /* make PB6 an input pin */
    GPIOB->DEN |= 0x40;         /* make PB6 as digital pin */
    GPIOB->AFSEL |= 0x40;       /* use PB6 alternate function */
    GPIOB->PCTL &= ~0x0F000000;  /* configure PB6 for T0CCP0 */
    GPIOB->PCTL |= 0x07000000;
    
	  /* PA4 as a digital output signal to provide trigger signal */
	  SYSCTL->RCGCGPIO |= 1;      /* enable clock to PORTA */
	  GPIOA->DIR |=(1<<4);         /* set PA4 as a digial output pin */
	  GPIOA->DEN |=(1<<4);         /* make PA4 as digital pin */

    TIMER0->CTL &= ~1;          /* disable timer0A during setup */
    TIMER0->CFG = 4;            /* 16-bit timer mode */
    TIMER0->TAMR = 0x17;        /* up-count, edge-time, capture mode */
    TIMER0->CTL |=0x0C;        /* capture the rising edge */
    TIMER0->CTL |= (1<<0);           /* enable timer0A */
}




void Delay_MicroSecond(int Time)
{
    int i;
    SYSCTL->RCGCTIMER |= 2;     /* enable clock to Timer Block 1 */
    TIMER1->CTL = 0;            /* disable Timer before initialization */
    TIMER1->CFG = 0x04;         /* 16-bit option */ 
    TIMER1->TAMR = 0x02;        /* periodic mode and down-counter */
    TIMER1->TAILR = 16 - 1;  /* TimerA interval load value reg */
    TIMER1->ICR = 0x1;          /* clear the TimerA timeout flag */
    TIMER1->CTL |= 0x01;        /* enable Timer A after initialization */

    for(i = 0; i < Time; i++)
    {
        while ((TIMER1->RIS & 0x1) == 0) ;      /* wait for TimerA timeout flag */
        TIMER1->ICR = 0x1;      /* clear the TimerA timeout flag */
    }
}

void Delay(unsigned long counter)
{
	unsigned long i = 0;
	
	for(i=0; i< counter*1000; i++);
}

void delay(int n){
	while(n--);
}

void transmitUART(uint32_t value) {
    char buffer[10];
    sprintf(buffer, " %d ", value,"\n");

    for (int i = 0; buffer[i] != '\0'; ++i) {
        while ((UART0->FR & (1 << 5)) != 0);
        UART0->DR = buffer[i];
    }
}
void adc_init()
{
	//ADC Initialization
	
	SYSCTL->RCGCGPIO |= 0x10;
	delayms(10); 
	SYSCTL->RCGCADC |= 0x01;

	/* initialize PE3 for AN0 input  */
    GPIOE->AFSEL |= (1 << 3);       /* enable alternate function for AN0 (PE3) */
    GPIOE->DEN &= ~(1 << 3);        /* disable digital function for AN0 (PE3) */
    GPIOE->AMSEL |= (1 << 3);       /* enable analog function for AN0 (PE3) */

    /* initialize PE2 for AN1 input  */
    GPIOE->AFSEL |= (1 << 2);       /* enable alternate function for AN1 (PE2) */
    GPIOE->DEN &= ~(1 << 2);        /* disable digital function for AN1 (PE2) */
    GPIOE->AMSEL |= (1 << 2);       /* enable analog function for AN1 (PE2) */

	 /* initialize sample sequencer3 for AN0 */
    ADC0->ACTSS &= ~(1 << 3);                       /* disable SS3 during configuration */
    ADC0->EMUX &= ~0xF000;                          /* software trigger conversion */
    ADC0->SSMUX3 = 0;                               /* get input from channel 0 (AN0) */
    ADC0->SSCTL3 |= (1 << 1) | (1 << 2);            /* take one sample at a time, set flag at 1st sample */
    ADC0->ACTSS |= (1 << 3);                        /* enable ADC0 sequencer 3 */

    /* initialize sample sequencer2 for AN1 */
    ADC0->ACTSS &= ~(1 << 2);                       /* disable SS2 during configuration */
    ADC0->SSMUX2 = 1;                               /* get input from channel 1 (AN1) */
    ADC0->SSCTL2 |= (1 << 1) | (1 << 2);            /* take one sample at a time, set flag at 1st sample */
    ADC0->ACTSS |= (1 << 2);                        /* enable ADC0 sequencer 2 */

}






uint32_t time; /*stores pulse on time */
uint32_t distance; /* stores measured distance value */
char mesg[20];  /* string format of distance value */
volatile uint32_t adc_value;


























int main(void)
{
adc_init();
init_ports();
init_portB();
uart_init();


Timer0ACapture_init();  /*initialize Timer0A in edge edge time */
uint8_t data;
unsigned int  adc_value2,temperature;    //variables that contain adc values

while(1)
{
time = Measure_distance(); /* take pulse duration measurement */ 
distance = (time * 10625)/10000000; /* convert pulse duration into distance */


ADC0->PSSI = 0x8;
while((ADC0->RIS & 0x8) == 0x0);
adc_value = ADC0->SSFIFO3;
ADC0->ISC = 8;                              /* Clear conversion clear flag bit */

/* Read from AN1 (PE2) */
ADC0->PSSI |= (1 << 2);                     /* Enable SS2 conversion or start sampling data from AN1 */
while ((ADC0->RIS & 4) == 0) ;              /* Wait until sample conversion completed */
adc_value2 = ADC0->SSFIFO2;                 /* Read ADC conversion result from SS2 FIFO */
temperature = (int)(((adc_value * 33) / 4096) + 1);

ADC0->ISC = 4;                              /* Clear conversion clear flag bit */


if(distance<100){
    GPIOA->DATA |= (1<<2);
}
else{
    GPIOA->DATA &= !(1<<2);
}

if(adc_value > 2900)
		{
			GPIOF->DATA = 1<<2;
			delay(1000000);
		}
		
else
    {
        GPIOF->DATA = 1<<3;
        delay(1000000);

    }
ADC0->ISC = 0x8;
while((UART0->FR & 0x20) != 0);

while(UART0->FR & (1<<4) !=0);
data=UART0->DR;
int percentage = (int)(4096-adc_value)*0.588  ;
if (data=='m'){
	sprintf(mesg, "\r\nMoisture Content  = %d percent ", percentage); /*convert float type distance data into string */
	uart_printstring(mesg); 
}
else if (data=='p'){
	sprintf(mesg, "\r\nDistance = %d cm", distance); /*convert float type distance data into string */
	uart_printstring(mesg); /*transmit data to computer */
}
else if (data=='t'){
	sprintf(mesg, "\r\nTemperature = %d C", temperature); /*convert float type distance data into string */
	uart_printstring(mesg); /*transmit data to computer */
}
Delay(2000);

	}
	
}