/*
* nunchuckRemoteC.c
*
* Created: 7-11-2017 22:54:24
* Author : Steven
*/
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include <avr/interrupt.h>

//uart
#define  FOSC 16000000 //16mhz
#define  BAUD 9600
#define  MYUBBR FOSC/16/BAUD -1


//utils
#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))
#define BIT(x) (0x01 << (x))
#define LONGBIT(x) ((unsigned long)0x00000001 << (x))

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

volatile unsigned long milliseconds = 0,previousMillis = 0, currentMillis = 0, disconnectTime = 2000;
volatile bool startByteBool = false, leftByteBool = false, rightByteBool = false, CKSMByteBool = false, armState = false;
volatile uint8_t leftByte,rightByte,CKSMByte;


// uint8_t startByte, leftByte, rightByte, CKSMByte;

void USART_Init(void);
void USART_Send(uint8_t *cpubyData, const uint8_t cubyLength);
void USART_Transmit( uint8_t data );
void USART_sendString(char* str);
void USART_line();
void USART_Printf(const char* format, ...);
long map(long x, long in_min, long in_max, long out_min, long out_max);
unsigned long millis();



int main(void)
{
	
	/* Replace with your application code */
	
	USART_Init();
	
	DDRB |= 0xFF;
	TCCR1A |= 1<<WGM11 | 1<<COM1A1 | 1<<COM1A0 | 1<<COM1B1 | 1<<COM1B0;
	TCCR1B |= 1<<WGM13 | 1<<WGM12 | 1<<CS11;
	ICR1 = 39999;
	
	TCCR0A |= 1<<WGM01;
	TCCR0B |= 1<<CS01 | 1<<CS00;
	TIMSK0 |= 1<<OCIE0A;
	OCR0A = 249;
	OCR1A = ICR1 - 2000;
	OCR1B = ICR1 - 2000;
	sei();
	
	
	while (1)
	{
		cli();
		currentMillis = millis();		
		if ((currentMillis - previousMillis) > disconnectTime) //watchdog
		{
			OCR1A = ICR1 - 2000;
			OCR1B = ICR1 - 2000;
			armState = false;
		}
		sei();
		
		
		/*
		currentMillis = millis();
		if ( bit_get (UCSR0A, BIT(RXC0)) ) //UART received
		{
			previousMillis = millis();
			startByte = UDR0;
			if (startByte == 0xFA)
			{
				while( !(bit_get( UCSR0A, BIT(RXC0))) ){ //wait for UART
				}
				leftByte = UDR0;
				while( !(bit_get( UCSR0A, BIT(RXC0))) ){ //wait for UART
				}
				rightByte = UDR0;
				while( !(bit_get( UCSR0A, BIT(RXC0))) ){ //wait for UART
				}
				CKSMByte = UDR0;
				if ((leftByte == 0xFB) && (rightByte == 0xFB))
				{
					armState = true;
				}
				else if ((leftByte == 0xFC) && (rightByte == 0xFC))
				{
					armState = false;
				}
				else if (((leftByte ^ rightByte) == CKSMByte) && armState)
				{
					OCR1A = ICR1 - map(constrain(leftByte,0,100),0,100,2000,4000);
					OCR1B = ICR1 - map(constrain(rightByte,0,100),0,100,2000,4000);
				}
				else if (!armState)
				{
					OCR1A = ICR1 - 2000;
					OCR1B = ICR1 - 2000;
				}
			}
		}
		else if (currentMillis - previousMillis > disconnectTime)
		{
			OCR1A = ICR1 - 2000;
			OCR1B = ICR1 - 2000;
			armState = false;
		}
		*/
	}
	
}

void USART_Init(void)
{
	// Set baud rate:
	UBRR0=MYUBBR;                 //UBRR= Fosc /(16*9600) -1 =103.166= 103

	// enable receiver and transmitter
	bit_set(UCSR0B, BIT(RXEN0)|BIT(TXEN0));

	// Set frame format : 8 data 1 stop bit
	UCSR0C = BIT(UCSZ00)|BIT(UCSZ01);
	UCSR0B |= BIT(RXCIE0); //interrupt enable
}

void USART_Send(uint8_t *cpubyData, const uint8_t cubyLength)
{
	uint8_t ubyDataIndex;
	for (ubyDataIndex = 0; ubyDataIndex < cubyLength; ubyDataIndex++)
	{
		while ( !( UCSR0A & (1<<UDRE0)) );
		/* Put data into buffer, sends the data */
		UDR0 = cpubyData[ubyDataIndex];
	}
}

void USART_Transmit( uint8_t data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
}



void USART_Printf(const char* format, ...)
{
	char dest[64];
	
	va_list argptr;
	va_start(argptr, format);
	vsnprintf(dest, 64, format, argptr);
	va_end(argptr);
	
	USART_sendString(dest);
}

void USART_sendString(char* str)
{
	while (*str)									//loop though array until NULL
	{
		USART_Transmit(*str++);
	}
}
void USART_line(){								//Send /r/n
	USART_Transmit(0x0D);
	USART_Transmit(0x0A);
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
ISR(TIMER0_COMPA_vect){
	++milliseconds;
}

ISR(USART_RX_vect){
	
	previousMillis = millis(); //Reset watchdog
	uint8_t rxByte = UDR0; //read incoming byte
	if (rxByte == 0xFA)  //startbyte
	{
		startByteBool = true;
	}
	else if (startByteBool && !leftByteBool && !rightByteBool)
	{
		leftByteBool = true;
		leftByte = rxByte;  //read in leftbyte
	}
	else if (startByteBool && leftByteBool && !rightByteBool)
	{
		rightByteBool = true;
		rightByte = rxByte; //read in rightbyte
	}
	else if (startByteBool && leftByteBool && rightByteBool)
	{
		if ((leftByte == 0xFB) && (rightByte == 0xFB)) //arm condition
		{
			armState = true;
			
			startByteBool = false;
			leftByteBool = false;  //setup for new packet
			rightByteBool = false;
		}
		else if ((leftByte == 0xFC) && (rightByte == 0xFC)) //disarm condition
		{
			armState = false;
			
			startByteBool = false;
			leftByteBool = false;  //setup for new packet
			rightByteBool = false;
		}
		else if (((leftByte ^ rightByte) == rxByte) && armState) //checksum and arm condition
		{
			OCR1A = ICR1 - map(leftByte,0,100,2000,4000);
			OCR1B = ICR1 - map(rightByte,0,100,2000,4000);
			
			startByteBool = false;
			leftByteBool = false;  //setup for new packet
			rightByteBool = false;			
		}
		else if (!armState) //not armed = low throttle
		{
			OCR1A = ICR1 - 2000;
			OCR1B = ICR1 - 2000;
			
			startByteBool = false;			
			leftByteBool = false;  //setup for new packet
			rightByteBool = false;	
		}
		else{
			startByteBool = false; //reset if garbage received
			leftByteBool = false;
			rightByteBool = false;
		}
	}
}

unsigned long millis(){
	return milliseconds;
}
