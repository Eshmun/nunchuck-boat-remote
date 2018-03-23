/*
* nunchuckRemoteC.c
*
* Created: 7-11-2017 22:54:24
* Author : Steven
*/
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/twi.h>
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

//I2C
#define F_SCL 100000UL //100khz
#define SendACK 0
#define SendNACK 1
#define read 1
#define write 0
#define Start 0
#define RepeatStart 1

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

typedef struct{
	uint8_t ubyStart;
	uint8_t ubyLeft;
	uint8_t ubyRight;
	uint8_t ubyCKSM;
}ST_DATA;
typedef struct
{
	uint8_t joyX;
	uint8_t joyY;
	uint8_t buttonZ;
	uint8_t buttonC;
}NUN_DATA;

volatile unsigned long milliseconds = 0;

void USART_Init(void);
void USART_Send(uint8_t *cpubyData, const uint8_t cubyLength);
void USART_Transmit( uint8_t data );
void USART_sendString(char* str);
void USART_line();
void USART_Printf(const char* format, ...);
void TWI_init_master(void);
void TWI_start(uint8_t StartMode);
void TWI_address(uint8_t SlaveAdress, uint8_t Direction);
void TWI_write_data(uint8_t data);
uint8_t TWI_read_data(uint8_t ReceiveMode);
void TWI_stop(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);
NUN_DATA nunchuck_get();
void arm();
void disarm();
unsigned long millis();

unsigned long previousMillis;

int main(void)
{
	
	/* Replace with your application code */
	TWI_init_master();
	USART_Init();
	USART_sendString("Started");
	USART_line();
	TWI_start(Start);
	TWI_address(0x52, write);
	TWI_write_data(0x40);
	TWI_write_data(0x00);
	TWI_stop();
	_delay_ms(1);
	
	DDRB |= 0xFF;
	TCCR1A |= 1<<WGM11 | 1<<COM1A1 | 1<<COM1A0 | 1<<COM1B1 | 1<<COM1B0;
	TCCR1B |= 1<<WGM13 | 1<<WGM12 | 1<<CS11;
	ICR1 = 39999;
	
	TCCR0A |= 1<<WGM01;
	TCCR0B |= 1<<CS01 | 1<<CS00;
	TIMSK0 |= 1<<OCIE0A;
	OCR0A = 249;
	
	sei();
	
	ST_DATA stData;
	stData.ubyStart = 0xFA;
	
	NUN_DATA nunData;
	bool calibrate = true,armed = false;
	uint8_t minX,minY,maxX,maxY;
	uint16_t armTime = 2000;
	
	while (1)
	{
		//Calibration
		nunData = nunchuck_get();
		while(!nunData.buttonZ && calibrate){
			nunData = nunchuck_get();
		}
		if (nunData.buttonZ && calibrate) {
			minY = nunData.joyY;
			USART_sendString("minY");
			USART_line();
			
			
			_delay_ms(20);
			while(nunData.buttonZ){
				nunData = nunchuck_get();
			}
			_delay_ms(20);
			while(!nunData.buttonZ){
				nunData = nunchuck_get();
			}
			maxY = nunData.joyY;
			USART_sendString("maxY");
			USART_line();
			
			
			_delay_ms(20);
			while(nunData.buttonZ){
				nunData = nunchuck_get();
			}
			_delay_ms(20);
			while(!nunData.buttonZ){
				nunData = nunchuck_get();
			}
			minX = nunData.joyX;
			USART_sendString("minX");
			USART_line();
			
			
			_delay_ms(20);
			while(nunData.buttonZ){
				nunData = nunchuck_get();
			}
			_delay_ms(20);
			while(!nunData.buttonZ){
				nunData = nunchuck_get();
			}
			maxX = nunData.joyX;
			USART_sendString("maxX");
			USART_line();
			calibrate = false;
			
			_delay_ms(20);
			while(nunData.buttonZ){
				nunData = nunchuck_get();
			}
			_delay_ms(20);
		}
		//end calibration
		
		uint8_t rawjoyX = constrain(nunData.joyX, minX, maxX);
		uint8_t rawjoyY = constrain(nunData.joyY, minY, maxY);
		
		float joyX = map(rawjoyX, minX, maxX, -100, 100);
		float joyY = map(rawjoyY, minY, maxY, 0, 100);
		
		joyX = joyX / 100;
		joyY = joyY / 100;
		
		float r = sqrtf((joyX * joyX) + (joyY * joyY));
		float t = atan2f(joyY, joyX);
		
		t -= (3.1415f / 4.0f);

		float left = r * cosf(t);
		float right = r * sinf(t);

		left = constrain((left * 1.414f * 100),0,100);
		right = constrain((right * 1.414f * 100),0,100);;
		
		stData.ubyLeft = (uint8_t)left;
		stData.ubyRight = (uint8_t)right;
		stData.ubyCKSM = (uint8_t)left ^ (uint8_t)right;
		
		USART_Send((uint8_t*)&stData, sizeof(ST_DATA));
		
		OCR1A = ICR1 - map((uint8_t)left,0,100,1800,4000);
		OCR1B = ICR1 - map((uint8_t)right,0,100,1800,4000);
		
		if (nunData.buttonC)
		{
			unsigned long currentMillis = millis();
			if(currentMillis - previousMillis > armTime) {
				previousMillis = currentMillis;
				if (armed)
				{
					arm();
				}
				else{
					disarm();
				}
				armed = !armed;
			}
		}
		else{
			previousMillis = millis();
		}
	}
}
void arm(){
	USART_Transmit(0xFA);
	USART_Transmit(0xFB);
	USART_Transmit(0xFB);
	USART_Transmit(0xFB^0xFB);
	
}
void disarm(){
	USART_Transmit(0xFA);
	USART_Transmit(0xFC);
	USART_Transmit(0xFC);
	USART_Transmit(0xFC^0xFC);
}
NUN_DATA nunchuck_get(){
	NUN_DATA nunData;
	TWI_start(Start);
	TWI_address(0x52,write);
	TWI_write_data(0x00);
	TWI_stop();
	_delay_ms(1);
	TWI_start(Start);
	TWI_address(0x52,read);
	nunData.joyX = (TWI_read_data(SendACK)^0x17)+0x17;
	nunData.joyY = (TWI_read_data(SendACK)^0x17)+0x17;
	(void)TWI_read_data(SendACK);
	(void)TWI_read_data(SendACK);
	(void)TWI_read_data(SendACK);
	uint8_t rawButtons = (TWI_read_data(SendNACK)^0x17)+0x17;
	TWI_stop();
	_delay_ms(1);
	nunData.buttonC = ((rawButtons & (2) ) != (2));
	nunData.buttonZ = ((rawButtons & (1) ) != (1));
	return nunData;
}
void USART_Init(void)
{
	// Set baud rate:
	UBRR0=MYUBBR;                 //UBRR= Fosc /(16*9600) -1 =103.166= 103

	// enable receiver and transmitter
	bit_set(UCSR0B, BIT(RXEN0)|BIT(TXEN0));

	// Set frame format : 8 data 1 stop bit
	UCSR0C = BIT(UCSZ00)|BIT(UCSZ01);
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
void TWI_init_master(void){										//Function to initialize master
	TWSR &= ~(1 << TWPS1) & ~(1 << TWPS0);						//Setting prescaler to 1
	TWBR = ((F_CPU / F_SCL) - 16)/(2 * 1);						//Bit rate
}

void TWI_start(uint8_t StartMode){
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);			//Clear TWI interrupt flag, Put start condition on SDA, Enable TWI
	while(!(TWCR & (1 << TWINT)));								//Wait till start condition is transmitted
	if(StartMode == Start)
	while((TWSR & 0xF8) != TW_START);						//Check for the acknowledgement
	else
	while((TWSR & 0xF8) != TW_REP_START);					//Check for the acknowledgement
}

void TWI_address(uint8_t SlaveAdress, uint8_t Direction){
	SlaveAdress = (SlaveAdress << 1) | Direction;
	TWDR = SlaveAdress;											//Address and read/write instruction
	TWCR = (1 << TWINT) | (1 << TWEN);							//Clear TWI interrupt flag,Enable TWI
	while(!(TWCR & (1 << TWINT)));								//Wait till complete TWDR byte transmitted
	if(Direction == write)
	while((TWSR & 0xF8)!= TW_MT_SLA_ACK);					//Check for the acknowledgement: Master Transmitter, SLA+W send ACK received
	else
	while((TWSR & 0xF8) != TW_MR_SLA_ACK);					//Check for the acknowledgement: Master Receiver, SLA+R send ACK received
}

void TWI_write_data(uint8_t data){
	TWDR = data;												//Put data in TWDR
	TWCR = (1 << TWINT) | (1 << TWEN);							//Clear TWI interrupt flag,Enable TWI
	while(!(TWCR & (1 << TWINT)));								//Wait till complete TWDR byte transmitted
	while((TWSR & 0xF8) != TW_MT_DATA_ACK);						//Check for the acknowledgement: Master Transmitter, Data send ACK received
}

uint8_t TWI_read_data(uint8_t ReceiveMode){
	TWCR = (1 << TWINT) | (1 << TWEN);
	if(ReceiveMode == SendACK)
	TWCR |= (1 << TWEA);
	while(!(TWCR & (1 << TWINT)));								//Wait till complete TWDR byte transmitted
	if(ReceiveMode == SendACK)
	while((TWSR & 0xF8) != TW_MR_DATA_ACK);					//Check for the acknowledgement: Master Receiver, Data Received ACK returned
	else
	while((TWSR & 0xF8) != TW_MR_DATA_NACK);				//Check for the acknowledgement: Master Receiver, Data Received NACK returned
	return TWDR;
}

void TWI_stop(void){
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);			//Clear TWI interrupt flag, Put stop condition on SDA, Enable TWI
	while(!(TWCR & (1 << TWSTO)));								//Wait till stop condition is transmitted
}
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
ISR(TIMER0_COMPA_vect){
	++milliseconds;
}
unsigned long millis(){
	return milliseconds;
}
