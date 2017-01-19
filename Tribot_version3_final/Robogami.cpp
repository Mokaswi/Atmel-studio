/*
* This file provides some function for writing pins, dealing Tribot's role and id and defines locomotion's pin, duty and during
* you might have to change these pin duty and during.
*/
//
//include
//
#include <avr/io.h>
#include "wiring.h"
#include "Robogami.h"
//
//Global variables
//
volatile uint8_t Duty_Torsional_L = 0;
volatile uint8_t Counter_Torsional_L = 0;
volatile uint8_t Duty_Torsional_R = 0;
volatile uint8_t Counter_Torsional_R = 0;
//
//PROGMEM
//these are used in locomotion.
//if you want to change preset duty, during or pin, you should change there
//units of during is 100ms so if you set a during 50 it means 5000ms
//Maximum value of during and duty is 255.
//but duty for tor 
//////////////////////////////////////////////////////////////////////You might have to change from here////////////////////////////////////////////////////////////////////////
const uint8_t Tasks::CRAWL_FORWARD_MANUAL[] PROGMEM = {
	Pins::TORSIONAL_R,
	Pins::SPRING_B,
	Pins::TORSIONAL_R,
	Pins::SPRING_B,
	Pins::TORSIONAL_L,
	Pins::SPRING_L,
	Pins::SPRING_R,
	Pins::TORSIONAL_L,
	Pins::END
};
const uint8_t Tasks::CRAWL_FORWARD_AUTO[][3] PROGMEM = {
	//Displacement of each step is 5mm? 
	#if ID == 1
	{Pins::TORSIONAL_R, 6, 50},
	{Pins::SPRING_B, 33, 20},
	{Pins::TORSIONAL_R, 0, 15},
	{Pins::SPRING_B, 0, 30},
	{Pins::TORSIONAL_L, 4, 50},
	{Pins::SPRING_L, 50, 0},
	{Pins::SPRING_R, 25, 20},
	{Pins::TORSIONAL_L, 0, 15},
	#elif ID == 2//ok
	/*for crawling
	{Pins::TORSIONAL_R, 4, 50},
	{Pins::SPRING_B, 45, 20},
	{Pins::TORSIONAL_R, 0, 15},
	{Pins::SPRING_B, 0, 30},
	{Pins::TORSIONAL_L, 4, 50},
	{Pins::SPRING_L, 30, 0},
	{Pins::SPRING_R, 30, 10},
	{Pins::TORSIONAL_L, 0, 25},
	*/
	///*for pushing
	{Pins::TORSIONAL_R, 4, 50},
	{Pins::SPRING_B, 30, 10},
	{Pins::TORSIONAL_R, 0, 25},
	{Pins::SPRING_B, 0, 30},
	{Pins::TORSIONAL_L, 4, 50},
	{Pins::SPRING_L, 22, 0},
	{Pins::SPRING_R, 17, 10},
	{Pins::TORSIONAL_L, 0, 25},
	#elif ID == 3
	{Pins::TORSIONAL_R, 4, 50},
	{Pins::SPRING_B, 40, 10},
	{Pins::TORSIONAL_R, 0, 25},
	{Pins::SPRING_B, 0, 30},
	{Pins::TORSIONAL_L, 4, 50},
	{Pins::SPRING_L, 55, 0},
	{Pins::SPRING_R, 55, 10},
	{Pins::TORSIONAL_L, 0, 25},
	#elif ID == 4
	/*for crawling
	{Pins::TORSIONAL_R, 4, 50},
	{Pins::SPRING_B, 28, 20},
	{Pins::TORSIONAL_R, 0, 15},
	{Pins::SPRING_B, 0, 30},
	{Pins::TORSIONAL_L, 4, 50},
	{Pins::SPRING_L, 28, 0},
	{Pins::SPRING_R, 18, 10},
	{Pins::TORSIONAL_L, 0, 25},
	//*/
	//For pushing
	{Pins::TORSIONAL_R, 4, 50},
	{Pins::SPRING_B, 33, 10},
	{Pins::TORSIONAL_R, 0, 25},
	{Pins::SPRING_B, 0, 30},
	{Pins::TORSIONAL_L, 4, 50},
	{Pins::SPRING_L, 29, 0},
	{Pins::SPRING_R, 13, 10},
	{Pins::TORSIONAL_L, 0, 25},
	#elif ID == 5
	{Pins::TORSIONAL_R, 6, 50},
	{Pins::SPRING_B, 30, 20},
	{Pins::TORSIONAL_R, 0, 15},
	{Pins::SPRING_B, 0, 30},
	{Pins::TORSIONAL_L, 7, 50},
	{Pins::SPRING_L, 45, 0},
	{Pins::SPRING_R, 30, 10},
	{Pins::TORSIONAL_L, 0, 15},
	#else
	#error This ID is not supported
	#endif
	{Pins::END, 0, 20}					//Pins:: end needs the end of list of each locomotion
};
const uint8_t Tasks::CRAWL_BACKWARD_MANUAL[] PROGMEM = {
	Pins::TORSIONAL_L,
	Pins::SPRING_B,
	Pins::TORSIONAL_L,
	Pins::SPRING_B,
	Pins::TORSIONAL_R,
	Pins::SPRING_L,
	Pins::SPRING_R,
	Pins::TORSIONAL_R,
	Pins::END
};
const uint8_t Tasks::CRAWL_BACKWARD_AUTO[][3] PROGMEM = {
	#if ID == 1
	{Pins::TORSIONAL_L, HIGH, 30},
	{Pins::SPRING_B, 30, 30},
	{Pins::TORSIONAL_L, LOW, 10},
	{Pins::SPRING_B, 0, 20},
	{Pins::TORSIONAL_R, HIGH, 30},
	{Pins::SPRING_L, 60, 0},
	{Pins::SPRING_R, 90, 50},
	{Pins::TORSIONAL_R, LOW, 0},
	//{Pins::SPRING_R, 0, 0},
	//{Pins::SPRING_L, 0, 30},
	#elif ID == 2
	#elif ID == 3
	#elif ID == 4
	#elif ID == 5
	#else
	#error This ID is not supported
	#endif
	{Pins::END, 0, 20}
};
const uint8_t Tasks::VERTICAL_JUMP1_MANUAL[] PROGMEM = {
	Pins::SPRING_L,
	Pins::SPRING_R,
	Pins::END
};
const uint8_t Tasks::VERTICAL_JUMP2_MANUAL[] PROGMEM = {
	Pins::TORSIONAL_L,
	Pins::TORSIONAL_R,
	Pins::SPRING_B,
	Pins::END
};
const uint8_t Tasks::VERTICAL_JUMP_AUTO[][3] PROGMEM = {
	#if ID == 1
	{Pins::SPRING_L, 40, 0},
	{Pins::SPRING_R, 20, 72},//this time is flexible
	{Pins::SPRING_L, 0, 0},
	{Pins::SPRING_R, 0, 80},
	{Pins::TORSIONAL_L, 8, 0},
	{Pins::TORSIONAL_R, 8, 0},
	{Pins::SPRING_B, 170, 45},
	#elif ID == 2
	{Pins::SPRING_L, 35, 0},
	{Pins::SPRING_R, 30, 73},//this time is flexible
	{Pins::SPRING_L, 0, 0},
	{Pins::SPRING_R, 0, 65},
	{Pins::TORSIONAL_L, 8, 0},
	{Pins::TORSIONAL_R, 8, 0},
	{Pins::SPRING_B, 160, 45},
	#elif ID == 3
	#elif ID == 4
	{Pins::SPRING_L, 30, 0},
	{Pins::SPRING_R, 20, 70},//this time is flexible
	{Pins::SPRING_L, 0, 0},
	{Pins::SPRING_R, 0, 80},
	{Pins::TORSIONAL_L, 8, 0},
	{Pins::TORSIONAL_R, 8, 0},
	{Pins::SPRING_B, 170, 45},
	#elif ID == 5
	#else
	#error This ID is not supported
	#endif
	{Pins::END, 0, 50}
};
const uint8_t Tasks::JUMP_FORWARD_AUTO[][3] PROGMEM = {
	#if ID == 1
	{Pins::SPRING_R, 20 , 0},//to 18
	{Pins::SPRING_B, 20 , 50},// to 32?
	{Pins::SPRING_R, 0 , 0},
	{Pins::SPRING_B, 0 , 60},
	{Pins::SPRING_L, 120 , 10},//to 100
	{Pins::SPRING_B, 20 , 40},//previous 20?
	#elif ID == 2
	{Pins::SPRING_R, 20 , 0},//or 32
	{Pins::SPRING_B, 30 , 50},//or32
	{Pins::SPRING_R, 0 , 0},
	{Pins::SPRING_B, 0 , 70},
	{Pins::SPRING_L, 70 , 0},
	{Pins::SPRING_B, 10 , 50},
	#elif ID == 3
	#elif ID == 4
	#elif ID == 5
	#else
	#error This ID is not supported
	#endif
	{Pins::END,0 ,50}
};
const uint8_t Tasks::JUMP_BACKWARD_AUTO[][3] PROGMEM = {
	#if ID == 1
	{Pins::SPRING_L, 37, 0},
	{Pins::SPRING_R, 22, 70},//this time is flexible
	{Pins::SPRING_L, 0, 0},
	{Pins::SPRING_R, 0, 70},
	{Pins::TORSIONAL_L, 8, 0},
	{Pins::TORSIONAL_R, 8, 0},
	{Pins::SPRING_R, 1, 0},
	{Pins::SPRING_B, 170, 45},
	#elif ID == 2
	{Pins::SPRING_L, 30, 0},
	{Pins::SPRING_R, 30, 70},//this time is flexible
	{Pins::SPRING_L, 0, 0},
	{Pins::SPRING_R, 0, 80},
	{Pins::TORSIONAL_L, 8, 0},
	{Pins::TORSIONAL_R, 8, 0},
	{Pins::SPRING_R, 20, 0},
	{Pins::SPRING_B, 130, 45},
	#elif ID == 3
	#elif ID == 4
	#elif ID == 5
	#else
	#error This ID is not supported
	#endif
	{Pins::END,0 ,50}
};
const uint8_t Tasks::ROLL_FORWARD1_MANUAL[] PROGMEM = {
	Pins::SPRING_R,
	Pins::SPRING_B,
	Pins::SPRING_R,
	Pins::SPRING_B,
	Pins::SPRING_L,
	Pins::SPRING_B,
	Pins::END
};
const uint8_t Tasks::ROLL_FORWARD1_AUTO[][3] PROGMEM = {
	#if ID == 1
	{Pins::SPRING_R, 22 , 0},//to 24?it's so sensitive
	{Pins::SPRING_B, 32 , 40},// to 40?it's so sensitive 
	{Pins::SPRING_R, 0 , 0},
	{Pins::SPRING_B, 0 , 70},
	{Pins::SPRING_L, 100, 0},//to 130
	{Pins::SPRING_B, 10 , 50},//fixing
	{Pins::SPRING_B, 0 , 0},
	{Pins::SPRING_L, 0 , 250},
	
	{Pins::SPRING_R, 30 , 0},
	{Pins::SPRING_L, 30 , 40},	
	{Pins::SPRING_L, 0 , 0},
	{Pins::SPRING_R, 110 , 50},
	{Pins::SPRING_R, 0 , 0},
	
	{Pins::SPRING_L, 40 , 30},
	{Pins::SPRING_L, 0 , 30},
	{Pins::SPRING_B, 100 , 40},
	{Pins::SPRING_B, 0 , 0},
	//*/
	#elif ID == 2	
	{Pins::SPRING_R, 31 , 0},//to 24?it's so sensitive
	{Pins::SPRING_B, 31 , 39},// to 40?it's so sensitive
	{Pins::SPRING_R, 0 , 0},
	{Pins::SPRING_B, 0 , 70},
	{Pins::SPRING_L, 90, 0},//to 130
	{Pins::SPRING_B, 5 , 50},//fixing
	{Pins::SPRING_B, 0 , 0},
	{Pins::SPRING_L, 0 , 250},
	
	{Pins::SPRING_R, 30 , 0},
	{Pins::SPRING_L, 10 , 40},
	{Pins::SPRING_L, 0 , 0},
	{Pins::SPRING_R, 110 , 50},
	{Pins::SPRING_R, 0 , 0},
	
	{Pins::SPRING_L, 40 , 30},
	{Pins::SPRING_L, 0 , 30},
	{Pins::SPRING_B, 100 , 40},
	{Pins::SPRING_B, 0 , 0},
	#elif ID == 3
	{Pins::SPRING_R, 22 , 0},//to 24?it's so sensitive
	{Pins::SPRING_B, 32 , 40},// to 40?it's so sensitive
	{Pins::SPRING_R, 0 , 0},
	{Pins::SPRING_B, 0 , 70},
	{Pins::SPRING_L, 100, 0},//to 130
	{Pins::SPRING_B, 10 , 50},//fixing
	#elif ID == 4
	#elif ID == 5
	#else
	#error This ID is not supported
	#endif
	{Pins::END,0 ,50}
};
const uint8_t Tasks::ROLL_BACKWARD_AUTO[][3] PROGMEM = {
	#if ID == 1
	{Pins::SPRING_L, 95 , 0},
	{Pins::SPRING_B, 95 , 33},
	{Pins::SPRING_L, 0 , 10},
	{Pins::SPRING_R, 70 , 20},
	{Pins::SPRING_B, 120 , 10},
	{Pins::SPRING_R, 120 , 30},
	{Pins::SPRING_B, 0 , 0},
	{Pins::SPRING_R, 0 , 50},
		
	{Pins::SPRING_B, 85 , 0},
	{Pins::SPRING_R, 85 , 33},
	{Pins::SPRING_B, 0 , 10},
	{Pins::SPRING_L, 90 , 20},
	{Pins::SPRING_B, 120 , 10},
	{Pins::SPRING_L, 140 , 30},
	{Pins::SPRING_B, 0 , 0},
	{Pins::SPRING_L, 0 , 50},
	
	{Pins::SPRING_R, 95 , 0},
	{Pins::SPRING_L, 85 , 33},
	{Pins::SPRING_R, 0 , 10},
	{Pins::SPRING_B, 70 , 20},
	{Pins::SPRING_L, 125 , 10},
	{Pins::SPRING_B, 125 , 30},
	//{Pins::SPRING_L, 0 , 0},
	//{Pins::SPRING_B, 0 , 50},
	#elif ID == 2
	#elif ID == 3
	#elif ID == 4
	#elif ID == 5
	#else
	#error This ID is not supported
	#endif
	{Pins::END,0 ,50}
};
///////////////////////////////////////////////////////////////////You might have to change until here/////////////////////////////////////////////////////////////////////////////////////

//I don't use these two array.
const uint8_t Tasks::SCAN_FORWARD[][3] PROGMEM = {
	{Pins::SPRING_B, 80, 0},
	{Pins::SPRING_L, 80, 60},
	{Pins::SCANNING_READY,0 ,0}
};
const uint8_t Tasks::SCAN_BACKWARD[][3] PROGMEM = {
	{Pins::SPRING_B, 80, 0},
	{Pins::SPRING_R, 80, 80},
	{Pins::SCANNING_READY,0 ,0}
};
//
//Constructor
//
Robogami::Robogami(){
}
void Robogami::begin(void) {
	/*
	* This function set pin Mode as input or output
	* DDRB |= _BV(PORTB0) means set DDRB's bit 0 1 and now PORTB0 is output
	* DDRB |= _BV(PORTB0) | _BV(PORTB1) means now PORTB0 and PORTB1 are set as output
	*/
	DDRA |= _BV(PORTA1);
	DDRB |= _BV(PORTB0) | _BV(PORTB1) | _BV(PORTB2) | _BV(PORTB3) | _BV(PORTB4);
	DDRD |= _BV(PORTD1) | _BV(PORTD5) | _BV(PORTD6);

}
void Robogami::setId(uint8_t id) {
	Robogami::id = id;
}
uint8_t Robogami::getId(void) {
	return Robogami::id;
}
void Robogami::setRole(uint8_t role) {
	Robogami::role = role;
}
uint8_t Robogami::getRole(void) {
	return Robogami::role;
}
void Robogami::writePin(uint8_t pin, uint8_t value) {
	/* This function is for writing pin.
	* I combined digitalWrite and analogWrite, and simplify because I had to save flush memory
	* Attiny4313 has two timer: timer0 and timer1 each of them has two PWM channel so totally attiny has 4 PWM channel
	*/
	switch (pin) {
		case 11://PORTB2
			if(value == 0) {TCCR0A &= ~_BV(COM0A1);}//disconnet timer0 to PORTB2
			else {TCCR0A |= _BV(COM0A1);}						//connect timer0 to PORTB2
			OCR0A = value;													//duty = OCR0A/256
			break;
		case 7://PORTD5
			if(value == 0) {TCCR0A &= ~_BV(COM0B1);}//disconnect timer0 from PORTD5
			else {TCCR0A |= _BV(COM0B1);}						//connect timer0 to PORTD5
			OCR0B = value;													//duty = OCR0B/256
			break;
		case 12://PORTB3
			if(value == 0) {TCCR1A &= ~_BV(COM1A1);}//disconnect timer1 from PORTB3
			else {TCCR1A |= _BV(COM1A1);}						//connect timer1 to PORTB3
			OCR1A = value;													//duty = OCR1A/256
			break;
		case Pins::TORSIONAL_L:										//PWM for torsional SMA is realized by software PWM which frequence is almost 48Hz
			Duty_Torsional_L = value;								//Duty should be set 0 to 10, if you set duty 5, "real duty value" is 0.5, 50%. 
			Counter_Torsional_L = 0;								//initialize counter 
			break;
		case Pins::TORSIONAL_R:
			Duty_Torsional_R = value;
			Counter_Torsional_R = 0;
			break;
		default://for normal pin
			if (value == 0) {
				*portOutputRegister(digitalPinToPort(pin)) &= ~digitalPinToBitMask(pin);
			}
			else {
				*portOutputRegister(digitalPinToPort(pin)) |= digitalPinToBitMask(pin);
			}
			break;
	}
}
void Robogami::stopSMA(void) {
	writePin(Pins::SPRING_L, 0);
	writePin(Pins::SPRING_R, 0);
	writePin(Pins::SPRING_B, 0);
	writePin(Pins::TORSIONAL_L, 0);
	writePin(Pins::TORSIONAL_R, 0);
}
//
// Preinstantiate Objects
//
Robogami Tribot = Robogami();