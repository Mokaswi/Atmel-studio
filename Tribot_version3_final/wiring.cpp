/*
 * wiring.cpp
 *
 * Created: 27.10.2016 23:13:35
 * Author: Kazuaki Mori
 * In this file, I state definitions of PROGMEM arrays that are declared in wiring.h
 * and Interrupt functions, receiving functions and basic functions that are used in main.cpp(e.g. millis, etc.)
 * 
 */
//
//include
//
#include <avr/interrupt.h>
#include <util/delay.h>
#include "wiring.h"
#include "Robogami.h"
#include "Messages.h"
//
//Constants
//
const uint8_t RX_BUF_LEN = 30;
//
//Global variables
//
volatile unsigned long timer0_millis = 0;
volatile uint8_t Saved_GIMSK = 0x07;					//IT should be initilized with 0x07
uint8_t Counter_Overlap_SavingGIMSK = 0;
extern volatile uint8_t Duty_Torsional_L;			//it's defined in Robogami.cpp
extern volatile uint8_t Counter_Torsional_L;	//it's defined in Robogami.cpp
extern volatile uint8_t Duty_Torsional_R;			//it's defined in Robogami.cpp
extern volatile uint8_t Counter_Torsional_R;	//it's defined in Robogami.cpp
//
//Global arrays
//
volatile uint8_t rx_Buf[RX_BUF_LEN];
//
//SoftwareIrDA
//
SoftwareIrDA irPortFore(1);	//For fore infrared transceiver
SoftwareIrDA irPortBack(2);	//For back infrared transceiver
//
//PROGMEM for macro function
//This is completely same as ones which is used in Arduino
//Don't change
//
const uint16_t port_to_mode_PGM[] PROGMEM =
{
	NOT_A_PORT,
	(uint16_t)&DDRA,
	(uint16_t)&DDRB,
	NOT_A_PORT,
	(uint16_t)&DDRD,
};

const uint16_t port_to_output_PGM[] PROGMEM =
{
	NOT_A_PORT,
	(uint16_t)&PORTA,
	(uint16_t)&PORTB,
	NOT_A_PORT,
	(uint16_t)&PORTD,
};

const uint16_t port_to_input_PGM[] PROGMEM =
{
	NOT_A_PORT,
	(uint16_t)&PINA,
	(uint16_t)&PINB,
	NOT_A_PORT,
	(uint16_t)&PIND,
};

const uint8_t digital_pin_to_port_PGM[] PROGMEM =
{
	PD, /* 0 */
	PD,
	PA,
	PA,
	PD,
	PD,
	PD,
	PD,
	PD, /* 8 */
	PB,
	PB,
	PB,
	PB,
	PB,
	PB, /* 14 */
	PB,
	PB,
	PA,
};

const uint8_t digital_pin_to_bit_mask_PGM[] PROGMEM =
{
	_BV(0), /* 0 */
	_BV(1),
	_BV(1),
	_BV(0),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6), /* 8 */
	_BV(0),
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5), /* 14 */
	_BV(6),
	_BV(7),
	_BV(2),
};

const uint8_t digital_pin_to_timer_PGM[] PROGMEM =
{
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER0B,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER0A,
	TIMER1A,
	TIMER1B,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
};
//
//interrupt functions
//There three interrupt functions, one is for timer0, another one is for receiving infrared signal with fore infrared transceiver, ant the other is for back infrared transceiver
//
ISR(TIMER0_OVF_vect) {
	//
	//For millis
	//
	timer0_millis += MILLIS_INC;//this is used in millis()
	//
	//Software PWM
	//
	//
	//Torshional SMA L
	if(Counter_Torsional_L == Duty_Torsional_L) {
		PORTD &= ~_BV(PORTD6);
	}
	else if(Counter_Torsional_L == 0) {
		PORTD |= _BV(PORTD6);
	}
	Counter_Torsional_L++;
	if(Counter_Torsional_L == 10) {
		Counter_Torsional_L = 0;
	}
	//
	//Torshional SMA R
	if(Counter_Torsional_R == Duty_Torsional_R) {
		PORTB &= ~_BV(PORTB1);
	}
	else if(Counter_Torsional_R == 0) {
		PORTB |= _BV(PORTB1);
	}
	Counter_Torsional_R++;
	if(Counter_Torsional_R == 10) {
		Counter_Torsional_R = 0;
	}
}
ISR(INT0_vect){
	rx_Buf[rx_BytesAvail++] = irPortFore.receiveByte();
	if (rx_BytesAvail > RX_BUF_LEN-2) {//overflow
		isEmptyRxBuf();
	}
	clearGIFR();
}
ISR(INT1_vect){
	rx_Buf[rx_BytesAvail++] = irPortBack.receiveByte();
	if (rx_BytesAvail > RX_BUF_LEN-2) {//overflow
		isEmptyRxBuf();
	}
	clearGIFR();
}
unsigned long millis(void) {
	unsigned long m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = timer0_millis;
	SREG = oldSREG;

	return m;
}
uint8_t rxAvailable(void) {
	return rx_BytesAvail - rx_LastRead;
}
uint8_t rxRead(void) {
	return rx_Buf[rx_LastRead++];
}
void initializeRxIdxs(void) {
	rx_BytesAvail = 0;
	rx_LastRead = 0;
}
uint8_t isEmptyRxBuf(void) {
	if ( rx_BytesAvail <= rx_LastRead ) {
		return true;
	}
	else {
		return false;
	}
}
void sendInfrared(uint8_t infrared[], char channel) {
	uint8_t i = 0;
	saveGIMSK();
	do {
		_delay_us(100);
		if(channel == 1) {
			irPortFore.transmitByte(infrared[i]);
		}
		else {
			irPortBack.transmitByte(infrared[i]);
		}
		i++;
	} while( infrared[i-1] != Infrareds::EOM );
	loadGIMSK();
}
void sendInfrared(uint8_t infrared[]) {
	uint8_t i = 0;
	saveGIMSK();
	do {
		_delay_us(100);
		irPortFore.transmitByte(infrared[i++]);
	} while( infrared[i-1] != Infrareds::EOM );
	i = 0;
	do {
		_delay_us(100);
		irPortBack.transmitByte(infrared[i++]);
	} while( infrared[i-1] != Infrareds::EOM );
	loadGIMSK();
}
void clearGIFR(void) {
	if(GIFR & _BV(INTF0)) {
		GIFR |= _BV(INTF0);
	}
	if(GIFR & _BV(INTF1)) {
		GIFR |= _BV(INTF1);
	}
}
void saveGIMSK(void) {
	if ( Saved_GIMSK == 0x07 ) {
		Saved_GIMSK = GIMSK;
		GIMSK &= ~_BV(INT0) & ~_BV(INT1);  //INT OFF
		Tribot.writePin(Pins::LED_G, LOW);
	}
	else {
		Counter_Overlap_SavingGIMSK++;
	}
}
void loadGIMSK(void) {
	if ( Counter_Overlap_SavingGIMSK ) {
		Counter_Overlap_SavingGIMSK--;
	}
	else {
		clearGIFR();
		//if( Saved_GIMSK != 0x07) {
			GIMSK = Saved_GIMSK;
			Saved_GIMSK = 0x07;
			Tribot.writePin(Pins::LED_G, HIGH);
		//}
	}
}