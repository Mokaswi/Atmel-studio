/*
 * wiring.h
 *
 * Created: 27.10.2016 23:05:57
 *  Author: Kazuaki Mori
 * In this file, I mention Macro constants, Macro functions about I/O port and about Timer0
 * and declare register variables. Besides, there are function prototypes for main.cpp
 */ 
#ifndef WIRING_H_
#define WIRING_H_
//
//F_CPU
//
#ifndef F_CPU
#define F_CPU 8000000UL
#endif
//
//include
//
#include <stdint.h>
#include <avr/pgmspace.h>
#include "SoftwareIrDAINT.h"
//
//Macro constants
//
#define NOT_A_PIN 0
#define NOT_A_PORT 0

#define NOT_AN_INTERRUPT -1


#define PA 1
#define PB 2
#define PC 3
#define PD 4

#define NOT_ON_TIMER 0
#define TIMER0A 1
#define TIMER0B 2
#define TIMER1A 3
#define TIMER1B 4

#define HIGH 1
#define LOW 0
//
//register variables
//
register volatile uint8_t rx_BytesAvail asm("r2");      // number of bytes requested but not read
register volatile uint8_t rx_LastRead asm("r3");
register uint8_t Infrared_Idx asm("r4");
register uint8_t SFlags asm("r5");
register uint8_t *Locomotion asm("r6");//r6:r7

//
//
//Global variables
//
// On the ATmega1280, the addresses of some of the port registers are
// greater than 255, so we can't store them in uint8_t's.
extern const uint16_t port_to_mode_PGM[] PROGMEM;
extern const uint16_t port_to_input_PGM[] PROGMEM;
extern const uint16_t port_to_output_PGM[]  PROGMEM;

extern const uint8_t digital_pin_to_port_PGM[] PROGMEM;
extern const uint8_t digital_pin_to_bit_mask_PGM[] PROGMEM;
extern const uint8_t digital_pin_to_timer_PGM[] PROGMEM;
//
//Macro functions
//
// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
//
// These perform slightly better as macros compared to inline functions
//
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
#define analogInPinToBit(P) (P)
#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
//
// These perform slightly better as macros compared to inline functions
//
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
#define analogInPinToBit(P) (P)
#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )
//
//About timer0 function
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )

#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
//
//SoftwareIrDA
//
extern SoftwareIrDA irPortFore;
extern SoftwareIrDA irPortBack;
//
//Function Prototypes
//
//
//For wiring.cpp
unsigned long millis();

uint8_t rxAvailable(void);
uint8_t rxRead(void);
void initializeRxIdxs(void);
uint8_t isEmptyRxBuf(void);
void sendInfrared(uint8_t infrared[], char channel);
void sendInfrared(uint8_t infrared[]);

void clearGIFR(void);
void saveGIMSK(void);
void loadGIMSK(void);
//
//for main.cpp
void setTxArray(uint8_t to, uint8_t command, uint8_t message1, uint8_t message2);
void setTxArray(uint8_t to, uint8_t command, uint8_t message1);
void setTxArray(uint8_t to, uint8_t command);
uint8_t translateFromInfrared(uint8_t *infrared);

void blink(uint8_t pin, uint8_t i);
void blinkLedRed(uint8_t i);
void setLocomotion(uint8_t *locomotion);

unsigned long measureProximityValue(uint8_t sensor, uint8_t n);
uint8_t calculateDistance(unsigned long distance, uint8_t sensor);

void rxReceiveEvent(void);
void infraredAvailableEvent(void);
void manualLocomotionEvent(void);
void autoLocomotionEvent(void);
void pushingEvent(void);
void followingEvent(void);
void scanningEvent1(void);
void scanningEvent2(void);

void setup(void);
void loop(void);
#endif /* WIRING_H_ */