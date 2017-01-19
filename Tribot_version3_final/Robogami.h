//
//  Robogami.h
//  
//
//  Created by Kazuaki MORI on 2016/07/23.
//
//
/*
 PORTB = 
 7:SDA
 6:SENSOR2
 5:SCL
 4:SPRING_B
 3:SPRING_R
 2:OMEGA_R
 1:RX2
 0:TX2
 PORTD = 
 6:OMEGA_L
 5:SPRING_L
 4:LED2
 3:LED1
 2:RX1
 1:Tx1
 0:SENSOR1
 PORTA =
 2:RESET
 1:Blank
 0:Blank
*/
#ifndef Robogami_h
#define Robogami_h
//
//include
//
#include "Pins.h"
//
//Global variables
//
extern volatile uint8_t Duty_Torsional_L;
extern volatile uint8_t Counter_Torsional_L;
extern volatile uint8_t Duty_Torsional_R;
extern volatile uint8_t Counter_Torsional_R;
//
//Const
//
struct Tasks {
	static const uint8_t BLINK_LED_G[][3] PROGMEM;
	static const uint8_t BLINK_LED_R[][3] PROGMEM;
	static const uint8_t SET_INITIAL[][3] PROGMEM;
	static const uint8_t CRAWL_FORWARD_AUTO[][3] PROGMEM;
	static const uint8_t CRAWL_FORWARD_MANUAL[] PROGMEM;
	static const uint8_t CRAWL_BACKWARD_AUTO[][3] PROGMEM;
	static const uint8_t CRAWL_BACKWARD_MANUAL[] PROGMEM;
		
	static const uint8_t VERTICAL_JUMP_AUTO[][3] PROGMEM;
	static const uint8_t VERTICAL_JUMP1_MANUAL[] PROGMEM;
	static const uint8_t VERTICAL_JUMP2_MANUAL[] PROGMEM;
	
	static const uint8_t JUMP_FORWARD_AUTO[][3] PROGMEM;
	static const uint8_t JUMP_FORWARD_MANUAL[] PROGMEM;
	static const uint8_t JUMP_BACKWARD_AUTO[][3] PROGMEM;
	static const uint8_t JUMP_BACKWARD_MANUAL[] PROGMEM;
	static const uint8_t ROLL_FORWARD1_AUTO[][3] PROGMEM;
	static const uint8_t ROLL_FORWARD1_MANUAL[] PROGMEM;
	static const uint8_t ROLL_BACKWARD_AUTO[][3] PROGMEM;
	static const uint8_t ROLL_BACKWARD_MANUAL[] PROGMEM;
	static const uint8_t SCAN_FORWARD[][3] PROGMEM;
	static const uint8_t SCAN_BACKWARD[][3] PROGMEM;
};
/////////////////////////////////////
//
//class
//
class Robogami{
  private:
	//
	//private members
	//
	uint8_t id;
	uint8_t role;
	//
	//public methods
	//
  public:
    Robogami();
    void setId(uint8_t id);
		void setRole(uint8_t role);
    uint8_t getId(void);
		uint8_t getRole(void);
    void begin(void);
		void simpleDelay(unsigned long ms);
    void setNeutral(void);
    void writeDigital(uint8_t pin, uint8_t value);
    void flushLED(uint8_t led, uint8_t repeat, uint16_t delaytime);
    void writePin(uint8_t pin, uint8_t value);
		void writePWM(uint8_t pin, uint8_t value);
		void doRoutine(uint8_t pinList[][3]);
		void stopSMA(void);
		void crawling(bool direction);
		void jumping(bool direction);
		void jumping(void);
		void rolling(bool direction);
		void checkSMA(void);
};
extern Robogami Tribot;
#endif /* Robogami_h */
