/* This file is written by Kazuaki Mori
* Basically(and hopefully) you have to change just this main.cpp and Robogami.cpp
* Usually, you have to change proxi/sensor convert table, offset, and setRole
*
*
*                 Back sensor|Fore sensor
*                            |
*              SPRING_R      |		SPRING_L
*                            |+
*                           / \
*                          /   \
*              TORSIONAL_R/     \TORSIONAL_L
*                      __/       \__
*                        SPRING_B
*
* + means there is micro controller
*/
//
//include
//
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "wiring.h"						//For basic functions
#include "TinyVCNL4010.h"			//For sensor
#include "SoftwareIrDAINT.h"	//For infrared communication
#include "Messages.h"					//Also for infrared communication
#include "Robogami.h"					//For activate SMAs, LEDs and locomotion
//
//Constants
//
//Involves with communication
const uint8_t INFRARED_BUF_LEN = 30;
const uint16_t RECEIVE_TIMEOUT = 1000;//1s
//const uint16_t DONE_TASK_TIMEOUT = 20000;
//const uint16_t RECEIVE_DISTANCE_TIMEOUT = 500;
//
//SFlag's bits
const uint8_t DRCF = 0; //DiReCtion Flag
const uint8_t MDRCF = 1; //Measure DiReCtion Flag
const uint8_t MLF = 2;	//Manual Locomotion Flag
const uint8_t IAF = 3;	//Infrared_buf Available Flag
const uint8_t PSF = 4;  //PuSh Flag
const uint8_t SCNF1 = 5;  //SCaNing Flag 1		
const uint8_t SCNF2 = 6;  //SCaNing Flag 2
const uint8_t FLWF = 7;  //FoLloWing Flag
//
//Roles
const uint8_t BASEMENT = 100;
const uint8_t LEADER = 101;
const uint8_t MEASURE = 102;
const uint8_t PUSH = 103;
const uint8_t RELAY = 104;
const uint8_t ALL = 105;
//
//Proxi/distance table
//
/*This is the table to convert Proximity value to distance
*
* (Proximity value - offset) = C1*exp(C2*Distance(mm))
*
* C1 and C2 depend on measured object and offset depends on the environment and differences between each sensors.
* To be more accurate, I add 2.5 or 1 or something to distance.
* For example, if sensor can detect 5mm difference and if I don't put the additional values, Tribot thinks 85~80 mm is 85 mm and 80 ~75 is 80mm
* If I add 2.5 which is a half value of resolution of sensor, Thibot thinks 87.5~82.5 mm is 85 mm and 82.5~77.5 is 80 mm.
* C1 and C2 have to be estimated.
*/
/////////////////////////////////////////////////You might have to change from here////////////////////////////////////////////////////////////////////////////
const uint16_t C1 = 1588 ;//For lying yellow block which we use in pushing
const double C2 = -0.032;//For lying yellow block which we use in pushing

const uint16_t PROXI_DISTANCE_TABLE_LYING_YELLOWBLOCK[][2] PROGMEM = {//{proxi, distance}
{(uint16_t)(C1*exp(C2*(90+2.5))), 90},
{(uint16_t)(C1*exp(C2*(85+2.5))), 85},
{(uint16_t)(C1*exp(C2*(80+2.5))), 80},
{(uint16_t)(C1*exp(C2*(78+1))), 78},
{(uint16_t)(C1*exp(C2*(76+1))), 76},
{(uint16_t)(C1*exp(C2*(74+1))), 74},
{(uint16_t)(C1*exp(C2*(72+1))), 72},
{(uint16_t)(C1*exp(C2*(70+1))), 70},
{(uint16_t)(C1*exp(C2*(68+1))), 68},
{(uint16_t)(C1*exp(C2*(66+1))), 66},
{(uint16_t)(C1*exp(C2*(64+1))), 64},
{(uint16_t)(C1*exp(C2*(62+1))), 62},
{(uint16_t)(C1*exp(C2*(60+1))), 60},
{(uint16_t)(C1*exp(C2*(58+1))), 58},
{(uint16_t)(C1*exp(C2*(56+1))), 56},
{(uint16_t)(C1*exp(C2*(54+1))), 54},
{(uint16_t)(C1*exp(C2*(52+1))), 52},
{(uint16_t)(C1*exp(C2*(50+1))), 50},
{(uint16_t)(C1*exp(C2*(48+1))), 48},
/////////////////////////////////////////////////You might have to change until here////////////////////////////////////////////////////////////////////////////
{0xFFFF, 0}
};
//
//Global variables
//
uint8_t Distance = 0;									//This is for pushing
//THRESHOULD
uint8_t Pushing_Threshoud = 10;
uint8_t Following_Threshold = 10;			//isn't used
uint8_t Scanning_Threshold = 10;			//isn't used
//
//Locomotion
unsigned long Locomotion_Millis = 0;
uint8_t Locomotion_Idx = 0;						
//
//Initial Sensor offsets
#if ID == 1
uint16_t Offset_SENSOR_FORE = 2514;  //measure at 23092016 17
uint16_t Offset_SENSOR_BACK = 2512;  //measure at 23092016 17
#elif ID == 2
uint16_t Offset_SENSOR_FORE = 2514;  //measure at 23092016 17
uint16_t Offset_SENSOR_BACK = 2512;  //measure at 23092016 17
#elif ID == 3
uint16_t Offset_SENSOR_FORE = 2514;  //measure at 23092016 17
uint16_t Offset_SENSOR_BACK = 2512;  //measure at 23092016 17
#elif ID == 4
uint16_t Offset_SENSOR_FORE = 2514;  //measure at 23092016 17
uint16_t Offset_SENSOR_BACK = 2512;  //measure at 23092016 17
#elif ID == 5
uint16_t Offset_SENSOR_FORE = 2380;  //measure at 2016/12/09
uint16_t Offset_SENSOR_BACK = 2438;  //measure at 2016/12/05
#else
#error This ID is not supported
#endif
//
//Global arrays
//
uint8_t Infrared_Buf[INFRARED_BUF_LEN];
//
//Functions
//
int main(void) {
  //setup should be called first.
	setup();
	//
	//main loop
	//
	while(1) {
		while(rxAvailable() & !(SFlags & _BV(IAF)) ) {//If Rx_buf have message
			rxReceiveEvent();
		}
		if( SFlags & _BV(IAF) ) {//If Infrared_buf have message
			infraredAvailableEvent();
		}
		if ( Locomotion != NULL ) {//If Tribot is doing a locmotion
			if( SFlags & _BV(MLF)) {
				manualLocomotionEvent();
			}
			else {
				autoLocomotionEvent();
			}
		}
		if ( SFlags & _BV(PSF) ) {//If Tribot(Leader) is doing pushing
			pushingEvent();
		}
		/*
		* isn't used
		if ( SFlags & _BV(FLWF) ) {
			followingEvent();
		}
		if ( SFlags & _BV(SCNF1) ) {
			scanningEvent1();  
		}
		if ( SFlags & _BV(SCNF2) ) {
			scanningEvent2();
		}
		//*/
  }
}
void setup(void) {
  //
  //initialize register variables,   register variables are declared in wiring.h
	SFlags = 0;
  rx_BytesAvail = 0;
  rx_LastRead = 0;
  Infrared_Idx = LEN_INFRARED_BOM;;
  Locomotion = NULL;
  //
  //set registers for timer0 interrupt and PWM
  //prescaler is 64 and mode is fast PWMs(Now frequence of PWM is 8MHz/64*256 = 488.28125 kHz
  TCCR0A |= _BV(WGM01) | _BV(WGM00);					//set mode of timer0 fast PWM
  TCCR0B |= _BV(CS01) | _BV(CS00);						//set prescaler of timer0 64
  TCCR1A |= _BV(WGM10);														//set mode of timer1 fast PWM
  TCCR1B |= _BV(WGM12) | _BV(CS11) | _BV(CS10);		//and set prescaler of timer1 64
  TIMSK |= _BV(TOIE0);														//enable timer0 interrupt
  sei();																					//enable interrput flag
  //
  //set ID and ROLE
	Tribot.setId(ID);
	////////////////////////////////////////////////////////////////////////////////////////////////You might have to change from here!////////////////////////////////////////////////////////
  #if ID == 1
  Tribot.setRole(LEADER);
  #elif ID == 2
  Tribot.setRole(PUSH);
  #elif ID == 3
  Tribot.setRole(RELAY);//role like mirror or messanger?
	#elif ID == 4
  Tribot.setRole(PUSH);
	#elif ID == 5
  Tribot.setRole(MEASURE);
  #endif
	////////////////////////////////////////////////////////////////////////////////////////////////You might have to change until here!////////////////////////////////////////////////////////
  //
  //Begin Tribot(Just set Pin mode for SMAs and LEDs)
  //
  Tribot.begin();
  //
  //Sensor initialization
	//set current for measuring(set to maximum current 200mA, which allow sensor to measure fare)
  //if you just send same command to sensors, you can turn on both at same time.
  Tribot.writePin(Pins::SWI_SENSOR_FORE, HIGH);
  Tribot.writePin(Pins::SWI_SENSOR_BACK, HIGH);

  VCNL4010.begin();

  Tribot.writePin(Pins::SWI_SENSOR_FORE, LOW);
  Tribot.writePin(Pins::SWI_SENSOR_BACK, LOW);
	//
	//Set SFalgs
	//
  //SFlags &= ~_BV(DRCF);		//used in following and scanning so i don't use this
  //SFlags |= _BV(DRCF);		//same as upper
  //
  //Set interrupt for INI0 and INT1
  //  
  MCUCR = _BV(ISC01) | _BV(ISC11); // INT0 and INT1 fall down
  clearGIFR();										//Just in case, when you INT0 and INT1 ON, you have to call this function
  GIMSK |= _BV(INT0) | _BV(INT1);	//enable interrupt by INT0 and INT 1 which are used in infrared communication  
  //
  //turn LED_G on
  //
  Tribot.writePin(Pins::LED_G, HIGH);
}
void rxReceiveEvent(void) {
  /*
	* Tribot use interrupt function which is declared in wiring.cpp to receive infrared messages.
 	* These data are stored in Rx_Buf and temporally and then they are read into Infrared_Buf from Beginning of Message(BOM) to End of Message(EOM)
	* One message composed of BOM,  Target_ID, Relay Flag, command, some parameters and EOM. 
	*/
	uint8_t error = false;
  Infrared_Buf[0] = rxRead();								//Read 1 character
  if ( Infrared_Buf[0] == Infrareds::BOM ) {//Found Beginning of message
		Infrared_Idx = LEN_INFRARED_BOM;				//Initialize Infrared_Idx
		unsigned long rxMillis = millis();			//For Time out
		//
		//Beginning of reading loop
		do {
			if ( rxAvailable() ) {//if there are unread data
				Infrared_Buf[Infrared_Idx++] = rxRead();//read 1 character
			}
			if ( millis() - rxMillis > RECEIVE_TIMEOUT || Infrared_Idx > INFRARED_BUF_LEN-2 ) {//Time out
				error = true;	//Now error happened		
				break;
			}
		} while( Infrared_Buf[Infrared_Idx-1] != Infrareds::EOM );//read until EOM
		//End of reading loop
		//
		if ( error == false ) {//if receiving is done successfully 
			saveGIMSK();					//Infrared communication off
			SFlags |= _BV(IAF);		//Now set flag and there is reasonable message in Infrared_Buf
		}
		else {//if fail
			blinkLedRed(1);	//blink red led once
		}		
	}
  if( isEmptyRxBuf() ) {//if it reads all received data
		initializeRxIdxs();
  }
}
void infraredAvailableEvent(void) {//This is called when Infrared_buf has data
	translateFromInfrared(Infrared_Buf);										//Translate message
	if( Infrared_Buf[IDX_RELAY_FLAG] == Tribot.getRole()) {	//If relay flag is for this robot
		sendInfrared(Infrared_Buf);														//Play a role as mirror
	}
	else {
		_delay_ms(50);
	}
	SFlags &= ~_BV(IAF);	//clear flag
	loadGIMSK();					//infrared communication on
}
void manualLocomotionEvent(void) {
	/*
	* This is to realize locomotion with parameters which you can change without uploading program
	* you will use this when you want to find good parameters or crawling in multiple terrains
	*	while do this, Tribot can't communicate until the end of locomotion because parameters are saved in Infrared_Buf
	* written pins are defined in Robogami.cpp
	*/
  if( millis() - Locomotion_Millis >= (uint16_t)100*Infrared_Buf[Locomotion_Idx+1] ) {//If time passed
		do {
			Locomotion++;							//increment
			Locomotion_Idx += 2;			//increment
			if ( pgm_read_byte(Locomotion) == Pins::END ) {//end of locomotion
				Tribot.stopSMA();			//Just in case, turn off every SMAs
				Locomotion = NULL;		//make Pointer empty
				SFlags &= ~_BV(MLF);	//clear Flag
				loadGIMSK();					//Infrared communication on
				break;
			}
			else {
				Tribot.writePin(pgm_read_byte(Locomotion), Infrared_Buf[Locomotion_Idx] );		//write pin
			}
			Locomotion_Millis = millis();		//update millis
		}while( Infrared_Buf[Locomotion_Idx+1] == 0 );//if during = 0, write next pin immediately
  }
}
void autoLocomotionEvent(void) {
	/*This function is for locomotion with pre-set parameters.
	* Tribot can communicate while doing this, but I make it not communicate because in some cases, noise generated by PWM and SMA cause problems in communication. 
	* If you want to change pre-set paramters and the order of pins, please check Robogami.cpp
	* And when end this locomotion, Tribot send message which means Done locomotion.
	* it is used in Pushing experiments.
	*/
  if( millis() - Locomotion_Millis >= (uint16_t)100*pgm_read_byte(Locomotion+2) ) {
		do {
			if ( pgm_read_byte(Locomotion) == Pins::END ) {	//end process
				Locomotion = NULL;														//make Pointer empty
				setTxArray(Infrared_Buf[IDX_FROM_ID], Infrared_Buf[IDX_RELAY_FLAG], Infrareds::DONE_TASK);//set message in Infrared_Buf
				sendInfrared(Infrared_Buf);											//Send infrared message
				break;
			}
			Locomotion += 3;	//increment of pointer
			if ( pgm_read_byte(Locomotion) == Pins::END ) {
				Tribot.stopSMA();							//Just in case, turn off every SMAs
				loadGIMSK();									//Communication on
			}
			else {
				Tribot.writePin(pgm_read_byte(Locomotion), pgm_read_byte(Locomotion+1) );//write pin
			}
			Locomotion_Millis = millis();//update millis
		} while( pgm_read_byte(Locomotion+2) == 0 );//if during is 0, go to next loop immediately
  }
}
void pushingEvent(void) {
	/*
	static unsigned long done_task_millis;
	static unsigned long receive_distance_millis;
	if (Infrared_Buf[IDX_COMMAND] == Infrareds::DONE_TASK) {
		Tribot.writePin(Pins::LED_R, LOW);
		setTxArray(MEASURE, RELAY, Infrareds::GET_DISTANCE_FORWARD);
		sendInfrared(Infrared_Buf);
		receive_distance_millis = millis();
		done_task_millis = millis();
	}
	if(millis() - receive_distance_millis > RECEIVE_DISTANCE_TIMEOUT) {
		setTxArray(MEASURE, RELAY, Infrareds::GET_DISTANCE_FORWARD);
		sendInfrared(Infrared_Buf);
	}
	if (Infrared_Buf[IDX_COMMAND] == Infrareds::RECEIVE_DISTANCE) {
		setTxArray(BASEMENT, 0, Infrareds::RECEIVE_DISTANCE, Distance);
		sendInfrared(Infrared_Buf);
		if ( Distance > Pushing_Threshoud ) {
			Tribot.writePin(Pins::LED_R, HIGH);
			setTxArray(PUSH, 0, Infrareds::CRAWL_FORWARD_AUTO);
			sendInfrared(Infrared_Buf);
			done_task_millis = millis();
			receive_distance_millis = millis();
		}
		else {
			blinkLedRed(5);
			SFlags &= ~_BV(PSF);
		}
	}
	if (millis() - done_task_millis > DONE_TASK_TIMEOUT) {
		setTxArray(PUSH, 0, Infrareds::CRAWL_FORWARD_AUTO);
		sendInfrared(Infrared_Buf);
	}
	//*/
	/*
	* This is pushing function. Just leader Tribot use it.
	* Any other Tribots just receive command or return message like distance, done_task.
	* Leader Tribot judge what it should do from received command
	* when it receive done_task command it means one pushing(crawling) is done, leader send measuring Tribot command "getdistanceforward"
	* when it receive receive_distance command it means measuring is done, leader Tribot judge distance is bigger than threshold or not.
	* if distance is bigger, Leader Tribot send pushing Tribot command "crawlingforwardauto" which means it have to crawl with pre-set parameters.
	* Pushing Tribots return message "Done_task" at the end of crawling, and then Leader Tribot is in the case when it receive done task command. 
	*/
	///*
	if (Infrared_Buf[IDX_COMMAND] == Infrareds::DONE_TASK) {//if one pushing cycle(crawling) is done
		_delay_ms(50);																								//It's necessary to prevent communication failure
		setTxArray(MEASURE, RELAY, Infrareds::GET_DISTANCE_FORWARD);	//set message to make measuring Tribot measure distance
		sendInfrared(Infrared_Buf);																		//send message
		Tribot.writePin(Pins::LED_R, LOW);														//Turn Red LED off
  }
  else if (Infrared_Buf[IDX_COMMAND] == Infrareds::RECEIVE_DISTANCE) {//if receive distance 
		_delay_ms(50);																									//It's necessary to prevent communication failure
		setTxArray(BASEMENT, 0, Infrareds::RECEIVE_DISTANCE, Distance);	//set message to inform distance to Basement
		sendInfrared(Infrared_Buf);																			//send message
		_delay_ms(100);																									//It's necessary to prevent communication failure
		if ( Distance > Pushing_Threshoud ) {//keep pushing
			setTxArray(PUSH, 0, Infrareds::CRAWL_FORWARD_AUTO);						//set message to make pushing Tribot push
			sendInfrared(Infrared_Buf);																		//send message
			Tribot.writePin(Pins::LED_R, HIGH);	//Turn Red LED on on-red led means now one pushing is being done
		}
		else {
			setTxArray(BASEMENT, 0, Infrareds::DONE_TASK);	// set message to inform pushing is done to basement
			sendInfrared(Infrared_Buf);											//send message
			SFlags &= ~_BV(PSF);														//Clear flag
			blinkLedRed(5);																	//to inform 
		}
  }
	//*/
}

unsigned long measureProximityValue(uint8_t sensor, uint8_t n){
  /*
	* This is function to command sensor to measure proximity value and receive it.
	* Proximity sensor use I2C communication. If you want to know how you have to command sensors, please check VCNL4010's datasheet
	* proximity value is 16-bit and it's read each 8 bit
	* this function average proximity value n times.
	*/
	unsigned long proximity = 0;				//should be initialize with 0 
  saveGIMSK();											//Infrared communication should be turned off because proximity sensors also use infrared
  Tribot.writePin(sensor, HIGH);		//one sensor on
  //
	//loop to avarage proximity value
	for(uint8_t i=0; i<n; i++) {			
    VCNL4010.write(ConstVCNL4010::COMMAND_REG);
    VCNL4010.write(ConstVCNL4010::MEASUREPROXIMITY);
    VCNL4010.transmission();
    VCNL4010.write(ConstVCNL4010::PROXIMITYDATA_REG);
    VCNL4010.transmission();
    VCNL4010.requestFrom(2);
    uint16_t tmp = 0;
    tmp |= (uint8_t)VCNL4010.read() << 8;			//Read higher byte
    tmp |= (uint8_t)VCNL4010.read();					//read lower byte
    proximity += tmp;													//summarize
  }
	//end of loop
	//
  Tribot.writePin(sensor, LOW);						//sensor off
  proximity /= (unsigned long)n;						//averaged
  loadGIMSK();														//infrared communication on
  return proximity;												//return averaged proximity value			
}
uint8_t calculateDistance(unsigned long proximity, uint8_t sensor) {
	/* This function return ditance by using proximity value and proxi/distance table which is declare at the beggining of this file.
	*/
	uint8_t distanceMm = 150;								//It should be initilized with maximum distance value which sensor can detect
	//first subtract offset from proximity value
	if(Pins::SWI_SENSOR_FORE == sensor) {//fore sensor
    if ( proximity > Offset_SENSOR_FORE ) {
			proximity -= Offset_SENSOR_FORE;
		}
		else {
			proximity = 0;
		}
  } 
  else {//back sensor
    if( proximity > Offset_SENSOR_BACK ) {
			proximity -= Offset_SENSOR_BACK;
		}
    else {
			proximity = 0;
		}
  }
	uint16_t *table = (uint16_t*)PROXI_DISTANCE_TABLE_LYING_YELLOWBLOCK;//table set
	while (1) {
		if(proximity > pgm_read_word(table) ) {//if value bigger than table value
			distanceMm = (uint8_t)pgm_read_word(table+1);
		}
		table += 2;
		if(pgm_read_word(table) == 0xFFFF) {//if reach end of the table
			break;
		}
	}
	return distanceMm;
}
void setTxArray(uint8_t to, uint8_t relay_Flag, uint8_t command, uint8_t message1, uint8_t message2) {
  /*
	* This function is to set infrared message to infrared buf
	*/ 
	uint8_t *tmp = Infrared_Buf;
  
  *tmp++ = Infrareds::BOM;
  *tmp++ = to;
	*tmp++ = relay_Flag;
  *tmp++ = Tribot.getId();
  *tmp++ = Tribot.getRole();
  *tmp++ = command;
  *tmp++ = message1;
  *tmp++ = message2;
  *tmp++ = Infrareds::EOM;
}
void setTxArray(uint8_t to, uint8_t relay_Flag, uint8_t command, uint8_t message1) {
  setTxArray(to, relay_Flag, command, message1, 0);
}
void setTxArray(uint8_t to, uint8_t relay_Flag, uint8_t command) {
  setTxArray(to, relay_Flag, command, 0, 0);
}
uint8_t translateFromInfrared(uint8_t *infrared){
	/*
	* this function is translate command to "real" command
	* what Tribot should do in each command is defined here
	* this function is simple but so hard to read
	*/
  infrared += LEN_INFRARED_BOM;//Idx set to ID_IDX
  if (*infrared != Tribot.getId() && *infrared != Tribot.getRole() && *infrared != ALL) {//if this messages is not for this Tribot
		return false;
  }
  infrared += LEN_INFRARED_HEADER;//Idx set to Command_IDX
  switch (*infrared) {
		/*
		case Infrareds::CHANGE_ROLE:
			Tribot.setRole(*(infrared+1));
			break;*/
		case Infrareds::WRITE_ONE_PIN:
			Tribot.writePin(*(infrared+1),*(infrared+2));
			//setTxArray(Infrared_Buf[IDX_FROM_ID], Infrared_Buf[IDX_RELAY_FLAG], Infrareds::ACK);
			//sendInfrared(Infrared_Buf);
			break;
		case Infrareds::WRITE_TWO_PINS:
			Tribot.writePin(*(infrared+1),*(infrared+2));
			Tribot.writePin(*(infrared+3),*(infrared+4));
			//setTxArray(Infrared_Buf[IDX_FROM_ID], Infrared_Buf[IDX_RELAY_FLAG], Infrareds::ACK);
			//sendInfrared(Infrared_Buf);
			break;
		case Infrareds::GET_OFFSET_FORE_SENSOR:
			setTxArray(Infrared_Buf[IDX_FROM_ID], Infrared_Buf[IDX_RELAY_FLAG], Infrareds::RECEIVE_OFFSET1, (uint8_t)(Offset_SENSOR_FORE >> 8), (uint8_t)Offset_SENSOR_FORE);
			sendInfrared(Infrared_Buf);
			break;
		case Infrareds::GET_OFFSET_BACK_SENSOR:
			setTxArray(Infrared_Buf[IDX_FROM_ID], Infrared_Buf[IDX_RELAY_FLAG], Infrareds::RECEIVE_OFFSET2, (uint8_t)(Offset_SENSOR_BACK >> 8), (uint8_t)Offset_SENSOR_BACK);
			sendInfrared(Infrared_Buf);
			break;
		case Infrareds::SET_OFFSET_FORE_SENSOR:
			Offset_SENSOR_FORE = measureProximityValue(Pins::SWI_SENSOR_FORE, 200); 
			setTxArray(Infrared_Buf[IDX_FROM_ID], Infrared_Buf[IDX_RELAY_FLAG], Infrareds::RECEIVE_OFFSET1, (uint8_t)(Offset_SENSOR_FORE >> 8), (uint8_t)Offset_SENSOR_FORE);
			sendInfrared(Infrared_Buf);
			break;
		case Infrareds::SET_OFFSET_BACK_SENSOR:
			Offset_SENSOR_BACK = measureProximityValue(Pins::SWI_SENSOR_BACK, 200);
			setTxArray(Infrared_Buf[IDX_FROM_ID], Infrared_Buf[IDX_RELAY_FLAG], Infrareds::RECEIVE_OFFSET2, (uint8_t)(Offset_SENSOR_BACK >> 8), (uint8_t)Offset_SENSOR_BACK);
			sendInfrared(Infrared_Buf);
			break;
		case Infrareds::GET_DISTANCE_FORWARD:
			setTxArray(Infrared_Buf[IDX_FROM_ID], Infrared_Buf[IDX_RELAY_FLAG], Infrareds::RECEIVE_DISTANCE, calculateDistance(measureProximityValue(Pins::SWI_SENSOR_FORE, 200), Pins::SWI_SENSOR_FORE));
			sendInfrared(Infrared_Buf);
			break;
		case Infrareds::GET_DISTANCE_BACKWARD:
			setTxArray(Infrared_Buf[IDX_FROM_ID], Infrared_Buf[IDX_RELAY_FLAG], Infrareds::RECEIVE_DISTANCE, calculateDistance(measureProximityValue(Pins::SWI_SENSOR_BACK, 200), Pins::SWI_SENSOR_BACK));
			sendInfrared(Infrared_Buf);
			break;
		case Infrareds::RECEIVE_DISTANCE:
			Distance = *(infrared+1);
			break;
		case Infrareds::CRAWL_FORWARD_AUTO:
			setLocomotion((uint8_t*)Tasks::CRAWL_FORWARD_AUTO);
			break;
		case Infrareds::CRAWL_FORWARD_MANUAL:
			SFlags |= _BV(MLF);
			setLocomotion((uint8_t*)Tasks::CRAWL_FORWARD_MANUAL);
			break;
		/*
		case Infrareds::VERTICAL_JUMP1_MANUAL:
			SFlags |= _BV(MLF);
			setLocomotion((uint8_t*)Tasks::VERTICAL_JUMP1_MANUAL);
			break;
		case Infrareds::VERTICAL_JUMP2_MANUAL:
			SFlags |= _BV(MLF);
			setLocomotion((uint8_t*)Tasks::VERTICAL_JUMP2_MANUAL);
		break;
		*/
		/*
		case Infrareds::CRAWL_BACKWARD_AUTO:
			setLocomotion((uint8_t*)Tasks::CRAWL_BACKWARD_AUTO);
			break;
		case Infrareds::CRAWL_BACKWARD_MANUAL:
			SFlags |= _BV(MLF);
			setLocomotion((uint8_t*)Tasks::CRAWL_BACKWARD_MANUAL);
			break;
		//*/
		case Infrareds::PUSH:
			SFlags |= _BV(PSF);
			Pushing_Threshoud = *(infrared+1);
			setTxArray(LEADER, RELAY, Infrareds::DONE_TASK);
			break;
		/*
		case Infrareds::STOP:
			SFlags &= ~(_BV(FLWF) | _BV(PSF));
			Locomotion = NULL;
			Tribot.stopSMA();
			break;
		case Infrareds::VERTIAL_JUMP_AUTO:
			setLocomotion((uint8_t*)Tasks::VERTICAL_JUMP_AUTO);
			break;
		
		case Infrareds::JUMP_FORWARD_AUTO:
			setLocomotion((uint8_t*)Tasks::JUMP_FORWARD_AUTO);
			break;
		case Infrareds::JUMP_FORWARD_MANUAL:
			setLocomotion((uint8_t*)Tasks::JUMP_FORWARD_MANUAL);
			break;  
		case Infrareds::JUMP_BACKWARD_AUTO:
			setLocomotion((uint8_t*)Tasks::JUMP_BACKWARD_AUTO);
			break;
		case Infrareds::JUMP_BACKWARD_MANUAL:
			setLocomotion((uint8_t*)Tasks::JUMP_BACKWARD_MANUAL);
			break;
		case Infrareds::ROLL_FORWARD1_MANUAL:
			SFlags |= _BV(MLF);
			setLocomotion((uint8_t*)Tasks::ROLL_FORWARD1_MANUAL);
			break;
		case Infrareds::ROLL_FORWARD_AUTO:
			setLocomotion((uint8_t*)Tasks::ROLL_FORWARD_AUTO);
			break; 
		case Infrareds::ROLL_BACKWARD_AUTO:
			setLocomotion((uint8_t*)Tasks::ROLL_BACKWARD_AUTO);
			break;
		case Infrareds::ROLL_BACKWARD_MANUAL:
			setLocomotion((uint8_t*)Tasks::ROLL_BACKWARD_MANUAL);
			break;
		//*/
		/* I don't use these one
		* if you want you can delete
		case Infrareds::SCAN_FORWARD:
			SFlags &= ~_BV(MDRCF);
			SFlags |= _BV(SCNF1);
			break;
		case Infrareds::SCAN_BACKWARD:
			SFlags |= _BV(MDRCF) | _BV(SCNF1);
			break;
		case Infrareds::FOLLOW:
			SFlags |= _BV(FLWF);
			break;
		//*/
    default:
			//blink(Pins::LED_R, 2);
			break; 
  }
  return true;
}
void setLocomotion(uint8_t *locomotion) {
  /*
	*This function start locomotion
	*/
	Locomotion = locomotion;
	saveGIMSK();					//infrared communication off
  if( SFlags & _BV(MLF)) {//when with received parameters
		Locomotion_Idx = (uint8_t)(IDX_MESSAGE1);
		Tribot.writePin(pgm_read_byte(Locomotion), Infrared_Buf[Locomotion_Idx]);
  }
  else {//with pre-set parameters
		Tribot.writePin(pgm_read_byte(Locomotion), pgm_read_byte(Locomotion+1));
  }
  Locomotion_Millis = millis();
}
void blinkLedRed(uint8_t i) {
	saveGIMSK();				//while blinking, in short time, Tribot can do nothing, so communication should be turned off
	Tribot.writePin(Pins::LED_R, LOW);
	i*=2;
	bool value = HIGH;
	for(; i>0; i--) {
		Tribot.writePin(Pins::LED_R, value);
		_delay_ms(100);
		value = !value;
	}
	loadGIMSK();
}
/*Functions written in lower aren't used
* You can delete if you want
* I'm not sure these function work well or not
* Just I kept it
*/
void blink(uint8_t pin, uint8_t i) {
	i = i*2;
	Tribot.writePin(pin, LOW);
	for(; i>0; i--) {
		Tribot.writePin(pin, HIGH);
		_delay_ms(100);
		Tribot.writePin(pin, LOW);
		_delay_ms(100);
	}
}
void followingEvent(void) {
	/*
	* I don't you this function
	*/
  if( SFlags & _BV(DRCF) ) {//backwards
	if ( calculateDistance(measureProximityValue(Pins::SWI_SENSOR_BACK, 200), Pins::SWI_SENSOR_BACK) > Following_Threshold ) {
	  if(Locomotion == NULL) {
		setLocomotion((uint8_t*)Tasks::CRAWL_BACKWARD_AUTO);
	  }
	  Tribot.writePin(Pins::LED_R, LOW);
	}
	else {
	  Tribot.stopSMA();
	  Locomotion = NULL;
	  Tribot.writePin(Pins::LED_R, HIGH);
	}
  }
  else { //forwards
	if ( calculateDistance(measureProximityValue(Pins::SWI_SENSOR_FORE, 200), Pins::SWI_SENSOR_FORE) > Following_Threshold ) {
	  if(Locomotion == NULL) {
		setLocomotion((uint8_t*)Tasks::CRAWL_FORWARD_AUTO);
	  }
	  Tribot.writePin(Pins::LED_R, LOW);
	}
	else {
	  Tribot.stopSMA();
	  Locomotion = NULL;
	  Tribot.writePin(Pins::LED_R, HIGH);
	}
  }
}
void scanningEvent1(void) {
  /*
	* I don't you this function
	*/
	if ( SFlags & _BV(MDRCF)) {//Backward
		if (Locomotion == NULL) {
			setLocomotion((uint8_t*)Tasks::SCAN_BACKWARD);
		}
		SFlags &= ~_BV(SCNF1);
  }
  else {//forward
	if (Locomotion == NULL) {
	  setLocomotion((uint8_t*)Tasks::SCAN_FORWARD);
	}
	SFlags &= ~_BV(SCNF1);
  }
}
void scanningEvent2(void) {
  /*
	* I don't you this function
	*/
	if ( SFlags & _BV(MDRCF)) {//Backward
		if (calculateDistance(measureProximityValue(Pins::SWI_SENSOR_BACK, 200), Pins::SWI_SENSOR_BACK) > Scanning_Threshold) {
			setLocomotion((uint8_t*)Tasks::BLINK_LED_G);
		}
  }
  else {//forward
		if (calculateDistance(measureProximityValue(Pins::SWI_SENSOR_FORE, 200), Pins::SWI_SENSOR_FORE) > Scanning_Threshold) {
			setLocomotion((uint8_t*)Tasks::BLINK_LED_G);
		}
  }
  Tribot.stopSMA();
  SFlags &= ~_BV(SCNF2);
}