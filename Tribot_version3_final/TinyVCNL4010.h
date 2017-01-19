#ifndef TinyVCNL4010_h
#define TinyVCNL4010_h
//
//include
//
#include <stdint.h>
#include "wiring.h"
//
//Const
//
struct ConstVCNL4010{
	static const uint8_t USI_SEND = 0;
	static const uint8_t USI_RCVE = 1;
	static const uint8_t USI_BUF_SIZE = 3;
	
	static const uint8_t ADDRESS = 0x13;
	static const uint8_t COMMAND_REG = 0x80;
	static const uint8_t PRUDUCTID_REG = 0x81;
	static const uint8_t PROXRATE_REG = 0x82;
	static const uint8_t IRCURRENT_REG = 0x83;
	static const uint8_t PROXIMITYDATA_REG = 0x87;//and 0x88
	static const uint8_t PROXINITYADJUST_REG = 0x8A;
	
	static const uint8_t MEASUREAMBIENT = 0x10;
	static const uint8_t MEASUREPROXIMITY = 0x08;
	static const uint8_t AMBIENTREADY = 0x40;
	static const uint8_t PROXIMITYREADY = 0x20;
	static const uint8_t PRUDUCTID = 0x21;
	
	static const uint8_t IRCURRENT = 20;
};
struct MessagesVCNL4010 {
	static const uint8_t BEGIN[] PROGMEM;
	static const uint8_t LEN_BEGIN;
	static const uint8_t GET_PROXI_VALUE1[] PROGMEM;
	static const uint8_t LEN_GET_PROXI_VALUE1;
	static const uint8_t GET_PROXI_VALUE2[] PROGMEM;
	static const uint8_t LEN_GET_PROXI_VALUE2;
}; 

class TinyVCNL4010 {
private:
	  static uint8_t USI_Buf[];           // holds I2C send and receive data
	  static uint8_t USI_BufIdx;          // current number of bytes in the send buff
	  static uint8_t USI_LastRead;        // number of bytes read so far
	  static uint8_t USI_BytesAvail;      // number of bytes requested but not read
	  static uint16_t OFFSET;
public:
	TinyVCNL4010();
	void begin(void);
	uint16_t convertTo16bit(uint8_t value[]);
    uint8_t check(void);
	uint16_t measureDistance(void);
	void	write(uint8_t);
	void transmission();
	void requestFrom(uint8_t);
	uint8_t read();
	uint8_t available();
};

extern TinyVCNL4010 VCNL4010;
#endif /* TinyVCNL4010_h */