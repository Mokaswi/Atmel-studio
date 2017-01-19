//
//include
//
#include "Tiny_USI_TWI_Master.h"
#include "TinyVCNL4010.h"
//
//global variable
//
uint8_t TinyVCNL4010::USI_Buf[ConstVCNL4010::USI_BUF_SIZE] = {(ConstVCNL4010::ADDRESS<<TWI_ADR_BITS) | ConstVCNL4010::USI_SEND};             // holds I2C send and receive data
uint8_t TinyVCNL4010::USI_BufIdx = 0;                    // current number of bytes in the send buff
uint8_t TinyVCNL4010::USI_LastRead = 0;                  // number of bytes read so far
uint8_t TinyVCNL4010::USI_BytesAvail = 0;                // number of bytes requested but not read
//
//Constructor
//
TinyVCNL4010::TinyVCNL4010() {
	USI_TWI_Master_Initialise();
}
void TinyVCNL4010::begin(){
	write(ConstVCNL4010::IRCURRENT_REG);
	write(ConstVCNL4010::IRCURRENT);
	transmission();
}
void TinyVCNL4010::write(uint8_t data){ // buffers up data to send
	USI_BufIdx++;
	USI_Buf[USI_BufIdx] = data;                                   // inc for next byte in buffer
}
void TinyVCNL4010::transmission() {
	USI_TWI_Start_Read_Write(USI_Buf,USI_BufIdx+1); // core func that does the work
	USI_BufIdx = 0;
	USI_TWI_Master_Stop();
}
void TinyVCNL4010::requestFrom(uint8_t numBytes){ // setup for receiving from slave
	USI_LastRead = 0;
	USI_BytesAvail = numBytes; // save this off in a global
	numBytes++;                // add extra byte to transmit header
	USI_Buf[0] |= ConstVCNL4010::USI_RCVE;   // setup address & Rcve bit
	USI_TWI_Start_Read_Write(USI_Buf, numBytes); // core func that does the work
	// USI_Buf now holds the data read
	USI_BufIdx = 0;
	USI_Buf[0] &= ~ConstVCNL4010::USI_RCVE;
	USI_TWI_Master_Stop();
}
uint8_t TinyVCNL4010::read(){ // returns the bytes received one at a time
	USI_LastRead++;     // inc first since first uint8_t read is in USI_Buf[1]
	return USI_Buf[USI_LastRead];
}
uint8_t TinyVCNL4010::available(){ // the bytes available that haven't been read yet
	return USI_BytesAvail - (USI_LastRead);
}
uint8_t TinyVCNL4010::check(void) {
	write(ConstVCNL4010::PRUDUCTID_REG);
	transmission();
	requestFrom(1);
	if( read() ==  ConstVCNL4010::PRUDUCTID)
		return ConstVCNL4010::PRUDUCTID;
	else
		return 0;
}
uint16_t TinyVCNL4010::measureDistance(void){
	uint16_t distance = 0;
	
	write(ConstVCNL4010::COMMAND_REG);
	write(ConstVCNL4010::MEASUREPROXIMITY);
	transmission();
	write(ConstVCNL4010::PROXIMITYDATA_REG);
	transmission();
	requestFrom(2);

	distance = (uint16_t) read() << 8;
	distance |= (uint16_t)read();			
	return distance;
}
//
//Preinstantiate Objects 
//
TinyVCNL4010 VCNL4010 = TinyVCNL4010();