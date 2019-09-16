#include <stdint.h>
#include <stdlib.h>

/*Wired up Port D, Pin 2 as the voltage supply to the UVIS25 so that it can be placed in low power down mode
when the AVR is sleeping. So we want a power up and a power down method*/

#ifndef UVIS25_h
#define UVIS25_h

class UVIS25 {
public:
	static uint8_t sensorAddress;
	uint8_t init(void);
	uint8_t readUV(void);
	uint8_t readReg(uint8_t);
	void readReg(uint8_t, uint8_t *, uint8_t);
	uint8_t writeReg(uint8_t, uint8_t);
	uint8_t writeReg(uint8_t, uint8_t *, size_t);
	void applyPower(bool enable=true);
	};
	
#endif

#ifndef aistin_h
#define aistin_h

class aistin {
private:
	
public:
	static uint8_t readReg(uint8_t, uint8_t);
	static uint8_t readReg(uint8_t, uint8_t, uint8_t *, uint8_t, bool autoIncrement=true);
	static uint8_t writeReg(uint8_t, uint8_t, uint8_t);
	static uint8_t writeReg(uint8_t, uint8_t, uint8_t *, size_t, bool autoIncrement=true);
};

#endif