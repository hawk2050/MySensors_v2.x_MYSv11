#include <stdint.h>
#include <stdlib.h>

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