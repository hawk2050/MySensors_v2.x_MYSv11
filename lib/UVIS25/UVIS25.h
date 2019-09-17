#include <stdint.h>
#include <stdlib.h>

/*Wired up Port D, Pin 2 as the voltage supply to the UVIS25 so that it can be placed in low power down mode
when the AVR is sleeping. So we want a power up and a power down method*/

#ifndef UVIS25_h
#define UVIS25_h

#ifdef __cplusplus
#include <Arduino.h>
#endif

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__) || defined (__AVR_ATmega168__)
#if defined(__AVR_ATmega328PB__)
#define __digitalPinToPortReg(__pin)	(((__pin) <= 7) ? &PORTD : (((__pin) >= 8 && (__pin) <= 13) ? &PORTB : (((__pin) >= 14 && (__pin) <= 19) ? &PORTC : &PORTE)))
#define __digitalPinToDDRReg(__pin)		(((__pin) <= 7) ? &DDRD : (((__pin) >= 8 && (__pin) <= 13) ? &DDRB : (((__pin) >= 14 && (__pin) <= 19) ? &DDRC : &DDRE)))
#define __digitalPinToPINReg(__pin)		(((__pin) <= 7) ? &PIND : (((__pin) >= 8 && (__pin) <= 13) ? &PINB : (((__pin) >= 14 && (__pin) <= 19) ? &PINC : &PINE)))
#define __digitalPinToBit(__pin)		(((__pin) <= 7) ? (__pin) : (((__pin) >= 8 && (__pin) <= 13) ? (__pin) - 8 : (((__pin) >= 14 && (__pin) <= 19) ? (__pin) - 14 : (((__pin) >= 20 && (__pin) <= 21) ? (__pin) - 18 : (__pin) - 22))))
#else
#define __digitalPinToPortReg(__pin)	(((__pin) <= 7) ? &PORTD : (((__pin) >= 8 && (__pin) <= 13) ? &PORTB : &PORTC))
#define __digitalPinToDDRReg(__pin)		(((__pin) <= 7) ? &DDRD : (((__pin) >= 8 && (__pin) <= 13) ? &DDRB : &DDRC))
#define __digitalPinToPINReg(__pin)		(((__pin) <= 7) ? &PIND : (((__pin) >= 8 && (__pin) <= 13) ? &PINB : &PINC))
#define __digitalPinToBit(__pin)		(((__pin) <= 7) ? (__pin) : (((__pin) >= 8 && (__pin) <= 13) ? (__pin) - 8 : (__pin) - 14))
#endif
#define digitalWriteFast(__pin, __value) do { if (__builtin_constant_p(__pin) && __builtin_constant_p(__value)) { bitWrite(*__digitalPinToPortReg(__pin), (uint8_t)__digitalPinToBit(__pin), (__value)); } else { digitalWrite((__pin), (__value)); } } while (0)
#define pinModeFast(__pin, __mode) do { if (__builtin_constant_p(__pin) && __builtin_constant_p(__mode) && (__mode!=INPUT_PULLUP)) { bitWrite(*__digitalPinToDDRReg(__pin), (uint8_t)__digitalPinToBit(__pin), (__mode)); } else { pinMode((__pin), (__mode)); } } while (0)
#define digitalReadFast(__pin) ( (bool) (__builtin_constant_p(__pin) ) ? (( bitRead(*__digitalPinToPINReg(__pin), (uint8_t)__digitalPinToBit(__pin))) ) : digitalRead((__pin)) )
#else
// for all other archs use built-in pin access functions
#define digitalWriteFast(__pin, __value) digitalWrite(__pin, __value)
#define pinModeFast(__pin, __value) pinMode(__pin, __value)
#define digitalReadFast(__pin) digitalRead(__pin)
#endif

// Define these as macros to save valuable space
#define hwDigitalWrite(__pin, __value) digitalWriteFast(__pin, __value)
#define hwDigitalRead(__pin) digitalReadFast(__pin)
#define hwPinMode(__pin, __value) pinModeFast(__pin, __value)

class UVIS25
{
public:
	UVIS25(uint8_t powerPin);
	static uint8_t sensorAddress;
	uint8_t init(void);
	uint8_t readUV(void);
	uint8_t readReg(uint8_t);
	void readReg(uint8_t, uint8_t *, uint8_t);
	uint8_t writeReg(uint8_t, uint8_t);
	uint8_t writeReg(uint8_t, uint8_t *, size_t);
	void applyPower(bool enable=true);
protected:
	uint8_t _powerPin;

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