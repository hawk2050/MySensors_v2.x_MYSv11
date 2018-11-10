#include <UVIS25.h>
#include <Wire.h>

uint8_t UVIS25::sensorAddress = 0x47;

uint8_t UVIS25::init(void)
{
	uint8_t ctrl_reg_values[3] = {0x01, 0x00, 0x00};
	return writeReg(0x20, ctrl_reg_values, sizeof(ctrl_reg_values));
}
uint8_t UVIS25::readUV()
{
	return readReg(0x28); 
}
uint8_t UVIS25::readReg(uint8_t regAddress)
{
	return aistin::readReg(sensorAddress, regAddress);
}

void UVIS25::readReg(uint8_t regAddress, uint8_t *regValue, uint8_t quanity)
{
	regAddress = 0x80 | regAddress; //set MSB to 1 to auto increment
	aistin::readReg(sensorAddress, regAddress, regValue, quanity);
}
uint8_t UVIS25::writeReg(uint8_t regAddress, uint8_t regValue)
{
	return aistin::writeReg(sensorAddress, regAddress, regValue);
}
uint8_t UVIS25::writeReg(uint8_t regAddress, uint8_t *regValue, size_t quanity)
{
	regAddress = 0x80 | regAddress; //set MSB to 1 to auto increment
	return aistin::writeReg(sensorAddress, regAddress, regValue, quanity);
}

uint8_t aistin::readReg(uint8_t sensorAddress, uint8_t regAddress)
{
	uint8_t regValue;
	Wire.beginTransmission(sensorAddress);
    Wire.write(regAddress);
    Wire.endTransmission();
    Wire.requestFrom(sensorAddress, (uint8_t)1);
	regValue = Wire.read();
	Wire.endTransmission(true);
	return regValue;
}

uint8_t aistin::readReg(uint8_t sensorAddress, uint8_t regAddress, uint8_t *regValue, uint8_t quanity, bool autoIncrement)
{
	Wire.beginTransmission(sensorAddress);
	if(autoIncrement){
		Wire.write(regAddress);
		Wire.endTransmission(false);
		Wire.requestFrom(sensorAddress, quanity);
		for(int i=0 ; i<quanity ; i++)
			regValue[i] = Wire.read();
	}
	else {
		for(size_t i=0 ; i<quanity ; i++){
			Wire.write(regAddress+i);
			Wire.endTransmission(false);
			Wire.requestFrom(sensorAddress, (uint8_t)1);
			regValue[i] = Wire.read();
			if( i<(quanity-1) ){
				Wire.endTransmission(false);
				Wire.beginTransmission(sensorAddress);
			}
		}
	}
	
	return	Wire.endTransmission(true);
}

uint8_t aistin::writeReg(uint8_t sensorAddress, uint8_t regAddress, uint8_t *regValue, size_t quanity, bool autoIncrement)
{
	Wire.beginTransmission(sensorAddress);
	if(autoIncrement) {
		Wire.write(regAddress);
		Wire.write(regValue, quanity);
	}
	else {
		for(size_t i=0 ; i<quanity ; i++){
			Wire.write(regAddress+i);
			Wire.write(regValue[i]);
			if( i<(quanity-1) ){
				Wire.endTransmission(false);
				Wire.beginTransmission(sensorAddress);
			}
		}
	}
	return Wire.endTransmission(true);
}
uint8_t aistin::writeReg(uint8_t sensorAddress, uint8_t regAddress, uint8_t regValue)
{
	Wire.beginTransmission(sensorAddress);
	Wire.write(regAddress);
	Wire.write(regValue);
	return Wire.endTransmission(true);
}