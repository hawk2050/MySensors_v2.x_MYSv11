#include <Arduino.h>
#include <UVSensor.hpp>

UVSensor::UVSensor(uint8_t powerPin):
    m_currentUV(0),
    m_lastUV(0),
    m_night_count(0)
{
    m_UV = new UVIS25(powerPin);
}

void UVSensor::init()
{
    m_UV->init();
}

void UVSensor::sleep()
{
    m_UV->applyPower(false);
}

void UVSensor::wake()
{
    m_UV->applyPower(true);
}


float UVSensor::read_sensor()
{
    m_lastUV = m_currentUV;
    uint8_t tempUV = m_UV->readUV();
    uint8_t temp = (tempUV >= 130) ? 130: tempUV;
    m_currentUV = 1.0*temp/10;

    return m_currentUV;
}

float UVSensor::get_uvi()
{
    return m_currentUV;
}

