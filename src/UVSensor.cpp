#include <Arduino.h>
#include <UVSensor.hpp>

UVSensor::UVSensor(uint8_t powerPin):
    m_currentUV(0),
    m_lastUV(0),
    m_night_count(0),
    m_night(false)
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

    /*If UVI is very low then we're assuming it's late in day so readings aren't interesting. This has to happen several times
    in a row before we set our boolean m_night to true*/
    if (m_currentUV < 0.5)
    {
        ++m_night_count; 
        #if DEBUG_RCC
        Serial.print("night_count:");
        Serial.print(night_count, 1);
        Serial.println();
        #endif
        
    }
    else
    {
        /*Some daylight is returning*/
        m_night_count = 0;
        #if DEBUG_RCC
        Serial.print("Reset night_count");
        Serial.println();
        #endif
    }

    if(m_night_count > 10)
    {
        m_night = true;
    }
    else
    {
        m_night = false;
    }
    

    return m_currentUV;
}

bool UVSensor::is_night()
{
   return m_night;
}

float UVSensor::get_uvi()
{
    return m_currentUV;
}

