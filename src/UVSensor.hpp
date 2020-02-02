#ifndef _UV_SENSOR_HPP
#define _UV_SENSOR_HPP

#include <stdint.h>
#include <UVIS25.h>

class UVSensor
{
public:
    UVSensor(uint8_t powerPin);

    void init();
    float read_sensor();
    float get_uvi();
    void power_down();
    void power_up();
    bool is_night();


private:
    UVIS25 *m_UV;
    float m_currentUV;
    float m_lastUV;
    uint32_t m_night_count;
    bool m_night;

};

#endif