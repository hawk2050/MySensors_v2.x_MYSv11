#ifndef _BATTERY_SENSE_HPP
#define _BATTERY_SENSE_HPP

#include <stdint.h>

class BatteryLevel
{
    public:
    BatteryLevel();

    uint16_t getVoltage(void);

    private:
    uint16_t readVcc(void);
    uint16_t m_lastVoltage;

};
#endif