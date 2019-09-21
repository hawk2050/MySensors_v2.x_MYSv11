#include <stdint.h>
#include <UVIS25.h>

class UVSensor
{
    public:
    UVSensor(uint8_t powerPin);

    float get_uv_reading();


    private:
    uint8_t m_currentUV;
    uint8_t m_lastUV;
};