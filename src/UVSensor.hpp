#include <stdint.h>
#include <UVIS25.h>

class UVSensor
{
    public:
    UVSensor(uint8_t powerPin);

    void init();

    float read_sensor();
    float get_uvi();

    void sleep();
    void wake();


    private:
    UVIS25 *m_UV;
    
    float m_currentUV;
    float m_lastUV;
    uint32_t m_night_count;

};