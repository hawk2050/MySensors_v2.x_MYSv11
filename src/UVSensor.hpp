#include <stdint.h>
#include <UVIS25.h>

class UVSensor
{
    public:
    UVSensor(uint8_t powerPin);

    void init();

    float get_uv_reading();

    void sleep();
    void wake();


    private:
    UVIS25 *m_UV;
    
    float m_currentUV;
    float m_lastUV;
    uint32_t m_night_count;

};