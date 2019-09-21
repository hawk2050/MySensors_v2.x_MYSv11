#include <stdint.h>
#include <UVIS25.h>

class UVSensor
{
    public:
    UVSensor(uint8_t powerPin);


    private:
    uint8_t currentUV;
    uint8_t lastUV;
}