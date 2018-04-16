#include <cstdint>
#include <WiFiUdp.h>
#include <time.h>

class NanMath {
    public:
        static float nanIfOutOfBounds(float value, float min, float max);
        static float averageIgnoringNan(float value1, float value2);
};