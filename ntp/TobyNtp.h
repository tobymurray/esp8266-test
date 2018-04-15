#include <cstdint>
#include <WiFiUdp.h>

class TobyNtp {
    const unsigned long POLLING_INTERVAL = 10 * 60 * 1000; // Request NTP time every 10 minutes
    // Unix time starts on Jan 1 1970. That's 2208988800 seconds in NTP time:
    const uint32_t SEVENTY_YEARS_IN_SECONDS = 2208988800UL;

    uint32_t millisAtLastNtpResponse = 0;
    uint32_t mostRecentNtpTimeInSeconds = 0;
    unsigned long previousNtpRequestMillis = 0;
    WiFiUDP udp;
    char* hostname;
    IPAddress timeServerIP;

    public:
        TobyNtp(WiFiUDP udp, char* hostname);
        uint32_t getTime();
        void sendNTPpacket(bool force = false);
    private:
        uint32_t getNewNtpTime();
        int getSeconds(uint32_t UNIXTime);
        int getMinutes(uint32_t UNIXTime);
        int getHours(uint32_t UNIXTime);
};