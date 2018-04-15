
#define byte uint8_t

#include <ESP8266WiFi.h>
// #include <ESP8266WiFiMulti.h>
#include <WiFiUdp.h>
#include <TobyNtp.h>

IPAddress timeServerIP;

const int NTP_PACKET_SIZE = 48;  // NTP time stamp is in the first 48 bytes of the message
byte NTPBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets

TobyNtp::TobyNtp(WiFiUDP udp, char* ntpHostname) :udp(udp), hostname(hostname) {
    WiFi.hostByName(ntpHostname, timeServerIP); // Get the IP address of the NTP server
}

uint32_t TobyNtp::getNewNtpTime() {
    if (udp.parsePacket() == 0) { // If there's no response (yet)
        return 0;
    }
    udp.read(NTPBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    // Combine the 4 timestamp bytes into one 32-bit number
    uint32_t NTPTime = (NTPBuffer[40] << 24) | (NTPBuffer[41] << 16) | (NTPBuffer[42] << 8) | NTPBuffer[43];
    // Convert NTP time to a UNIX timestamp:
    uint32_t UNIXTime = NTPTime - SEVENTY_YEARS_IN_SECONDS; // Not sure why, but the time stamp is in EST??
    return UNIXTime;
}

uint32_t TobyNtp::getTime() {
    uint32_t newNtpTimeInSeconds = getNewNtpTime();
    if (newNtpTimeInSeconds != 0) {
        millisAtLastNtpResponse = millis();
        mostRecentNtpTimeInSeconds = newNtpTimeInSeconds;
        return mostRecentNtpTimeInSeconds;
    }

    if (mostRecentNtpTimeInSeconds == 0) {
        return 0;
    }

    uint32_t secondsSinceLastNtpResponse = (millis() - millisAtLastNtpResponse) / 1000;
    return mostRecentNtpTimeInSeconds + secondsSinceLastNtpResponse;
}

void TobyNtp::sendNTPpacket(bool force) {
    if (!force && millis() - previousNtpRequestMillis <= POLLING_INTERVAL) return;

    previousNtpRequestMillis = millis();

    memset(NTPBuffer, 0, NTP_PACKET_SIZE);  // set all bytes in the buffer to 0
    // Initialize values needed to form NTP request
    NTPBuffer[0] = 0b11100011;   // LI, Version, Mode
    // send a packet requesting a timestamp:
    udp.beginPacket(timeServerIP, 123); // NTP requests are to port 123
    udp.write(NTPBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
}

inline int TobyNtp::getSeconds(uint32_t UNIXTime) {
  return UNIXTime % 60;
}

inline int TobyNtp::getMinutes(uint32_t UNIXTime) {
  return UNIXTime / 60 % 60;
}

inline int TobyNtp::getHours(uint32_t UNIXTime) {
  return UNIXTime / 3600 % 24;
}