#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <WiFiUdp.h>


#define LED_PIN 2
#define DHT_1_PIN 5 // D1 - The GPIO of the first DHT22
#define DHT_2_PIN 4 // D2 - The GPIO the DHT22 is connected to
#define DHT_TYPE DHT22  // The specific type of sensor

#define HEAT_PIN 13 // D7 - The GPIO to turn the heat on or off
#define FAN_PIN 15 // D8 - The GPIO to turn the fan on or off

DHT dht1(DHT_1_PIN, DHT_TYPE);
DHT dht2(DHT_2_PIN, DHT_TYPE);

const char* mqtt_server = "192.168.1.135";
char* ssid = "Love Shack";
const char* password = "pachamama";
const char* TEMPERATURE_TOPIC_1 = "desk/temperature/1";
const char* HUMIDITY_TOPIC_1 = "desk/humidity/1";
const char* TEMPERATURE_TOPIC_2 = "desk/temperature/2";
const char* HUMIDITY_TOPIC_2 = "desk/humidity/2";
const char* HEATER_TOPIC = "desk/heater";
const char* HEATER_TREND_TOPIC = "desk/heater/trend";
const int MILLIS_BETWEEN_DHT_READ = 5000;

WiFiUDP UDP;                     // Create an instance of the WiFiUDP class to send and receive
IPAddress timeServerIP;          // time.nist.gov NTP server address
const char* NTPServerName = "0.ca.pool.ntp.org";
const int NTP_PACKET_SIZE = 48;  // NTP time stamp is in the first 48 bytes of the message
byte NTPBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets

WiFiClient espClient;
PubSubClient client(espClient);
long timeOfLastDhtRead = 0;
bool temperatureRisinboolean g = true;

typedef struct MqttMessage {
  char topic[128];
  char body[128];
  bool retained;
};
MqttMessage mqttMessage;

void set_up_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  Serial.print("Wifi status is: ");
  Serial.println(WiFi.status());

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world, I'm connected");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void startUDP() {
  Serial.println("Starting UDP");
  UDP.begin(123);                          // Start listening for UDP messages on port 123
  Serial.print("Local port:\t");
  Serial.println(UDP.localPort());
  Serial.println();
}



uint32_t getTime() {
  if (UDP.parsePacket() == 0) { // If there's no response (yet)
    return 0;
  }
  UDP.read(NTPBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
  // Combine the 4 timestamp bytes into one 32-bit number
  uint32_t NTPTime = (NTPBuffer[40] << 24) | (NTPBuffer[41] << 16) | (NTPBuffer[42] << 8) | NTPBuffer[43];
  // Convert NTP time to a UNIX timestamp:
  // Unix time starts on Jan 1 1970. That's 2208988800 seconds in NTP time:
  const uint32_t seventyYears = 2208988800UL;
  // subtract seventy years:
  uint32_t UNIXTime = NTPTime - seventyYears;
  return UNIXTime;
}

void sendNTPpacket(IPAddress& address) {
  memset(NTPBuffer, 0, NTP_PACKET_SIZE);  // set all bytes in the buffer to 0
  // Initialize values needed to form NTP request
  NTPBuffer[0] = 0b11100011;   // LI, Version, Mode
  // send a packet requesting a timestamp:
  UDP.beginPacket(address, 123); // NTP requests are to port 123
  UDP.write(NTPBuffer, NTP_PACKET_SIZE);
  UDP.endPacket();
}

inline int getSeconds(uint32_t UNIXTime) {
  return UNIXTime % 60;
}

inline int getMinutes(uint32_t UNIXTime) {
  return UNIXTime / 60 % 60;
}

inline int getHours(uint32_t UNIXTime) {
  return UNIXTime / 3600 % 24;
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(2000);

  // initialize digital pin 13 as an output.
  pinMode(LED_PIN, OUTPUT);
  pinMode(HEAT_PIN, OUTPUT);
  Serial.println("Device Started");
  Serial.println("-------------------------------------");
  Serial.println("Running DHT!");
  Serial.println("-------------------------------------");
  set_up_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  if (!WiFi.hostByName(NTPServerName, timeServerIP)) { // Get the IP address of the NTP server
    Serial.println("DNS lookup failed. Rebooting.");
    Serial.flush();
    ESP.reset();
  }

  startUDP();

  Serial.print("Time server IP:\t");
  Serial.println(timeServerIP);

  Serial.println("\r\nSending NTP request ...");
  sendNTPpacket(timeServerIP);
  dht1.begin();
  dht2.begin();
}

float readTemperatureAndHumidity(long lastUnixTimestamp, long lastNTPResponse, DHT dht) {
  long start = millis();
  timeOfLastDhtRead = start;
  float temperatureCelsius = readTemperature(lastUnixTimestamp, lastNTPResponse, dht);
  readHumidity(lastUnixTimestamp, lastNTPResponse, dht);
  long end = millis();
  Serial.print("Reading temperature and humidity took ");
  Serial.print(end - start);
  Serial.println("ms");
  return temperatureCelsius;
}

float readTemperature(long lastUnixTimestamp, long lastNTPResponse, DHT dht) {
  // Read temperature as Celsius (the default)
  float temperatureCelsius = dht.readTemperature();

  // Check if read failed and exit early
  if ( isnan(temperatureCelsius) ) {
    Serial.println("Failed to read temperature from DHT sensor!");
    return sqrt(-1);
  }

  long now = millis();
  strcpy(mqttMessage.topic, TEMPERATURE_TOPIC_1);
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& messageBody = jsonBuffer.createObject();
  messageBody["degreesCelsius"] = temperatureCelsius;
  messageBody["secondsSinceEpoch"] = lastUnixTimestamp + (now - lastNTPResponse) / 1000;
  messageBody.printTo(mqttMessage.body, 128);
  client.publish(mqttMessage.topic, mqttMessage.body);
  return temperatureCelsius;
}

void readHumidity(long lastUnixTimestamp, long lastNTPResponse, DHT dht) {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float relativeHumidity = dht.readHumidity();

  // Check if read failed and exit early
  if ( isnan(relativeHumidity) ) {
    Serial.println("Failed to read humidity from DHT sensor!");
    return;
  }

  long now = millis();
  strcpy(mqttMessage.topic, HUMIDITY_TOPIC_1);
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& messageBody = jsonBuffer.createObject();
  messageBody["relativeHumidity"] = relativeHumidity;
  messageBody["secondsSinceEpoch"] = lastUnixTimestamp + (now - lastNTPResponse) / 1000;
  messageBody.printTo(mqttMessage.body, 128);
  client.publish(mqttMessage.topic, mqttMessage.body);
}

void adjustHeat(float temperatureCelsius, uint32_t actualTime) {
  if (temperatureRising) {
    if (temperatureCelsius <= 37.9) {
      digitalWrite(HEAT_PIN, LOW);
      strcpy(mqttMessage.topic, HEATER_TOPIC);

      StaticJsonBuffer<200> jsonBuffer;
      JsonObject& messageBody = jsonBuffer.createObject();
      messageBody["heating"] = true;
      messageBody["secondsSinceEpoch"] = actualTime;
      messageBody.printTo(mqttMessage.body, 128);
      client.publish(mqttMessage.topic, mqttMessage.body);
    } else {
      digitalWrite(HEAT_PIN, HIGH);
      temperatureRising = false;

      strcpy(mqttMessage.topic, HEATER_TOPIC);
      StaticJsonBuffer<200> jsonBuffer;
      JsonObject& messageBody = jsonBuffer.createObject();
      messageBody["heating"] = false;
      messageBody["secondsSinceEpoch"] = actualTime;
      messageBody.printTo(mqttMessage.body, 128);
      client.publish(mqttMessage.topic, mqttMessage.body);

      strcpy(mqttMessage.topic, HEATER_TREND_TOPIC);
      JsonObject& messageBody2 = jsonBuffer.createObject();
      messageBody2["temperatureRising"] = false;
      messageBody2["secondsSinceEpoch"] = actualTime;
      messageBody2.printTo(mqttMessage.body, 128);
      client.publish(mqttMessage.topic, mqttMessage.body);
    }
  } else {
    if (temperatureCelsius <= 37.7) {
      digitalWrite(HEAT_PIN, LOW);
      temperatureRising = true;

      StaticJsonBuffer<200> jsonBuffer;
      JsonObject& messageBody = jsonBuffer.createObject();
      messageBody["heating"] = true;
      messageBody["secondsSinceEpoch"] = actualTime;
      messageBody.printTo(mqttMessage.body, 128);
      client.publish(mqttMessage.topic, mqttMessage.body);

      strcpy(mqttMessage.topic, HEATER_TREND_TOPIC);
      JsonObject& messageBody2 = jsonBuffer.createObject();
      messageBody2["temperatureRising"] = true;
      messageBody2["secondsSinceEpoch"] = actualTime;
      messageBody2.printTo(mqttMessage.body, 128);
      client.publish(mqttMessage.topic, mqttMessage.body);
    } else {
      digitalWrite(HEAT_PIN, HIGH);

      strcpy(mqttMessage.topic, HEATER_TOPIC);
      StaticJsonBuffer<200> jsonBuffer;
      JsonObject& messageBody = jsonBuffer.createObject();
      messageBody["heating"] = false;
      messageBody["secondsSinceEpoch"] = actualTime;
      messageBody.printTo(mqttMessage.body, 128);
      client.publish(mqttMessage.topic, mqttMessage.body);
    }
  }
}

unsigned long ntpPollingInterval = 10 * 60 * 1000; // Request NTP time every 10 minutes
unsigned long previousNTP = 0;
unsigned long lastNTPResponse = millis();
uint32_t secondsSinceEpoch = 0;


// the loop function runs over and over again forever
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();

  if (now - previousNTP > ntpPollingInterval) { // If a minute has passed since last NTP request
    previousNTP = now;
    Serial.println("\r\nSending NTP request ...");
    sendNTPpacket(timeServerIP);               // Send an NTP request
  }

  uint32_t time = getTime();                   // Check if an NTP response has arrived and get the (UNIX) time

  if (time) {                                  // If a new timestamp has been received
    secondsSinceEpoch = time;
    Serial.print("NTP response:\t");
    Serial.println(secondsSinceEpoch);
    lastNTPResponse = now;
  } else if ((now - lastNTPResponse) > 3600000) {
    Serial.println("More than 1 hour since last NTP response. Rebooting.");
    Serial.flush();
    ESP.reset();
  }

  uint32_t actualTime = secondsSinceEpoch + (now - lastNTPResponse) / 1000;

  if ( (now - timeOfLastDhtRead) > MILLIS_BETWEEN_DHT_READ && secondsSinceEpoch != 0) {
    float temperatureCelsius = readTemperatureAndHumidity(secondsSinceEpoch, lastNTPResponse, dht1);
    if ( !isnan(temperatureCelsius) ) {
      adjustHeat(temperatureCelsius, actualTime);
    }
  }

  digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
}

