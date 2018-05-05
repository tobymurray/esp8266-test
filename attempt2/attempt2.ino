// DHT22 Sensor library
#include <dht.h>

// Connect to local WiFi LAN
#include <ESP8266WiFi.h>

// Use UDP for time server
#include <WiFiUdp.h>

// Local LAN configuration
#include <LanConfiguration.h>
const char* mqtt_server = "192.168.1.2";

// For MQTT client
#include <PubSubClient.h>

// Helper class for getting real time
#include <TobyNtp.h>

#include <NanMath.h>

// For data about the ESP8266 itself (memory/VCC)
#include <Esp.h>

#define LED_PIN D4
#define HEAT_PIN D6 // The GPIO to turn the heat on or off
#define HUMIDITY_PIN D7 // The GPIO to turn the mister on or off
#define DHT22_PIN_1 D1
#define DHT22_PIN_2 D2
#define SENSOR_POWER_PIN D3
#define BUTTON_PIN D5

// Values to bound what constitutes a correct sensor reading
const float MAX_REASONABLE_TEMPERATURE = 50;
const float MIN_REASONABLE_TEMPERATURE = 10;

// Throttle sensor reads to avoid polling too frequently
const unsigned int MIN_SENSOR_READ_MILLIS = 2500;

// Definition of window of acceptable temperature - avoids constantly flicking heat on and off
const float MAX_TEMPERATURE = 37.9;
const float MIN_TEMPERATURE = 37.7;

const float MAX_HUMIDITY = 65;
const float MIN_HUMIDITY = 55;

const char* ssid = SSID;
const char* password = PASSWORD;

const char* TEMPERATURE_TOPIC_1 = "desk/temperature/1";
const char* TEMPERATURE_TOPIC_2 = "desk/temperature/2";
const char* TEMPERATURE_TOPIC_AVERAGE = "desk/temperature/average";
const char* HUMIDITY_TOPIC_1 = "desk/humidity/1";
const char* HUMIDITY_TOPIC_2 = "desk/humidity/2";
const char* HUMIDITY_TOPIC_AVERAGE = "desk/humidity/average";
const char* FREE_MEMORY_TOPIC = "desk/freeMemory";
const char* ELAPSED_TIME_IN_SECONDS_TOPIC = "desk/elapsedSeconds";

const unsigned long DEBOUNCE_MILLIS = 50;

bool temperatureRising = true;
bool humidityRising = true;

int consecutiveFailedTemperatureReads = 0;
int consecutiveSensorTimeouts = 0;

int cyclesInHeatCoolLoop = 0;
int cyclesToHeatToMax = 0;
int cyclesToCoolToMin = 0;

EspClass esp;
dht DHT;

unsigned long unixTimeAtStart;
unsigned long lastDebounceTime = 0;
int buttonState;
int lastButtonState = HIGH;
unsigned long lastSensorReadMillis = 0;
bool boardLedState = false;

struct statistics {
  uint32_t total;
  uint32_t ok;
  uint32_t crc_error;
  uint32_t time_out;
  uint32_t unknown;
};

struct sensorReading {
  float temperatureCelsius;
  float relativeHumidity;
  long microsToRead; 
};

typedef struct MqttMessage {
  char topic[128];
  char body[128];
  bool retained;
};
MqttMessage mqttMessage;

statistics sensor1Stats = { 0,0,0,0,0 };
statistics sensor2Stats = { 0,0,0,0,0 };

WiFiUDP UDP;
TobyNtp* ntp;

WiFiClient wiFiClient;
PubSubClient mqttClient(wiFiClient);

time_t ntpTime;

struct tm * timeinfo;

void printStatistics(int sensorNumber, statistics& stats) {
  Serial.print(sensorNumber);
  Serial.print(": ");
  Serial.print(stats.total);
  Serial.print("\t\t\t");
  Serial.print(stats.ok);
  Serial.print("\t\t\t");
  Serial.print(stats.crc_error);
  Serial.print("\t\t\t");
  Serial.print(stats.time_out);
  Serial.print("\t\t\t");
  Serial.print(stats.unknown);
  Serial.print("\t\t\t");
  Serial.print(esp.getFreeHeap());
  Serial.println("");
}

sensorReading readSensor(int sensorNumber, uint8_t dhtGpio, statistics& stats) {
    uint32_t start = micros();
    int checkStatus = DHT.read22(dhtGpio);
    uint32_t stop = micros();

    stats.total++;
    switch (checkStatus) {
    case DHTLIB_OK:
        stats.ok++;
        consecutiveSensorTimeouts = 0;
        return { DHT.temperature, DHT.humidity, stop - start };
    case DHTLIB_ERROR_CHECKSUM:
        Serial.print("Sensor ");
        Serial.print(sensorNumber);
        Serial.print(": checksum error on read #");
        Serial.println(stats.total);
        stats.crc_error++;
        consecutiveSensorTimeouts = 0;
        break;
    case DHTLIB_ERROR_TIMEOUT:
        Serial.print("Sensor ");
        Serial.print(sensorNumber);
        Serial.print(": timeout error on read #");
        Serial.println(stats.total);
        stats.time_out++;
        consecutiveSensorTimeouts++;
        break;
    default:
        Serial.print("Sensor ");
        Serial.print(sensorNumber);
        Serial.print(": unknown error on read #");
        Serial.println(stats.total);
        stats.unknown++;
        consecutiveSensorTimeouts = 0;
        break;
    }

    // On any error conditions, return a neutered value
    return { NAN, NAN, stop - start };
}

void printAverages(sensorReading averages, sensorReading reading1, sensorReading reading2) {
  float temperatureDifference = fabsf(reading1.temperatureCelsius - reading2.temperatureCelsius);
  float humidityDifference = fabsf(reading1.relativeHumidity - reading2.relativeHumidity);

  if (ntpTime) {
    Serial.print(ntpTimeToString());
    Serial.print(": ");
  }

  Serial.print("average temperature: ");
  Serial.print(averages.temperatureCelsius);
  Serial.print(" (");
  Serial.print(reading1.temperatureCelsius);
  Serial.print(", ");
  Serial.print(reading2.temperatureCelsius);
  Serial.print(": \u0394");
  Serial.print(temperatureDifference);
  Serial.print(")  average humidity: ");
  Serial.print(averages.relativeHumidity);
  Serial.print(" (\u0394");
  Serial.print(humidityDifference);
  Serial.println(")");
}

float averagePotentiallyInvalidTemperatures(float temperature1, float temperature2) {
  float value1 = NanMath::nanIfOutOfBounds(temperature1, MIN_REASONABLE_TEMPERATURE, MAX_REASONABLE_TEMPERATURE);
  float value2 = NanMath::nanIfOutOfBounds(temperature2, MIN_REASONABLE_TEMPERATURE, MAX_REASONABLE_TEMPERATURE);

  return NanMath::averageIgnoringNan(value1, value2);
}

sensorReading computeAverages(sensorReading reading1, sensorReading reading2) {
  float averageTemperature = averagePotentiallyInvalidTemperatures(reading1.temperatureCelsius, reading2.temperatureCelsius);
  float averageHumidity = NanMath::averageIgnoringNan(reading1.relativeHumidity, reading2.relativeHumidity);

  sensorReading averages = { averageTemperature, averageHumidity };
  // printAverages(averages, reading1, reading2);

  return averages;
}

void printWiFiStatus() {
  Serial.print("WiFi status: ");
  switch (WiFi.status()) {
    case WL_IDLE_STATUS: Serial.println("idle"); break;
    case WL_NO_SSID_AVAIL: Serial.println("no SSID available"); break;
    case WL_SCAN_COMPLETED: Serial.println("scan completed"); break;
    case WL_CONNECTED: Serial.println("connected"); break;
    case WL_CONNECT_FAILED: Serial.println("connection failed"); break;
    case WL_CONNECTION_LOST: Serial.println("connection lost"); break;
    case WL_DISCONNECTED: Serial.println("disconnected"); break;
    default: break;
  }
}

void setUpWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  printWiFiStatus();

  randomSeed(micros());

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void initializeRealTime() {
  ntp = new TobyNtp(UDP, "0.ca.pool.ntp.org");
  Serial.print("Querying NTP server 0.ca.pool.ntp.org for real time");

  ntp->sendNTPpacket(true);
  ntpTime = ntp->getTime();
  while(ntpTime == 0) {
    delay(500);
    ntpTime = ntp->getTime();
    Serial.print(".");
  }
  Serial.print(" obtained time: ");
  Serial.println(ntpTimeToString());
  unixTimeAtStart = ntpTime;
}

char * ntpTimeToString() {
  timeinfo = localtime(&ntpTime);
  Serial.print("The current hour is: ");
  Serial.println(timeinfo->tm_hour);
  char* timeString = asctime(timeinfo);
  // Strip the newline off the string
  timeString[strlen(timeString) - 1] = 0;
  return timeString;
}

void turnHeatOn() {
  digitalWrite(HEAT_PIN, HIGH);
}

void turnHeatOff() {
  digitalWrite(HEAT_PIN, LOW);
}

void turnSensorsOn() {
  digitalWrite(SENSOR_POWER_PIN, HIGH);
}

void turnSensorsOff() {
  digitalWrite(SENSOR_POWER_PIN, LOW);
}

void adjustHeat(float temperatureCelsius) {
  if (temperatureRising) {
    if (temperatureCelsius <= MAX_TEMPERATURE) {
      cyclesToHeatToMax++;
      cyclesInHeatCoolLoop++;
      turnHeatOn();
    } else {
      // Serial.print("Time to heat to maximum temperature roughly ");
      // Serial.print(cyclesToHeatToMax * 2);
      // Serial.println(" seconds.");
      // Serial.print("Full heat/cool cycle lasted roughly ");
      // Serial.print(cyclesInHeatCoolLoop * 2);
      // Serial.println(" seconds.");
      cyclesToHeatToMax = 0;
      cyclesInHeatCoolLoop = 0;
      turnHeatOff();
      temperatureRising = false;
    }
  } else { // Temperature is falling
    if (temperatureCelsius <= MIN_TEMPERATURE) {
      // Serial.print("Time to drop to minimum temperature roughly ");
      // Serial.print(cyclesToCoolToMin * 2);
      // Serial.println(" seconds.");
      cyclesToCoolToMin = 0;
      cyclesInHeatCoolLoop++;
      turnHeatOn();
      temperatureRising = true;
    } else {
      cyclesToCoolToMin++;
      cyclesInHeatCoolLoop++;
      turnHeatOff();
    }
  }
}

void adjustHumidity(float relativeHumidity) {
  if (humidityRising) {
    if (relativeHumidity <= MAX_HUMIDITY) {
      digitalWrite(HUMIDITY_PIN, HIGH);
    } else {
      digitalWrite(HUMIDITY_PIN, LOW);
      humidityRising = false;
    }
  } else {
    if (relativeHumidity <= MIN_HUMIDITY) {
      digitalWrite(HUMIDITY_PIN, HIGH);
      humidityRising = true;
    } else {
      digitalWrite(HUMIDITY_PIN, LOW);
    }
  }
}

void processTemperature(float temperatureCelsius) {
  if ( isnan(temperatureCelsius) ) {
    consecutiveFailedTemperatureReads++;
    // If it's been a minute (30 * 2 seconds) without a valid temperature value, turn off the heat to fail safe
    if (consecutiveFailedTemperatureReads >= 30) {
      Serial.println("No recent viable temperature reading, turning off heat");
      turnHeatOff();
    }
  } else {
    adjustHeat(temperatureCelsius);
    consecutiveFailedTemperatureReads = 0;
  }
}

void mqttMessageCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnectToMqttServer() {
  if (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);

    // Attempt to connect
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");

      // 
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
    }
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(HUMIDITY_PIN, OUTPUT);
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  turnSensorsOn();
  // Pull up to ensure input isn't floating when button is not pressed
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Ensure the timezone is correctly set
  setenv("TZ", "EST5EDT", 1);

  setUpWiFi();
  UDP.begin(123);
  Serial.print("Started UDP on port ");
  Serial.println(UDP.localPort());

  initializeRealTime();

  // Interrupts potentially cause timeouts when reading the DHT22s
  DHT.setDisableIRQ(true);

  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttMessageCallback);

  while (!mqttClient.connected()) {
    reconnectToMqttServer();
    if (!mqttClient.connected()) {
      Serial.println("Trying again in 5 seconds");
      delay(5000);
    }
  }
}

void sendMessage(const char* topic, float value) {
  strcpy(mqttMessage.topic, topic);
  dtostrf(value, 6, 2, mqttMessage.body); // Leave room for too large numbers!
  mqttClient.publish(mqttMessage.topic, mqttMessage.body);
}

void handleButtonState(int currentButtonState) {
  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_MILLIS) {
    if (currentButtonState != buttonState) {
      buttonState = currentButtonState;

      if (buttonState == LOW) {
        Serial.println("Button has been pressed!");
      }
    }
  }

  lastButtonState = currentButtonState;
}

void loop() {
  if (!mqttClient.loop()) {
    reconnectToMqttServer();
  }

  handleButtonState(digitalRead(BUTTON_PIN));

  unsigned long currentMillis = millis();

  if (currentMillis - lastSensorReadMillis >= MIN_SENSOR_READ_MILLIS) {
    lastSensorReadMillis = currentMillis;

    sensorReading sensor1Reading = readSensor(1, DHT22_PIN_1, sensor1Stats);
    sendMessage(TEMPERATURE_TOPIC_1, sensor1Reading.temperatureCelsius);
    sendMessage(HUMIDITY_TOPIC_1, sensor1Reading.relativeHumidity);

    sensorReading sensor2Reading = readSensor(2, DHT22_PIN_2, sensor2Stats);
    sendMessage(TEMPERATURE_TOPIC_2, sensor2Reading.temperatureCelsius);
    sendMessage(HUMIDITY_TOPIC_2, sensor2Reading.relativeHumidity);

    sensorReading averages = computeAverages(sensor1Reading, sensor2Reading);
    sendMessage(TEMPERATURE_TOPIC_AVERAGE, averages.temperatureCelsius);
    sendMessage(HUMIDITY_TOPIC_AVERAGE, averages.relativeHumidity);
    
    sendMessage(FREE_MEMORY_TOPIC, esp.getFreeHeap());
    sendMessage(ELAPSED_TIME_IN_SECONDS_TOPIC, ntpTime - unixTimeAtStart);

      // It's unlikely temperature reads will fail sequentially other than persistent timeout (which requires power cycling)
    if (consecutiveSensorTimeouts >= 10) {
      Serial.println("Power cycling sensors");
      turnSensorsOff();
      delay(100);
      turnSensorsOn();
      consecutiveSensorTimeouts = 0;
    }

    processTemperature(averages.temperatureCelsius);

    // Throttle statistic output so it's not overwhelming
    if (sensor1Stats.total % 50 == 0) {
      Serial.println("Total\t\t\tOK\t\t\tChecksum error\t\tTimeout\t\t\tUnknown\t\t\tFree heap");
      printStatistics(1, sensor1Stats);
      printStatistics(2, sensor2Stats);
    }

    if ( !isnan(averages.relativeHumidity)) {
      adjustHumidity(averages.relativeHumidity);
    } 

    ntp->sendNTPpacket();
    ntpTime = ntp->getTime();
    
    boardLedState = !boardLedState;
    digitalWrite(LED_PIN, boardLedState ? HIGH : LOW);
  }
}
