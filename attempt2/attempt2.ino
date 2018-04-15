// DHT22 Sensor library
#include <dht.h>

// Connect to local WiFi LAN
#include <ESP8266WiFi.h>

// Local LAN configuration
#include <LanConfiguration.h>

#define LED_PIN 2
#define HEAT_PIN D6 // The GPIO to turn the heat on or off
#define HUMIDITY_PIN D7 // The GPIO to turn the mister on or off
#define DHT22_PIN_1 D1
#define DHT22_PIN_2 D2

// Values to bound what constitutes a correct sensor reading
const float MAX_REASONABLE_TEMPERATURE = 50;
const float MIN_REASONABLE_TEMPERATURE = 10;

const float MAX_TEMPERATURE = 37.9;
const float MIN_TEMPERATURE = 37.7;

const float MAX_HUMIDITY = 70;
const float MIN_HUMIDITY = 60;

const char* ssid = SSID;
const char* password = PASSWORD;

bool temperatureRising = true;
bool humidityRising = true;

int consecutiveFailedTemperatureReads = 0;

int cyclesInHeatCoolLoop = 0;
int cyclesToHeatToMax = 0;
int cyclesToCoolToMin = 0;

dht DHT;

struct statistics {
  uint32_t total;
  uint32_t ok;
  uint32_t crc_error;
  uint32_t time_out;
  uint32_t connect;
  uint32_t ack_l;
  uint32_t ack_h;
  uint32_t unknown;
};

struct sensorReading {
  float temperatureCelsius;
  float relativeHumidity;
  long microsToRead; 
};

statistics sensor1Stats = { 0,0,0,0,0,0,0,0 };
statistics sensor2Stats = { 0,0,0,0,0,0,0,0 };

sensorReading readSensor(uint8_t dhtGpio, statistics stats) {
    uint32_t start = micros();
    int checkStatus = DHT.read22(dhtGpio);
    uint32_t stop = micros();

    stats.total++;
    switch (checkStatus) {
    case DHTLIB_OK:
        stats.ok++;
        return { DHT.temperature, DHT.humidity, stop - start };
    case DHTLIB_ERROR_CHECKSUM:
        Serial.println("Checksum error");
        stats.crc_error++;
        break;
    case DHTLIB_ERROR_TIMEOUT:
        Serial.println("timeout error");
        stats.time_out++;
        break;
    default:
        Serial.println("unknown error");
        stats.unknown++;
        break;
    }

    // On any error conditions, return a neutered value
    return { NAN, NAN, stop - start };
}

void printSensorReading(int sensorNumber, sensorReading reading) {
  Serial.print("Sensor ");
  Serial.print(sensorNumber);
  Serial.print(" - Temperature: ");
  Serial.print(reading.temperatureCelsius);
  Serial.print(" humidity: ");
  Serial.print(reading.relativeHumidity);
  Serial.print(" micros to read: ");
  Serial.println(reading.microsToRead);
}

void printAverages(sensorReading averages, sensorReading reading1, sensorReading reading2) {
  float temperatureDifference = fabsf(reading1.temperatureCelsius - reading2.temperatureCelsius);
  float humidityDifference = fabsf(reading1.relativeHumidity - reading2.relativeHumidity);

  Serial.print("Average temperature: ");
  Serial.print(averages.temperatureCelsius);
  Serial.print(" (\u0394");
  Serial.print(reading1.temperatureCelsius);
  Serial.print(", ");
  Serial.print(reading2.temperatureCelsius);
  Serial.print(": ");
  Serial.print(temperatureDifference);
  Serial.print(")  average humidity: ");
  Serial.print(averages.relativeHumidity);
  Serial.print(" (\u0394");
  Serial.print(humidityDifference);
  Serial.println(")");
}

float nanIfOutOfBounds(float temperature) {
  if (isnan(temperature) || temperature > MAX_REASONABLE_TEMPERATURE || temperature < MIN_REASONABLE_TEMPERATURE) {
    return NAN;
  }

  return temperature;
}

float averagePotentiallyInvalidTemperatures(float temperature1, float temperature2) {
  float value1 = nanIfOutOfBounds(temperature1);
  float value2 = nanIfOutOfBounds(temperature2);

  return averageIgnoringNan(value1, value2);
}

float averageIgnoringNan(float value1, float value2) {
  if (isnan(value1) && isnan(value2)) {
    return NAN;
  } else if (isnan(value1)) {
    return value2;
  } else if (isnan(value2)) {
    return value1;
  } else {
    return (value1 + value2) / 2;
  }
}

sensorReading computeAverages(sensorReading reading1, sensorReading reading2) {
  float averageTemperature = averagePotentiallyInvalidTemperatures(reading1.temperatureCelsius, reading2.temperatureCelsius);
  float averageHumidity = averageIgnoringNan(reading1.relativeHumidity, reading2.relativeHumidity);

  sensorReading averages = { averageTemperature, averageHumidity };
  printAverages(averages, reading1, reading2);

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

void turnHeatOn() {
  digitalWrite(HEAT_PIN, HIGH);
}

void turnHeatOff() {
  digitalWrite(HEAT_PIN, LOW);
}

void adjustHeat(float temperatureCelsius) {
  if (temperatureRising) {
    if (temperatureCelsius <= MAX_TEMPERATURE) {
      cyclesToHeatToMax++;
      cyclesInHeatCoolLoop++;
      turnHeatOn();
    } else {
      Serial.print("Time to heat to maximum temperature roughly ");
      Serial.print(cyclesToHeatToMax * 2);
      Serial.println(" seconds.");
      Serial.print("Full heat/cool cycle lasted roughly ");
      Serial.print(cyclesInHeatCoolLoop * 2);
      Serial.println(" seconds.");
      cyclesToHeatToMax = 0;
      cyclesInHeatCoolLoop = 0;
      turnHeatOff();
      temperatureRising = false;
    }
  } else { // Temperature is falling
    if (temperatureCelsius <= MIN_TEMPERATURE) {
      Serial.print("Time to drop to minimum temperature roughly ");
      Serial.print(cyclesToCoolToMin * 2);
      Serial.println(" seconds.");
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
      turnHeatOff();
    }
  } else {
    adjustHeat(temperatureCelsius);
    consecutiveFailedTemperatureReads = 0;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(HUMIDITY_PIN, OUTPUT);

  setUpWiFi();
  DHT.setDisableIRQ(true);
}

void loop() {
  sensorReading sensor1Reading = readSensor(DHT22_PIN_1, sensor1Stats);
  sensorReading sensor2Reading = readSensor(DHT22_PIN_2, sensor2Stats);
  sensorReading averages = computeAverages(sensor1Reading, sensor2Reading);

  processTemperature(averages.temperatureCelsius);

  if ( !isnan(averages.relativeHumidity)) {
    adjustHumidity(averages.relativeHumidity);
  }
  
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  digitalWrite(LED_PIN, HIGH);
}

