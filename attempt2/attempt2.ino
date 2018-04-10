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

const float MAX_TEMPERATURE = 37.9;
const float MIN_TEMPERATURE = 37.7;

const float MAX_HUMIDITY = 70;
const float MIN_HUMIDITY = 60;

const char* ssid = SSID;
const char* password = PASSWORD;

bool temperatureRising = true;
bool humidityRising = true;

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

statistics sensor1Stats = { 0,0,0,0,0,0,0,0};
statistics sensor2Stats = { 0,0,0,0,0,0,0,0};

sensorReading readSensor(uint8_t dhtGpio) {
    uint32_t start = micros();
    int chk = DHT.read22(dhtGpio);
    uint32_t stop = micros();

    sensor1Stats.total++;
    switch (chk) {
    case DHTLIB_OK:
        sensor1Stats.ok++;
        break;
    case DHTLIB_ERROR_CHECKSUM:
        sensor1Stats.crc_error++;
        break;
    case DHTLIB_ERROR_TIMEOUT:
        sensor1Stats.time_out++;
        break;
    default:
        sensor1Stats.unknown++;
        break;
    }

    return { DHT.temperature, DHT.humidity, stop - start };
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

sensorReading printAverages(sensorReading reading1, sensorReading reading2) {
  float averageTemperature = (reading1.temperatureCelsius + reading2.temperatureCelsius) / 2;
  float temperatureDifference = fabsf(reading1.temperatureCelsius - reading2.temperatureCelsius);
  float averageHumidity = (reading1.relativeHumidity + reading2.relativeHumidity) / 2;
  float humidityDifference = fabsf(reading1.relativeHumidity - reading2.relativeHumidity);

  Serial.print("Average temperature: ");
  Serial.print(averageTemperature);
  Serial.print(" (\u0394");
  Serial.print(temperatureDifference);
  Serial.print(")  average humidity: ");
  Serial.print(averageHumidity);
  Serial.print(" (\u0394");
  Serial.print(humidityDifference);
  Serial.println(")");
  return { averageTemperature, averageHumidity };
}

void set_up_wifi() {
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

void adjustHeat(float temperatureCelsius) {
  if (temperatureRising) {
    if (temperatureCelsius <= MAX_TEMPERATURE) {
      digitalWrite(HEAT_PIN, HIGH);
    } else {
      digitalWrite(HEAT_PIN, LOW);
      temperatureRising = false;
    }
  } else {
    if (temperatureCelsius <= MIN_TEMPERATURE) {
      digitalWrite(HEAT_PIN, HIGH);
      temperatureRising = true;
    } else {
      digitalWrite(HEAT_PIN, LOW);
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

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(HUMIDITY_PIN, OUTPUT);

  set_up_wifi();
}

void loop() {
  sensorReading sensor1Reading = readSensor(DHT22_PIN_1);
  sensorReading sensor2Reading = readSensor(DHT22_PIN_2);

  sensorReading reading = printAverages(sensor1Reading, sensor2Reading);

  if ( !isnan(reading.temperatureCelsius) ) {
    adjustHeat(reading.temperatureCelsius);
  }

  if ( !isnan(reading.relativeHumidity)) {
    adjustHumidity(reading.relativeHumidity);
  }
  
  delay(1000);
  digitalWrite(LED_PIN, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(1000);
  digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
}

