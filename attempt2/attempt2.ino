#include <dht.h>

#define LED_PIN 2
#define DHT22_PIN_1 D1
#define DHT22_PIN_2 D2

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

void printAverages(sensorReading reading1, sensorReading reading2) {
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
}


void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  sensorReading sensor1Reading = readSensor(DHT22_PIN_1);
  sensorReading sensor2Reading = readSensor(DHT22_PIN_2);

  printAverages(sensor1Reading, sensor2Reading);
  digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(2000);
  digitalWrite(LED_PIN, LOW);   // turn the LED on (HIGH is the voltage level)
}

