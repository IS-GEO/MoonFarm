// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_CCS811.h>

// This #include statement was automatically added by the Particle IDE.
#include <photon-thermistor.h>

/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/

// #include <Wire.h>
// #include <SPI.h>
// #include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// #include <Adafruit_CCS811.h>

#define BME_SCK D4
#define BME_MISO D3
#define BME_MOSI D2
#define BME_CS D5

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
Thermistor *thermistor; // thermistor
Adafruit_CCS811 ccs; // CO2
// Adafruit_CCS811 ccs; 

//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

int moisture_pin_sm6  = A0; // upper soil moisture
int moisture_pin_sm18 = A3; //lower soil moisture
int thermistor_pin    = A1; // soil temp thermistor 10K

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ; // time to get serial running
    Serial.println(F("BME280 test"));
    
    // initialize ccs811
    ccs.begin();

    unsigned status;

    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin();
    if (!status)
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x");
        Serial.println(bme.sensorID(), 16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1)
            ;
    }

    Serial.println("-- Default Test --");
    delayTime = 1000;

    pinMode(moisture_pin_sm6, INPUT);
    pinMode(moisture_pin_sm18, INPUT);
    
    /*
    * Particle constructor, sets defaults: vcc=3.3, analogReference=3.3, adcMax=4095
    *
    * arg 1: pin: Photon analog pin
    * arg 2: seriesResistor: The ohms value of the fixed resistor (based on your hardware setup, usually 10k)
    * arg 3: thermistorNominal: Resistance at nominal temperature (will be documented with the thermistor, usually 10k)
    * arg 4: temperatureNominal: Temperature for nominal resistance in celcius (will be documented with the thermistor, assume 25 if not stated)
    * arg 5: bCoef: Beta coefficient (or constant) of the thermistor (will be documented with the thermistor, typically 3380, 3435, or 3950)
    * arg 6: samples: Number of analog samples to average (for smoothing)
    * arg 7: sampleDelay: Milliseconds between samples (for smoothing)
    */
    // Thermistor(int pin, int seriesResistor, int thermistorNominal, int temperatureNominal, int bCoef, int samples, int sampleDelay);
    
    thermistor = new Thermistor(A1, 10000, 10000, 25, 3977, 5, 20);
}

void loop()
{
    // co2
    int eCO2_read = 0;
    
    if (ccs.available()){
        bool error = ccs.readData(); //returns True if an error occurs during the read
        int eCO2_read = ccs.geteCO2(); //returns eCO2 reading
        int TVOC = ccs.getTVOC(); //return TVOC reading
    }
    
    // soil temp
    double soil_temperature = thermistor->readTempC();
    
    // Soil moisture logic
    int moisture_analog_sm6  = analogRead(moisture_pin_sm6); // read soil moisture sensor
    int moisture_analog_sm18 = analogRead(moisture_pin_sm18);
    
    float sensor_min_value = 0.0;  // Decided to use full range to calculate percent of raw
    float sensor_max_value = 4095; // Decided to use full range to calculate percent of raw
    
    float moisture_percentage_sm6  = (100 - ( ( (moisture_analog_sm6 - sensor_min_value)/sensor_max_value) * 100 ) );
    float moisture_percentage_sm18 = (100 - ( ( (moisture_analog_sm18 - sensor_min_value)/sensor_max_value) * 100 ) );
    
    //Pull Sensor data to variables
    // char json[] = "{\"hello\":\"world\"}";
    float sm6      = moisture_percentage_sm6;  // shallow soil moisture sensor: percent of raw
    float sm18     = moisture_percentage_sm18; // deep soil moisture sensor: percent of raw
    float st6      = soil_temperature;         // shallow soil temp (C)
    int eCO2       = eCO2_read;                        // set garbage value and replace
    float surft    = bme.readTemperature();    // surface temperature
    float rh       = bme.readHumidity();       // relative humidity
    float sm6_raw  = moisture_analog_sm6;      // shallow raw soil moisture: raw
    float sm18_raw = moisture_analog_sm18;     // deep raw soil moisture: raw
    
    // Format string for submission to CHORDS
    String data = String::format(
      "{"
        "\"sm6\":%.2f,"
        "\"sm18\":%.2f,"
        "\"st6\":%.2f,"
        "\"CO2\":%.2f,"
        "\"surft\":%.2f,"
        "\"rh\":%.2f,"
        "\"sm6_raw\":%.2f,"
        "\"sm18_raw\":%.2f"
      "}",
      sm6,
      sm18,
      st6,
      eCO2,
      surft,
      rh,
      sm6_raw,
      sm18_raw);
      
    Particle.publish("MoonFarm", data, PRIVATE);
    delay(60000);
}
/*
void printValues()
{
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}
*/
