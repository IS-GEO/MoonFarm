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
// Adafruit_CCS811 ccs; 

//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ; // time to get serial running
    Serial.println(F("BME280 test"));

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

    Serial.println();
}

void loop()
{
    //Pull Sensor data to variables
    // char json[] = "{\"hello\":\"world\"}";
    float sm6 = 25.5;//leaf.bme_temp();
    float sm18 = 40;//leaf.bme_rh();
    float st6 = bme.readTemperature(); //leaf.bme_p();
    float st18 = bme.readTemperature();
    float eCO2 = 0; // set garbage value and replace
    float surft = bme.readTemperature();
    float rh = bme.readHumidity();
    
    // printValues();
    // delay(delayTime);
    // if(ccs.available()){
    //     float temp = ccs.calculateTemperature();
    //     if(!ccs.readData()){
    //         // Serial.print("CO2: ");
    //         // Serial.print(ccs.geteCO2());
    //         eCO2 = ccs.geteCO2();
    //         // Serial.print("ppm, TVOC: ");
    //         // Serial.print(ccs.getTVOC());
    //         // Serial.print("ppb   Temp:");
    //         // Serial.println(temp);
    //         st6 = temp;
    //     }
    //     else{
    //         Serial.println("ERROR!");
    //         while(1);
    //     }
    // }

    // readAlgorithmResults();
    
    String data = String::format(
      "{"
        "\"sm6\":%.2f,"
        "\"sm18\":%.2f,"
        "\"st6\":%.2f,"
        "\"st18\":%.2f,"
        "\"CO2\":%.2f,"
        "\"surft\":%.2f,"
        "\"rh\":%.2f"
      "}",
      sm6,
      sm18,
      st6,
      st18,
      eCO2,
      surft,
      rh);
      
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
