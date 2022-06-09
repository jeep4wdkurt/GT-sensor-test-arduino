// GT-SensorTest-Arduino.ino
//
// Description:
//    Growtacular Sensors Test for Arduino platform
//
//    Tests the following sensors:
//    	  o DS18B20 waterproof temperature sensor(s), one or more connected in parallel to GPIO
//    	  o DHT11 temperature and pressure sensor, connected to GPIO
//        o BME280 temperature/humidity/pressure sensor, connected to I2C bus
//        o ADS1115 4-channel ADC, connected to I2C bus
//        o Capacitive Soil Sensor 1.2, connected via ADS11115 ADC
//        o Capacitive Soil Sensor 1.2, connected via analog GPIO
//
// Copyright:
//    Copyright (c) 2022 Kurt Schulte
//
// History:
//    01.00.00  2022.06.06  KSchulte      Original version
//

#include <DHT.h>												// DHT11 temperature/humidity sensor library
#include <Bme280.h>											// BME280 temperature/humidity/pressur sensor library

#include "GT_SoilTemperatureSensor.h"		// Growtacular soil temperature sensors

//
// DATA
//

// DHT11 Sensor Data

#define DHT11_PIN 10                  	// DHT11 Temp/Humidity sensor wire is plugged into pin 10 on the Arduino
DHT dhtSensor = DHT(DHT11_PIN,DHT11); 	// DHT11 sensor instance

// DS18B20 Sensor Data

#define ONE_WIRE_BUS 4                // OneWire Data wire (DS18B20 sensor(s)) plugged into pin 4 on the Arduino

typedef uint8_t DeviceAddress[8];     // Device address datatatype (8*unsigned byte)

DeviceAddress owDevice0;              // Address of OneWire device #0
DeviceAddress owDevice1;              // Address of OneWire device #1
DeviceAddress owDevice2;              // Address of OneWire device #2
DeviceAddress owDevice3;              // Address of OneWire device #3
DeviceAddress owDevice4;              // Address of OneWire device #4
DeviceAddress owDevice5;              // Address of OneWire device #5
DeviceAddress owDevice6;              // Address of OneWire device #6
DeviceAddress owDevice7;              // Address of OneWire device #7
uint8_t *owDevices[8];                // List of OneWire device addresses
int owDeviceCt = 0;                   // OneneWire device count
float owTemp[8];                      // Sensor Temps

OneWire oneWire(ONE_WIRE_BUS);        // OneWire instance to communicate with any OneWire devices
                                      //    (not just Maxim/Dallas temperature ICs)
DallasTemperature owSensors(&oneWire);  // DallasTemperature instance
 
//
// UTILITY FUNCTIONS
//
String zeroPad(String inValue, int digits) {
  String returnVal = "0";
  returnVal.concat(inValue);
  returnVal = returnVal.substring(returnVal.length()-2);
  return returnVal;
}

String deviceAddressToString( uint8_t* deviceAddress ) {
  String addressText = "";
  for (int ix=0; ix<8 ; ix++) {
    String byteHex = zeroPad(String(deviceAddress[ix],HEX),2);
    addressText.concat(byteHex);
  }
  return addressText;
}


//
// INITIALIZATION
// 
void setup(void) {

  // Initialize device address list
  owDevices[0] = owDevice0;
  owDevices[1] = owDevice1;
  owDevices[2] = owDevice2;
  owDevices[3] = owDevice3;
  owDevices[4] = owDevice4;
  owDevices[5] = owDevice5;
  owDevices[6] = owDevice6;
  owDevices[7] = owDevice7;

  // Initialize DHT Sensor Object
  //dhtSensor = DHT(DHT11_PIN,DHT11);
//  if ( dhtSensor == NULL ) {
//    Serial.println("Error instancing DHT sensor object. Nothing returned.");
//  }
  
  // Start serial port
  Serial.begin(9600);

  // Report startup
  Serial.println("");
  Serial.println("Growtacular Sensors Test - Arduino Platform");
  Serial.println("Initializing...");  
  delay(500);                   // Delay half a second before starting

  // Start up the libraries
  owSensors.begin();
  dhtSensor.begin();

  //Check to see how many sensors are on the one-wire bus
  int tempSensorCt = owSensors.getDS18Count();
  String outText = "DS18xx Count: ";
  outText.concat(tempSensorCt);
  Serial.println(outText);

  // Report the addressess of the temperature sensors...
  uint8_t *sensorAddress;
  
  if ( tempSensorCt > 0 ) {
    for (int deviceIx = 0; deviceIx < tempSensorCt; deviceIx++) {
      sensorAddress = owDevices[deviceIx];
      owSensors.getAddress( sensorAddress, deviceIx );
      outText = "DS18xx[";
      outText.concat(deviceIx);
      outText.concat("] address: ");
      outText.concat(deviceAddressToString(sensorAddress));
      Serial.println(outText);
    }
  }

  Serial.println("Starting...");
  
  delay(500);                   // Delay half a second before starting
  
}

//
// MAIN
// 
void loop(void)
{
  String statusText = "";
  
  // read DS18D20 sensor(s)
  //    call requestTemperatures() to issue a global temperature
  //    request to all devices on the bus
  Serial.print(" Polling DS18B20 temperature sensor(s)...");
  owSensors.requestTemperatures(); // Send the command to get temperatures

  float soilTempF = 0.0;
  int tempSensorCt = owSensors.getDS18Count();
  if ( tempSensorCt == 0 ) {
    Serial.println("Error! No DS18xx temperature sensors found!!");
  } else {
    for (int sensorIx=0; sensorIx < tempSensorCt; sensorIx++) {
      soilTempF = owSensors.getTempF(owDevices[sensorIx]);
      owTemp[sensorIx] = soilTempF;
    }
  }

  // read the DHT11 sensor
  Serial.print(" Polling DHT11 temp/humidity sensor...");

  float airTempF = dhtSensor.convertCtoF(dhtSensor.readTemperature());
  float airHumidity = dhtSensor.readHumidity();
    
  Serial.println("DONE");
  
  // Report sensor data
  //

  if ( tempSensorCt != 0 ) {
    uint8_t *sensorAddress;
    for (int sensorIx=0; sensorIx < tempSensorCt; sensorIx++) {
      // prepare output
      sensorAddress = owDevices[sensorIx];
      statusText = "";
      statusText.concat("SoilTemp @");
      statusText.concat(deviceAddressToString(sensorAddress));
      statusText.concat(" : ");
      statusText.concat(owTemp[sensorIx]);
      statusText.concat("F");
      Serial.println(statusText);
    }
  }

  statusText = "AirTemp                    : ";
  statusText.concat(airTempF);
  statusText.concat("F");
  Serial.println(statusText);

  statusText = "AirHumidity                : ";
  statusText.concat(airHumidity);
  statusText.concat("%");
  Serial.println(statusText);
  
  delay(10000);
}
