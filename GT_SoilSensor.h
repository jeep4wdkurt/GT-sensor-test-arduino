/* GT_SoilSensor.h
 *
 * Description:
 *    Growtacular Soil Temperature Sensor Class
 *
 *    DS18B20 Waterproof Soil Temperature Sensor
 *
 * Copyright:
 *    Copyright (c) 2022 Kurt Schulte
 *
 * History:
 *    Version   Date        Author        Desc
 *    01.00.00  2022.06.08  KSchulte      Original version
 */

#ifndef GT_SoilTemperatureSensor_h
#define GT_SoilTemperatureSensor_h

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define SENSORS_MAX	8													// Maximum supported number of sensors

typedef uint8_t DeviceAddress[8];     				// Device address datatatype (8*unsigned byte)

enum {
	MEASUREMENT_AIR_TEMPERATURE =	1,
	MEASUREMENT_AIR_PRESSURE =      2,
}

/*
 * Growtastic Sensor
 */
class GT_Sensor {
public:
  GT_Sensor(void);
  begin(bool addr_detect=true);
  end(void);
  bool measures_temperature(void);
  bool measures_hulidity();
  bool measures_pressure();






// Growtastic Soil Temperature Sensor
class GT_SoilTemperatureSensor {
public:
  GT_SoilSensor(uint8_t addr, TwoWire *theWire = &Wire);
  uint8_t address(void);
  bool begin(bool addr_detect = true);
  void end(void);

private:
  DeviceAddress owDeviceAddress;              // Address of OneWire device
  float temperatureReading;                   // Temperature reading
};

// Growtastic Soil Temperature Sensor Manager
class GT_SoilTemperatureSensors {
public:
	GT_SoilTemperatureSensors(uint8_t oneWirePort);
private:
	OneWire _oneWire;                           // OneWire instance to communicate with any OneWire devices
                                              //    (not just Maxim/Dallas temperature ICs)
	DallasTemperature _owSensors;               // DallasTemperature instance
	int _owDeviceCt;                            // OneWire device count
	GT_SoilTemperatureSensor _soilSensors[SENSORS_MAX];		// GT Soil Sensors list
	
//  uint8_t _addr;
//  TwoWire *_wire;
  bool _begun;
  size_t _maxBufferSize;
//  bool _read(uint8_t *buffer, size_t len, bool stop);
};

#endif // GT_SoilTemperatureSensor_h
