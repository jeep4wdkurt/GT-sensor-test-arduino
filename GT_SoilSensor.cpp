/* GT-SoilTemperatureSensor.cpp
 *
 * Description:
 *    Growtacular Soil Temperature Sensor Class
 *
 *    DS18B20 Waterproot Soil Temperature Sensor
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

#include "GT_SoilTemperatureSensor.h"

/*
 * Growtastic Soil Temperature Sensor Manager
 */
GT_SoilTemperatureSensors::GT_SoilTemperatureSensors(uint8_t oneWireGpioPort) {
	_oneWire = OneWire(oneWireGpioPort);       					// Instance to communicate with OneWire devices
	_owSensors = DallasTemperature(&_oneWire);					// Instance for DS18B20 OneneWire sensors
	_owDeviceCt = 0;																		// OneneWire device count
	GT_SoilTemperatureSensor soilSensors[SOIL_TEMP_SENSORS_MAX];	// GT Soil Sensors list

	//  uint8_t _addr;
	//  TwoWire *_wire;
  bool _begun;
  size_t _maxBufferSize;
//  bool _read(uint8_t *buffer, size_t len, bool stop);
};

/*
 * Growtastic Soil Temperature Sensor
 */
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

#endif // GT_SoilTemperatureSensor_h
