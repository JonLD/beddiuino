#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <Arduino.h>

#include "../lib/Common/measurements.h"
#include "../lib/Common/sensors_state.h"
#include "eeprom_data/eeprom_data.h"
#include "lcd/lcd.h"
#include "peripherals/tof.h"

class SensorReader
{
  public:
    SensorReader()
    {
        thermoTimer = 0u;
        previousPressure_bar = 0.f;
    }
    void sensorReadStep(SensorState &currentState, const eepromValues_t &runningCfg,
                        const bool brewActive, const NextionPage lcdCurrentPageId,
                        Measurements &weightMeasurements);
    float getChangeInPressure(const SensorState &currentState);
    void themocoupleHealthCheck(SensorState &currentState, const eepromValues_t &runningCfg);
    static long readFlow(SensorState &currentState, const float elapsedTimeSec);
    void initWaterLevelSensor(SensorState &currentState);

  private:
    unsigned long thermoTimer;
    float previousPressure_bar;
    TOF tofSensor;
    static void readSwitches(SensorState &currentState);
    void readTemperature(SensorState &currentState, const eepromValues_t &runningCfg);
    static void readWeight(SensorState &currentState, const bool brewActive,
                           Measurements &weightMeasurements);
    void readPressure(SensorState &currentState);
    void readTankWaterLevel(SensorState &currentState, const NextionPage lcdCurrentPageId);
};

#endif // SENSOR_READER_H
