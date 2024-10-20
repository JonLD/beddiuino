#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <Arduino.h>

#include "../lib/Common/sensors_state.h"
#include "eeprom_data/eeprom_data.h"
#include "lcd/lcd.h"

class SensorReader
{
  public:
    void sensorReadStep(SensorState &currentState, const eepromValues_t &runningCfg,
                        const bool brewActive, const NextionPage lcdCurrentPageId);
    float getChangeInPressure(const SensorState &currentState);
    void themocoupleHealthCheck(SensorState &currentState, const eepromValues_t runningCfg);
    static long readFlow(SensorState &currentState, const float elapsedTimeSec);
    static void initWaterLevelSensor(SensorState &currentState);

  private:
    unsigned long thermoTimer;
    float previousSmoothedPressure;
    static void readSwitches(SensorState &currentState);
    void readTemperature(SensorState &currentState, const eepromValues_t &runningCfg);
    static void readWeight(SensorState &currentState, const bool brewActive);
    void readPressure(SensorState &currentState);
    static void readTankWaterLevel(SensorState &currentState, const NextionPage lcdCurrentPageId);
};

#endif // SENSOR_READER_H
