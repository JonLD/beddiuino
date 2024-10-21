#include "sensor_reader.h"
#include "../lib/Common/measurements.h"
#include "../lib/Common/sensors_state.h"
#include "eeprom_data/eeprom_data.h"
#include "lcd/lcd.h"
#include "log.h"
#include "peripherals/gpio.h"
#include "peripherals/pressure_sensor.h"
#include "peripherals/pump.h"
#include "peripherals/scales.h"
#include "peripherals/thermocouple.h"
#include "peripherals/tof.h"
#include <Arduino.h>
#include <SimpleKalmanFilter.h>

// Define some const values
#if defined SINGLE_BOARD
// max31855 amp module data read interval not recommended to be changed to lower than 70 (ms)
#define GET_KTYPE_READ_EVERY 70
#else
// max6675 amp module data read interval not recommended to be changed to lower than 250 (ms)
#define GET_KTYPE_READ_EVERY 250
#endif
#define GET_PRESSURE_READ_EVERY 10 // Pressure refresh interval (ms)
#define GET_SCALES_READ_EVERY 100  // Scales refresh interval (ms)

TOF tof;

namespace
{
SimpleKalmanFilter smoothPressure(0.6f, 0.6f, 0.1f);
SimpleKalmanFilter smoothPumpFlow(0.1f, 0.1f, 0.01f);
SimpleKalmanFilter smoothScalesFlow(0.5f, 0.5f, 0.01f);
Measurements weightMeasurements(4);
} // namespace

/* Public method definitions */

void SensorReader::sensorReadStep(SensorState &currentState, const eepromValues_t &runningCfg,
                                  const bool brewActive, const NextionPage lcdCurrentPageId)
{
    readSwitches(currentState);
    readTemperature(currentState, runningCfg);
    readWeight(currentState, brewActive);
    readPressure(currentState);
    readTankWaterLevel(currentState, lcdCurrentPageId);
}

float SensorReader::getChangeInPressure(const SensorState &currentState)
{
    return previousSmoothedPressure - currentState.smoothedPressure;
}

void SensorReader::themocoupleHealthCheck(SensorState &currentState,
                                          const eepromValues_t runningCfg)
{
    if (millis() > thermoTimer)
    {
        LOG_ERROR("Cannot read temp from thermocouple (last read: %.1lf)!",
                  static_cast<double>(currentState.temperature));
        currentState.steamSwitchState ? lcdShowPopup("COOLDOWN")
                                      : lcdShowPopup("TEMP READ ERROR"); // writing a LCD message
        currentState.temperature =
            thermocoupleRead() - runningCfg.offsetTemp; // Making sure we're getting a value
        thermoTimer = millis() + GET_KTYPE_READ_EVERY;
    }
}

/* Private method definitions */

void SensorReader::readSwitches(SensorState &currentState)
{
    currentState.brewSwitchState = gpio::brewState();
    currentState.steamSwitchState = gpio::steamState();
    // use either an actual switch, or the GC/GCP switch combo
    currentState.hotWaterSwitchState =
        gpio::waterPinState() || (currentState.brewSwitchState && currentState.steamSwitchState);
}

void SensorReader::readTemperature(SensorState &currentState, const eepromValues_t &runningCfg)
{
    if (millis() > thermoTimer)
    {
        currentState.temperature = thermocoupleRead() - runningCfg.offsetTemp;
        thermoTimer = millis() + GET_KTYPE_READ_EVERY;
    }
}

void SensorReader::readWeight(SensorState &currentState, const bool brewActive)
{
    static unsigned long scalesTimer;
    uint32_t elapsedTime = millis() - scalesTimer;

    if (elapsedTime > GET_SCALES_READ_EVERY)
    {
        currentState.scalesPresent = scalesIsPresent();
        if (currentState.scalesPresent)
        {
            if (currentState.tarePending)
            {
                scalesTare();
                weightMeasurements.clear();
                weightMeasurements.add(scalesGetWeight());
                currentState.tarePending = false;
            }
            else
            {
                weightMeasurements.add(scalesGetWeight());
            }
            currentState.weight = weightMeasurements.latest().value;

            if (brewActive)
            {
                currentState.shotWeight = currentState.tarePending ? 0.f : currentState.weight;
                currentState.weightFlow =
                    fmax(0.f, weightMeasurements.measurementChange().changeSpeed());
                currentState.smoothedWeightFlow =
                    smoothScalesFlow.updateEstimate(currentState.weightFlow);
            }
        }
        scalesTimer = millis();
    }
}

void SensorReader::readPressure(SensorState &currentState)
{
    static unsigned long pressureTimer;
    uint32_t elapsedTime = millis() - pressureTimer;

    if (elapsedTime > GET_PRESSURE_READ_EVERY)
    {
        float elapsedTimeSec = elapsedTime / 1000.f;
        currentState.pressure = getPressure();
        previousSmoothedPressure = currentState.smoothedPressure;
        currentState.smoothedPressure = smoothPressure.updateEstimate(currentState.pressure);
        currentState.pressureChangeSpeed =
            (currentState.smoothedPressure - previousSmoothedPressure) / elapsedTimeSec;
        pressureTimer = millis();
    }
}

long SensorReader::readFlow(SensorState &currentState, const float elapsedTimeSec)
{
    static float previousSmoothedPumpFlow;
    long pumpClicks = getAndResetClickCounter();
    currentState.pumpClicks = (float)pumpClicks / elapsedTimeSec;

    currentState.pumpFlow = getPumpFlow(currentState.pumpClicks, currentState.smoothedPressure);

    previousSmoothedPumpFlow = currentState.smoothedPumpFlow;
    // Some flow smoothing
    currentState.smoothedPumpFlow = smoothPumpFlow.updateEstimate(currentState.pumpFlow);
    currentState.pumpFlowChangeSpeed =
        (currentState.smoothedPumpFlow - previousSmoothedPumpFlow) / elapsedTimeSec;
    return pumpClicks;
}

void SensorReader::readTankWaterLevel(SensorState &currentState, const NextionPage lcdCurrentPageId)
{
    if (lcdCurrentPageId == NextionPage::Home)
    {
        // static uint32_t tof_timeout = millis();
        // if (millis() >= tof_timeout) {
        currentState.waterLvl = tof.readLvl();
        // tof_timeout = millis() + 500;
        // }
    }
}

void SensorReader::initWaterLevelSensor(SensorState &currentState)
{
    tof.init(currentState);
}
