#include "sensor_reader.h"

#include <Arduino.h>
#include <SimpleKalmanFilter.h>

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

/* Private defines */
#if defined SINGLE_BOARD
// max31855 amp module data read interval not recommended to be changed to lower than 70 (ms)
#define GET_KTYPE_READ_EVERY 70
#else
// max6675 amp module data read interval not recommended to be changed to lower than 250 (ms)
#define GET_KTYPE_READ_EVERY 250
#endif
#define GET_PRESSURE_READ_EVERY 10 // Pressure refresh interval (ms)
#define GET_SCALES_READ_EVERY 100  // Scales refresh interval (ms)

/* Public method definitions */

void SensorReader::sensorReadStep(SensorState &currentState, const eepromValues_t &runningCfg,
                                  const bool brewActive, const NextionPage lcdCurrentPageId,
                                  Measurements &weightMeasurements)
{
    readSwitches(currentState);
    readTemperature(currentState, runningCfg);
    readWeight(currentState, brewActive, weightMeasurements);
    readPressure(currentState);
    readTankWaterLevel(currentState, lcdCurrentPageId);
}

float SensorReader::getChangeInPressure(const SensorState &currentState)
{
    return previousPressure_bar - currentState.pressure_bar;
}

void SensorReader::themocoupleHealthCheck(SensorState &currentState,
                                          const eepromValues_t &runningCfg)
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

long SensorReader::readFlow(SensorState &currentState, const float elapsedTimeSec)
{
    static SimpleKalmanFilter smoothPumpFlow(0.1f, 0.1f, 0.01f);
    static float previousPumpFlow;
    long pumpClicks = getAndResetClickCounter();
    currentState.pumpClicks = (float)pumpClicks / elapsedTimeSec;

    currentState.rawPumpFlow = getPumpFlow(currentState.pumpClicks, currentState.pressure_bar);

    previousPumpFlow = currentState.pumpFlow;
    // Some flow smoothing
    currentState.pumpFlow = smoothPumpFlow.updateEstimate(currentState.rawPumpFlow);
    currentState.pumpFlowChangeSpeed =
        (currentState.pumpFlow - previousPumpFlow) / elapsedTimeSec;
    return pumpClicks;
}

void SensorReader::initWaterLevelSensor(SensorState &currentState)
{
    tofSensor.init();
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

void SensorReader::readWeight(SensorState &currentState, const bool brewActive,
                              Measurements &weightMeasurements)
{
    static SimpleKalmanFilter smoothScalesFlow(0.5f, 0.5f, 0.01f);

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
                const float rawWeightFlow =
                    fmax(0.f, weightMeasurements.measurementChange().changeSpeed());
                currentState.weightFlow =
                    smoothScalesFlow.updateEstimate(rawWeightFlow);
            }
        }
        scalesTimer = millis();
    }
}

void SensorReader::readPressure(SensorState &currentState)
{
    static SimpleKalmanFilter smoothPressure(0.6f, 0.6f, 0.1f);
    static unsigned long pressureTimer;

    uint32_t elapsedTime = millis() - pressureTimer;

    if (elapsedTime > GET_PRESSURE_READ_EVERY)
    {
        float elapsedTimeSec = elapsedTime / 1000.f;
        previousPressure_bar = currentState.pressure_bar;
        const float rawPressure_bar = getPressure();
        currentState.pressure_bar = smoothPressure.updateEstimate(rawPressure_bar);
        currentState.pressureChangeSpeed =
            (currentState.pressure_bar - previousPressure_bar) / elapsedTimeSec;
        pressureTimer = millis();
    }
}

void SensorReader::readTankWaterLevel(SensorState &currentState, const NextionPage lcdCurrentPageId)
{
    if (lcdCurrentPageId == NextionPage::Home)
    {
        // static uint32_t tof_timeout = millis();
        // if (millis() >= tof_timeout) {
        currentState.waterLvl = tofSensor.readLvl();
        // tof_timeout = millis() + 500;
        // }
    }
}
