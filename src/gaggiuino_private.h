#ifndef GAGGIUINO_PRIVATE_H
#define GAGGIUINO_PRIVATE_H

#include "eeprom_data/eeprom_data.h"
#include "profiling_phases.h"
#include <Arduino.h>

static void sensorsRead(void);
static void sensorReadSwitches(void);
static void sensorsReadTemperature(void);
static void sensorsReadWeight(void);
static void sensorsReadPressure(void);
static long sensorsReadFlow(float elapsedTimeSec);
static void calculateWeightAndFlow(void);
static void readTankWaterLevel(void);
static void pageValuesRefresh();
static void modeSelect(void);
static void lcdRefresh(void);
static void fillBoiler(void);
static void tryEepromWrite(const eepromValues_t &eepromValues);
static void
lcdSwitchActiveToStoredProfile(const eepromValues_t &storedSettings);
static bool isBoilerFillPhase(unsigned long elapsedTime);
static void addFillBasketPhase(float flowRate);
static void addPressurePhase(Transition pressure, float flowRestriction,
                             int timeMs, float pressureAbove,
                             float pressureBelow, float shotWeight,
                             float isWaterPumped);
static void addFlowPhase(Transition flow, float pressureRestriction, int timeMs,
                         float pressureAbove, float pressureBelow,
                         float shotWeight, float isWaterPumped);
static void addPhase(PHASE_TYPE type, Transition target, float restriction,
                     int timeMs, float pressureAbove, float pressureBelow,
                     float shotWeight, float isWaterPumped);
static void addPreinfusionPhases();
static void addSoakPhase();
static void addMainExtractionPhasesAndRamp();
static void insertRampPhaseIfNeeded(size_t rampPhaseIndex);
static bool isBoilerFull(unsigned long elapsedTime);
static bool isSwitchOn(void);
static void fillBoilerUntilThreshod(unsigned long elapsedTime);
static void updateStartupTimer(void);
static void cpsInit(eepromValues_t &eepromValues);
static void doLed(void);
static void updateProfilerPhases(void);
static void profiling(void);
static void manualFlowControl(void);
static void brewDetect(void);
static void brewParamsReset(void);
static bool sysReadinessCheck(void);
static inline void sysHealthCheck(float pressureThreshold);
static unsigned long getTimeSinceInit(void);

#endif // GAGGIUINO_PRIVATE_H
