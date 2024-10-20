/* 09:32 15/03/2023 - change triggering comment */
#ifndef GAGGIUINO_H
#define GAGGIUINO_H

#include <Arduino.h>

#include "functional/predictive_weight.h"

// Define some const values
#define REFRESH_SCREEN_EVERY 150      // Screen refresh interval (ms)
#define REFRESH_FLOW_EVERY 50         // Flow refresh interval (ms)
#define HEALTHCHECK_EVERY 30000       // System checks happen every 30sec
#define BOILER_FILL_START_TIME 3000UL // Boiler fill start time - 3 sec since system init.
#define BOILER_FILL_TIMEOUT 8000UL    // Boiler fill timeout - 8sec since system init.
#define BOILER_FILL_SKIP_TEMP 85.f    // Boiler fill skip temperature threshold
#define SYS_PRESSURE_IDLE 0.7f        // System pressure threshold at idle
#define MIN_WATER_LVL 10u             // Min allowable tank water lvl

enum class OPERATION_MODES
{
    OPMODE_straight9Bar,
    OPMODE_justPreinfusion,
    OPMODE_justPressureProfile,
    OPMODE_manual,
    OPMODE_preinfusionAndPressureProfile,
    OPMODE_flush,
    OPMODE_descale,
    OPMODE_flowPreinfusionStraight9BarProfiling,
    OPMODE_justFlowBasedProfiling,
    OPMODE_steam,
    OPMODE_FlowBasedPreinfusionPressureBasedProfiling,
    OPMODE_everythingFlowProfiled,
    OPMODE_pressureBasedPreinfusionAndFlowProfile
};

// Some consts
#ifndef LEGO_VALVE_RELAY
const float calibrationPressure = 2.f;
#else
const float calibrationPressure = 0.65f;
#endif

// Timers
unsigned long systemHealthTimer;
unsigned long pageRefreshTimer;
unsigned long steamTime;

// brew detection vars
bool brewActive = false;
bool nonBrewModeActive = false;

// PP&PI variables
int preInfusionFinishedPhaseIdx = 3;
bool homeScreenScalesEnabled = false;

// Public function declarations

void lcdSaveSettingsTrigger(void);

#endif // GAGGIUINO_H
