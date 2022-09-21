#ifndef KOMPOST_APP_H_
#define KOMPOST_APP_H_

#include <CayenneLPP.h>

#include "HeltecBatterySensor.h"
#include "TemperatureSensor.h"

struct KompostState
{
    const char *lastLmicEvent = nullptr;
    float batteryVoltage;
    uint8_t numTemperatures;
    float temperatures[3] = {};
};

class KompostApp
{
protected:
    /**
     * Current state of the kompost
     */
    KompostState *state;

    /**
     * Cayenne LPP container for encoded sensor data
     * Automatically updated on .update() call
     */
    CayenneLPP *lpp;

    HeltecBatterySensor *batterySensor;

    TemperatureSensor *tempSensors;

public:
    KompostApp(KompostState *state, HeltecBatterySensor *batterySensor, TemperatureSensor *tempSensors, CayenneLPP *lpp);

    void update();
};

#endif // KOMPOST_APP_H_
