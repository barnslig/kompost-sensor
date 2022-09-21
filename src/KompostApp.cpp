#include "KompostApp.h"

KompostApp::KompostApp(KompostState *state,
                       HeltecBatterySensor *batterySensor,
                       TemperatureSensor *tempSensors,
                       CayenneLPP *lpp) : state(state),
                                          batterySensor(batterySensor),
                                          tempSensors(tempSensors),
                                          lpp(lpp)
{
}

void KompostApp::update()
{
    lpp->reset();

    state->batteryVoltage = batterySensor->read();
    lpp->addAnalogInput(1, state->batteryVoltage);

    tempSensors->update();
    state->numTemperatures = tempSensors->getDeviceCount();
    for (uint8_t i = 0; i < state->numTemperatures; i += 1)
    {
        state->temperatures[i] = tempSensors->read(i);
        lpp->addTemperature(i + 1, state->temperatures[i]);
    }
}
