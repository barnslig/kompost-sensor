#include "Display.h"

Display::Display(U8G2 *u8g2) : u8g2(u8g2)
{
}

void Display::begin()
{
    u8g2->begin();
}

void Display::print(KompostState *state)
{
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_siji_t_6x10); // 12x12

    // last lmic event
    u8g2->setCursor(0, 12);
    u8g2->print(state->lastLmicEvent);

    // battery state
    if (state->batteryVoltage < 3.5)
    {
        // low
        u8g2->drawGlyph(80, 28, 57910);
    }
    else if (state->batteryVoltage < 3.8)
    {
        // mid
        u8g2->drawGlyph(80, 28, 57911);
    }
    else
    {
        // high
        u8g2->drawGlyph(70, 28, 57911);
    }
    u8g2->setCursor(86, 28);
    u8g2->print(state->batteryVoltage);
    u8g2->print(F(" V"));

    // temp sensors
    for (uint8_t i = 0; i < state->numTemperatures; i += 1)
    {
        u8g2->drawGlyph(0, 28 + i * 16, 57551);
        u8g2->setCursor(16, 28 + i * 16);
        u8g2->print(state->temperatures[i]);
        u8g2->print(F(" C"));
    }

    u8g2->sendBuffer();
}
