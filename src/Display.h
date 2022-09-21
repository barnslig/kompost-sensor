#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <U8g2lib.h>

#include "KompostApp.h"

class Display
{
protected:
    U8G2 *u8g2;

public:
    Display(U8G2 *u8g2);

    void begin();

    void print(KompostState *state);
};

#endif // DISPLAY_H_
