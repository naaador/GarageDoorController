#pragma once

#ifndef QuadratureEncoder_h
#define QuadratureEncoder_h


#include <Arduino.h>

class QuadratureEncoder {
    protected:
        int pin1, pin2, qencOld = 0, qencNew = 0, multiplier = 1;
        volatile int qencDir = 0, qencPos = 0;
        unsigned int maxChangeMs = 250;
        const int QEM[16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};
        volatile unsigned long lastEncChange = 0;

    public:
        QuadratureEncoder(int pin1, int pin2);

        void setInterruptHandler(void (*interruptHandler)(void));
        void setInvert(bool state = true);
        int getPosition();
        void resetPosition(int pos = 0);
        int getDirection();
        bool isMoving();
        void loop();
};

#endif