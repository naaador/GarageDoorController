#include "QuadratureEncoder.h"

QuadratureEncoder::QuadratureEncoder(int pin1, int pin2){
    this->pin1 = pin1;
    this->pin2 = pin2;

    lastEncChange = millis();

    pinMode(pin1, INPUT);
    pinMode(pin2, INPUT);

    loop();
}

void QuadratureEncoder::setInterruptHandler(void (*interruptHandler)(void)){
    //attach the supplied ISR to the encoder's pins
    //ISR should call the loop function
    attachInterrupt(digitalPinToInterrupt(pin1), interruptHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pin2), interruptHandler, CHANGE);
}

void QuadratureEncoder::setInvert(bool state){
    //multiplier is set to -1 if we need to invert the direction of the encoder, ie because it was installed "backwards"
    multiplier = state ? -1 : 1;
}

void QuadratureEncoder::resetPosition(int pos){
    qencPos = pos;
}

int QuadratureEncoder::getPosition(){
    return qencPos;
}

int QuadratureEncoder::getDirection(){
    return qencDir;
}

bool QuadratureEncoder::isMoving(){
    return millis() - lastEncChange < maxChangeMs && qencDir != 0 && abs(qencDir) != 2;
}

void QuadratureEncoder::loop(){
    qencOld = qencNew;
    qencNew = ((int)digitalRead(pin1) << 1) | digitalRead(pin2); //originally the bitwise OR was addition
    qencDir = multiplier * QEM[(qencOld << 2) | qencNew];  //shifting 2 bits left multiplies by 4, also the bitwise OR was originally addition
    if(qencOld != qencNew && abs(qencDir) != 2){
        lastEncChange = millis();
        qencPos += qencDir;
    }
}