#include <Arduino.h>
#include <Homie.h>
#include <QuadratureEncoder.h>
#include <FS.h>

#define FIRMWARE_VERSION "0.3.3"

//it seemed easier to represent the door's state this way
enum class DoorState : byte {
  Opening,
  OpeningStopped,
  Open,
  Closing,
  ClosingStopped,
  Closed
};

const int PIN_ENC1 = D6; //connect to one of the encoder elements (light break/hall effect/reflection)
const int PIN_ENC2 = D5; //connect to other encoder element
const int PIN_REED_OPEN = D1; //to a N/O reed switch (or some other source that can pull pin low) triggered at full open
const int PIN_REED_CLOSED = D2; //to a N/O reed switch triggered at full close position
const int PIN_RELAY = D7; //using a relay board that enables the relay when its input is pulled low

HomieNode doorNode("door", "Door", "door"); //just one node for now

const int RELAY_PULSE_LENGTH = 500; //how long to pulse the relay in ms
unsigned long lastRelayTrigger = 0; //last time in millis() that the relay was closed

ICACHE_RAM_ATTR void relayOff(){
  //called by the timer started in sendOpenerPulse() to open relay
  digitalWrite(PIN_RELAY, HIGH);
}

const int DOOR_STATE_REPORT_DURATION = 1000; //at least this much time between door position reports
unsigned long doorStateLastReport = 0; //last time that the door state was reported via mqtt
byte doorStateLastReportValue = 255; //since the door state is a percentage it should never be more than 100

unsigned long resumeOpenWhileMoving = 0; //this will hold the time the next pulse should be sent to get the door moving if it was moving when command was sent

//quadrature encoder
QuadratureEncoder qenc = QuadratureEncoder(PIN_ENC1, PIN_ENC2);

volatile bool encStateChanged = false;
ICACHE_RAM_ATTR void encChangeDetected(){
  encStateChanged = true;
  qenc.loop();
}

byte calibrationState = 0; //byte zero means full open is calibrated, byte one indicates door has been fully closed
int DOOR_FULL_OPEN = 1000;  //1000 seems like a sensible default, probably not approopriate if measuring from a sprocket/pully
byte doorPercent = 0;

//last door sensorstates
bool lastIsOpen = false;
bool lastIsClosed = false;

void updateDoorSensorStates(){
  //pins have pull-ups so they'll be low when the reed switches are closed (N/O switches)
  bool isOpen = !digitalRead(PIN_REED_OPEN);
  bool isClosed = !digitalRead(PIN_REED_CLOSED);

  if(lastIsOpen != isOpen || lastIsClosed != isClosed || (isClosed && doorPercent != 100) || (isOpen && doorPercent != 0)){
    lastIsOpen = isOpen;
    lastIsClosed = isClosed;
    if(isOpen && isClosed){
      Homie.getLogger() << "Invalid door sensor state detected!!!  Open sensor: " << isOpen << ", Closed sensor: " << isClosed << endl;
    }
    else if(isOpen){
      //Homie.getLogger() << "Door is in its fully opened position, calibrationState: " << calibrationState << endl;
      if(!(calibrationState & 1) && ((calibrationState >> 1) & 1)){
        //calibration just completed so we'll save the full open encoder position to flash
        DOOR_FULL_OPEN = qenc.getPosition();
        calibrationState |= 1;
        SPIFFS.begin();

        File calibration = SPIFFS.open("/calibration", "w");
        calibration.write(String(DOOR_FULL_OPEN).c_str());

        SPIFFS.end();
        Homie.getLogger() << "Sensor now fully calibrated, full open at " << DOOR_FULL_OPEN << endl;
      }
      qenc.resetPosition(DOOR_FULL_OPEN);
      doorPercent = 0;
    }
    else if(isClosed){
      //Homie.getLogger() << "Door is in its fully closed position, calibrationState: " << calibrationState << endl;
      calibrationState |= 2;  //probably less time wasted just setting the bit than checking first if it needs to be set
      qenc.resetPosition();  //door is closed to encoder is in the 0 position
      doorPercent = 100;
    }
  }
}

bool canSendOpenerPulse(){
  //make sure we don't trigger the relay too quickly back to back
  return millis() - lastRelayTrigger >= RELAY_PULSE_LENGTH * 2 || lastRelayTrigger == 0;
}

void sendOpenerPulse(){
  //if the opener can be triggered, we close the relay, then wait RELAY_PULSE_LENGTH ms to open the relay (using N/O) contacts
  if(canSendOpenerPulse()){
    Homie.getLogger() << "Triggering opener" << endl;
    digitalWrite(PIN_RELAY, LOW);

    //now set a timer to turn off the relay after RELAY_PULSE_LENGTH ms
    timer1_attachInterrupt(relayOff);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
    timer1_write(RELAY_PULSE_LENGTH * 1000 * 5);
  }
  else{
    Homie.getLogger() << "Can't trigger the door again so soon!" << endl;
  }
}

DoorState getCurrentState(){
  if(qenc.isMoving()){
    return qenc.getDirection() < 0 ? DoorState::Closing : DoorState::Opening;
  } else {
    return doorPercent > 95 ? DoorState::Closed : DoorState::Open;
  }
}

bool doorOpenHandler(const HomieRange& range, const String& value){
  //if we receiving the value 'rc' we will reset the calibration data
  if(value == "rc"){
    calibrationState = 0;
    SPIFFS.begin();
    SPIFFS.remove("/calibration");
    SPIFFS.end();
    return true;
  }
  Homie.getLogger() << "in doorOpenHandler: " << value << endl;

  DoorState desiredState = value == "true" ? DoorState::Closed : DoorState::Open;
  DoorState currentState = getCurrentState();
  bool handled = false;

  //TODO: should take into account if the door is stationary, but not completely open or closed.  Door will move in the oposite direction of what it was
  //last moving so that needs to be accounted for
  if((currentState == DoorState::Closed && desiredState == DoorState::Open) || (currentState == DoorState::Open && desiredState == DoorState::Closed)){
    //door is stationary and in the oposite state from what we want
    sendOpenerPulse();
    handled = true;
  } else if((currentState == DoorState::Opening && desiredState == DoorState::Closed) || (currentState == DoorState::Closing && desiredState == DoorState::Open)){
    //door is moving the oposite direction than it needs to
    sendOpenerPulse();
    resumeOpenWhileMoving = millis() + RELAY_PULSE_LENGTH + 1000;
    handled = true;
  }

  return handled;
}

void loopHandler(){
  if((doorPercent != doorStateLastReportValue || doorStateLastReportValue == 255) 
      && (millis() - doorStateLastReport >= DOOR_STATE_REPORT_DURATION || doorStateLastReport == 0)){
    Homie.getLogger() << doorPercent << "%" << endl;
    doorNode.setProperty("openpercent").send(String(doorPercent));
    doorNode.setProperty("open").send(doorPercent > 95 ? "true" : "false"); // a little leway in case the reed switch doesn't work
    doorStateLastReport = millis();
    doorStateLastReportValue = doorPercent;
  }
}

void setup() {
  //TODO: add setting for inverting the quadrature encoder readings so we don't have to recompile to revers its direction
  Serial.begin(115200);
  Serial << endl << endl;

  //check to see if we have calibration data stored in flash so the position will be acurate
  SPIFFS.begin();

  if(SPIFFS.exists("/calibration")){
    File calibration = SPIFFS.open("/calibration", "r");
    DOOR_FULL_OPEN = calibration.readString().toInt();
    calibration.close();
    calibrationState = 3;
  }

  SPIFFS.end();

  //using interupts to manage the quadrature encoder
  qenc.setInterruptHandler(encChangeDetected);

  //set gpio states
  pinMode(PIN_REED_OPEN, INPUT);
  pinMode(PIN_REED_CLOSED, INPUT);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, HIGH);

  updateDoorSensorStates();

  Homie_setFirmware("garage-door-controller", FIRMWARE_VERSION); // The underscore is not a typo! See Magic bytes

  Homie.setLoopFunction(loopHandler);
  doorNode.advertise("openpercent").setName("OpenPercent").setDatatype("integer").setUnit("%");
  doorNode.advertise("open").setName("Open").setDatatype("boolean").settable(doorOpenHandler);
  
  Homie.setup();
}

void loop() {
  Homie.loop();
  
  if(encStateChanged){
    //Homie.getLogger() << qenc.getPosition() << endl;
    //Homie.getLogger() << "ISR time: " << qenc.getLastLoopTime() << endl;
    doorPercent = 100 - (qenc.getPosition() * 100 / DOOR_FULL_OPEN);
    if(doorPercent > 100) doorPercent = 100; // can't be more than 100% open and sometimes this happens because of overrunning the reed switch
    encStateChanged = false;
  }

  if(resumeOpenWhileMoving > 0 && resumeOpenWhileMoving < millis()){
    resumeOpenWhileMoving = 0;
    sendOpenerPulse();
  }

  updateDoorSensorStates();
}