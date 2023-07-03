#include <Arduino.h>
#include "tinycontroller.h"
#include "blinky.h"

/**
 * Safety controller for BatterBox switching
 *
 * Wiring:
 * Input0: MotorSwitch (0 off, VBAT on)
 *
 * Output0: Motor with antispark resistor+
 * Output1: Antispark off contactor+
 * Output2: Status LED+
 */

#define PIN_IN_MOTOR PIN_IN_0
#define PIN_IN_DEADMAN PIN_IN_1

#define PIN_OUT_RESISTOR PIN_OUT_0
#define PIN_OUT_CONTACTOR PIN_OUT_1
#define PIN_OUT_STATUSLED PIN_OUT_2

#define ANTISPARK_TIME_DELAY_MS 5000
#define STATUSLED_BLINK_PERIOD 750
#define STATUSLED_NOTSAFE_BLINK_PERIOD 350


#define LED_BLINK_OK 500
#define LED_BLINK_NOTSAFE 250

uint64_t lls = 0;

uint64_t lastLedSwitch = 0;

enum class CtrlState {
    NotSafeInit,
    MotorOff,
    AntisparkOn,
    MotorOn,
};


CtrlState ctrlState = CtrlState::NotSafeInit;


void setup() {
    digitalWrite(PIN_OUT_RESISTOR, LOW);
    digitalWrite(PIN_OUT_CONTACTOR, LOW);
    digitalWrite(PIN_OUT_STATUSLED, LOW);

    pinMode(PIN_IN_MOTOR, INPUT);
    pinMode(PIN_IN_DEADMAN, INPUT);
    pinMode(PIN_IN_BUTTON, INPUT);

    // NOT USED
    digitalWrite(PIN_OUT_3, LOW);
    digitalWrite(PIN_OUT_LED, LOW);

    pinMode(PIN_OUT_RESISTOR, OUTPUT);
    pinMode(PIN_OUT_CONTACTOR, OUTPUT);
    pinMode(PIN_OUT_STATUSLED, OUTPUT);
    pinMode(PIN_OUT_LED, OUTPUT);

    delay(1000);
}

uint64_t motorOnTime = 0;

void loop() {
    // read inputs
    int8_t motorSwitch = digitalRead(PIN_IN_MOTOR);
    int8_t deadmanSwitch = digitalRead(PIN_IN_DEADMAN);
    digitalWrite(PIN_OUT_3, deadmanSwitch);

    // update state machine
    switch (ctrlState) {
        case CtrlState::NotSafeInit:
            if(motorSwitch == LOW && deadmanSwitch == LOW)
                ctrlState = CtrlState::MotorOff;
            break;
        case CtrlState::MotorOff:
            if(motorSwitch == HIGH) {
                ctrlState = CtrlState::AntisparkOn;
                motorOnTime = millis();
            }
            break;
        case CtrlState::AntisparkOn:
            if(motorSwitch == LOW) {
                ctrlState = CtrlState::MotorOff;
            } else if(millis() - motorOnTime > ANTISPARK_TIME_DELAY_MS) {
                ctrlState = CtrlState::MotorOn;
            }
            break;
        case CtrlState::MotorOn:
            if(motorSwitch == LOW) {
                ctrlState = CtrlState::MotorOff;
            }
            break;
    }
    if(deadmanSwitch == HIGH) {
        ctrlState = CtrlState::NotSafeInit;
    }

    // ------------------------------------------------------------------------
    if(ctrlState == CtrlState::MotorOff || ctrlState == CtrlState::NotSafeInit) {
        digitalWrite(PIN_OUT_RESISTOR, LOW);
        digitalWrite(PIN_OUT_CONTACTOR, LOW);
        digitalWrite(PIN_OUT_STATUSLED, LOW);

        if(ctrlState == CtrlState::NotSafeInit) {
            if(lastLedSwitch + STATUSLED_NOTSAFE_BLINK_PERIOD < millis()) {
                digitalWrite(PIN_OUT_STATUSLED, CHANGE);
                lastLedSwitch = millis();
            }
        }else {
            digitalWrite(PIN_OUT_STATUSLED, LOW);
        }
    }else if(ctrlState == CtrlState::MotorOn) {
        digitalWrite(PIN_OUT_RESISTOR, HIGH);
        digitalWrite(PIN_OUT_CONTACTOR, HIGH);
        digitalWrite(PIN_OUT_STATUSLED, HIGH);
    }else if(ctrlState == CtrlState::AntisparkOn) {
        digitalWrite(PIN_OUT_RESISTOR, HIGH);
        digitalWrite(PIN_OUT_CONTACTOR, LOW);

        if(lastLedSwitch + STATUSLED_BLINK_PERIOD < millis()) {
            digitalWrite(PIN_OUT_STATUSLED, CHANGE);
            lastLedSwitch = millis();
        }
    }

    uint64_t time = millis();
    if(ctrlState == CtrlState::NotSafeInit) {
        if(lls + LED_BLINK_NOTSAFE < time){
            lls = time;
            digitalWrite(PIN_OUT_LED, CHANGE);
        }
    }else {
        if(lls + LED_BLINK_OK < time) {
            lls = time;
            digitalWrite(PIN_OUT_LED, CHANGE);
        }
    }
}