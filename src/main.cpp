#include <Arduino.h>
#include "tinycontroller.h"

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

#define HEARTBEAT_TIME_MS 500
#define LED_BLINK_TIME 250
#define LED_STATE_UNSAFE_INIT 0
#define LED_STATE_ANTISPARK 1
#define LED_STATE_MOTOR_ON 2
#define LED_STATE_MOTOR_OFF 3
#define LED_STATE_DEADMAN 4

static const bool led_patterns[][8] = {
        {0, 1, 0, 1, 0, 1, 0, 1},         // UNSAFE_INIT
        {0, 0, 1, 1, 0, 0, 1, 1},         // Antispark wait
        {1, 1, 1, 1, 1, 1, 1, 1},         // Motor on
        {0, 0, 0, 0, 0, 0, 0, 0},         // Motor off
        {1, 0, 1, 1, 0, 0, 1, 0},         // Deadmanswitch off
};

uint8_t led_state_index = LED_STATE_UNSAFE_INIT;
uint8_t led_pattern_index = 0;
uint64_t lastLedSwitch = 0;
uint64_t last_heartbeat_switch;
bool last_heartbeat_state = false;
bool deadman_error = false;
uint64_t motorOnTime = 0;

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
    digitalWrite(PIN_OUT_LED, LOW);

    pinMode(PIN_OUT_RESISTOR, OUTPUT);
    pinMode(PIN_OUT_CONTACTOR, OUTPUT);
    pinMode(PIN_OUT_STATUSLED, OUTPUT);
    pinMode(PIN_OUT_LED, OUTPUT);

    delay(1000);
}

void loop() {
    // read inputs
    int8_t motorSwitch = digitalRead(PIN_IN_MOTOR);
    int8_t deadmanSwitch = digitalRead(PIN_IN_DEADMAN);
    digitalWrite(PIN_OUT_3, deadmanSwitch);

    // update state machine
    switch (ctrlState) {
        case CtrlState::NotSafeInit:
            if (motorSwitch == LOW && deadmanSwitch == LOW)
                ctrlState = CtrlState::MotorOff;
            break;
        case CtrlState::MotorOff:
            if (motorSwitch == HIGH) {
                ctrlState = CtrlState::AntisparkOn;
                motorOnTime = millis();
            }
            break;
        case CtrlState::AntisparkOn:
            if (motorSwitch == LOW) {
                ctrlState = CtrlState::MotorOff;
            } else if (millis() - motorOnTime > ANTISPARK_TIME_DELAY_MS) {
                ctrlState = CtrlState::MotorOn;
            }
            break;
        case CtrlState::MotorOn:
            if (motorSwitch == LOW) {
                ctrlState = CtrlState::MotorOff;
            }
            break;
    }
    if (deadmanSwitch == HIGH) {
        ctrlState = CtrlState::NotSafeInit;
        deadman_error = true;
    } else {
        deadman_error = false;
    }

    // ------------------------------------------------------------------------
    if (ctrlState == CtrlState::MotorOff || ctrlState == CtrlState::NotSafeInit) {
        digitalWrite(PIN_OUT_RESISTOR, LOW);
        digitalWrite(PIN_OUT_CONTACTOR, LOW);
    } else if (ctrlState == CtrlState::MotorOn) {
        digitalWrite(PIN_OUT_RESISTOR, HIGH);
        digitalWrite(PIN_OUT_CONTACTOR, HIGH);
    } else if (ctrlState == CtrlState::AntisparkOn) {
        digitalWrite(PIN_OUT_RESISTOR, HIGH);
        digitalWrite(PIN_OUT_CONTACTOR, LOW);
    }

    switch (ctrlState) {
        case CtrlState::NotSafeInit: {
            if (deadman_error) {
                led_state_index = LED_STATE_DEADMAN;
            } else {
                led_state_index = LED_STATE_UNSAFE_INIT;
            }
        }
            break;
        case CtrlState::MotorOff: {
            led_state_index = LED_STATE_MOTOR_OFF;
        }
            break;
        case CtrlState::AntisparkOn: {
            led_state_index = LED_STATE_ANTISPARK;
        }
            break;
        case CtrlState::MotorOn: {
            led_state_index = LED_STATE_MOTOR_ON;
        }
            break;
    }

    uint64_t time = millis();
    if (lastLedSwitch + LED_BLINK_TIME < time) {
        led_pattern_index = (led_pattern_index + 1) % 8;
        digitalWrite(PIN_OUT_STATUSLED, led_patterns[led_state_index][led_pattern_index] == 1 ? HIGH : LOW);
        lastLedSwitch = time;
    }

    // heartbeat
    if (last_heartbeat_switch + HEARTBEAT_TIME_MS < time) {
        digitalWrite(PIN_OUT_LED, last_heartbeat_state ? HIGH : LOW);
        last_heartbeat_switch = time;
        last_heartbeat_state = !last_heartbeat_state;
    }
}