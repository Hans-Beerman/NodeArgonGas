#include <Wire.h>
#include "MCP23017IO.h"
#include <Arduino.h>
//#include "RFID.h"

Adafruit_MCP23017 mcp;


#define INPUT_CHANGE_WINDOW     500 // in ms = time input changes are not taking into account
#define FET1_OUTPUT             11
#define FET2_OUTPUT             12
#define RELAY1_OUTPUT           8
#define RELAY2_OUTPUT           9
#define RELAY3_OUTPUT           10
#define SWITCH1_INPUT           0
#define SWITCH2_INPUT           1
#define SWITCH3_INPUT           2

#define PANIC_BUTTON            SWITCH1_INPUT
#define RESET_BUTTON            SWITCH2_INPUT
#define SIGNAL_SAFE_LAMP        RELAY2_OUTPUT
#define SIGNAL_ACTIVE_LAMP      RELAY3_OUTPUT
#define GREEN_LED               FET1_OUTPUT
#define BLUE_LED                SWITCH3_INPUT // for this node used as output
#define BUZZER                  FET2_OUTPUT

// unsigned long lastInputCheck = 0;
// int lastSwitch1Input = 0;

unsigned long lastResetButtonChecked = 0;
bool resetButtonPressed = false;
unsigned long lastPanicButtonChecked = 0;
bool panicButtonPressed = false;
bool signalSafeLampIsOn = false;
bool signalActiveLampIsOn = false;
bool greenLedIsOn = false;
bool blueLedIsOn = false;
bool buzzerIsOn = false;

void setup_MCP23017() {
    Wire.begin(RFID_SDA_PIN, RFID_SCL_PIN, RFID_I2C_FREQ);
    mcp.begin(&Wire);
    mcp.pinMode(RESET_BUTTON, INPUT);
    mcp.pullUp(RESET_BUTTON, HIGH);
    mcp.pinMode(PANIC_BUTTON, INPUT);
    mcp.pullUp(PANIC_BUTTON, HIGH);
    mcp.pinMode(SIGNAL_SAFE_LAMP, OUTPUT);
    mcp.digitalWrite(SIGNAL_SAFE_LAMP, 0);
    mcp.pinMode(SIGNAL_ACTIVE_LAMP, OUTPUT);
    mcp.digitalWrite(SIGNAL_ACTIVE_LAMP, 0);
    mcp.pinMode(GREEN_LED, OUTPUT);
    mcp.digitalWrite(GREEN_LED, 0);
    mcp.pinMode(BLUE_LED, OUTPUT);
    mcp.digitalWrite(BLUE_LED, 0);
    mcp.pinMode(BUZZER, OUTPUT);
    mcp.digitalWrite(BUZZER, 0);
}

void signalSafeLampOn() {
    if (!signalSafeLampIsOn) {
        mcp.digitalWrite(SIGNAL_SAFE_LAMP, 1);
        signalSafeLampIsOn = true;
    }
}

void signalSafeLampOff() {
    if (signalSafeLampIsOn) {
        mcp.digitalWrite(SIGNAL_SAFE_LAMP, 0);
        signalSafeLampIsOn = false;
    }
}

void signalActiveLampOn() {
    if (!signalActiveLampIsOn) {
        mcp.digitalWrite(SIGNAL_ACTIVE_LAMP, 1);
        signalActiveLampIsOn = true;
    }
}

void signalActiveLampOff() {
    if (signalActiveLampIsOn) {
        mcp.digitalWrite(SIGNAL_ACTIVE_LAMP, 0);
        signalActiveLampIsOn = false;
    }
}

void greenLedOn() {
    if (!greenLedIsOn) {
        mcp.digitalWrite(GREEN_LED, 1);
        greenLedIsOn = true;
    }
}

void greenLedOff() {
    if (greenLedIsOn) {
        mcp.digitalWrite(GREEN_LED, 0);
        greenLedIsOn = false;
    }
}

void blueLedOn() {
    if (!blueLedIsOn) {
        mcp.digitalWrite(BLUE_LED, 1);
        blueLedIsOn = true;
    }
}

void blueLedOff() {
    if (blueLedIsOn) {
        mcp.digitalWrite(BLUE_LED, 0);
        blueLedIsOn = false;
    }
}

void buzzerOn() {
    if (!buzzerIsOn) {
        mcp.digitalWrite(BUZZER, 1);
        buzzerIsOn = true;
    }
}

void buzzerOff() {
    if (buzzerIsOn) {
        mcp.digitalWrite(BUZZER, 0);
        buzzerIsOn = false;
    }
}

void loop_MCP23017() {
    unsigned long now = millis();
    bool currentResetButtonPressed = (mcp.digitalRead(RESET_BUTTON) == 0);
    if ((currentResetButtonPressed != resetButtonPressed) && ((now - lastResetButtonChecked) > INPUT_CHANGE_WINDOW)) {
        lastResetButtonChecked = now;
        resetButtonPressed = currentResetButtonPressed;
        if (resetButtonPressed) {
            Serial.print("Reset button pressed\n\r");
        } else {
            Serial.print("Reset button released\n\r");
        }
    }

    now = millis();
    bool currentPanicButtonPressed = (mcp.digitalRead(PANIC_BUTTON) == 0);
    if ((currentPanicButtonPressed != panicButtonPressed) && ((now - lastPanicButtonChecked) > INPUT_CHANGE_WINDOW)) {
        lastPanicButtonChecked = now;
        panicButtonPressed = currentPanicButtonPressed;
        if (panicButtonPressed) {
            Serial.print("Panic button pressed\n\r");
        } else {
            Serial.print("Panic button released\n\r");
        }
    }

}
