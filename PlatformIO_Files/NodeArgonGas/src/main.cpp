/*
   Copyright 2015-2018 Dirk-Willem van Gulik <dirkx@webweaving.org>
   Copyright 2020-2021 Hans Beerman <hans.beerman@xs4all.nl>
                       Stichting Makerspace Leiden, the Netherlands.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#ifndef ESP32
#error "The NodeArgonGase uses an ESP32 based board (Olimex ESP32-PoE)!"
#endif

// An Olimex ESP32-PoE is used (default is POESP board (Aart board))
#define ESP32_PoE


// for debugging
// see DEBUGIT flag in platformio.ini

// i2C params
// Defined in platformio.ini:
// RFID_SDA_PIN=13
// RFID_SCL_PIN=16

#include <Arduino.h>
#include <PowerNodeV11.h>
#include <ACNode.h>
#include <RFID.h>   // NFC version
#include <SIG2.h>
#include <Cache.h>
#include <OptoDebounce.h>
#include <ButtonDebounce.h>
#include <EEPROM.h>
#include <WiFiUdp.h>
#include <NTP.h> // install NTP by Stefan Staub
//
// information about NTP.h, see: https://platformio.org/lib/show/5438/NTP
//

#include "MCP23017IO.h"
#include "PressureSensor.h"
#include ".passwd.h"

/* If using WiFi instead of a fixed ethernet connection, and/or OTA:

Add Passwd.h to the NodeOven/src directoy containing the following information

#pragma once

#define WIFI_NETWORK "YOUR_SSID"
#define WIFI_PASSWD "YOUR_PASSWD"

#define OTA_PASSWD "YOUR_OTA_PASSWD"
*/

// software version
#define SOFTWARE_VERSION "  V0.0.0.1 "

#define MACHINE "argongas"

// See https://mailman.makerspaceleiden.nl/mailman/private/deelnemers/2019-February/019837.html

// Introduced by alex - 2020-01-8



#define SEND_MAIL_MESSAGES_RAW                  (true)  // set to false for encrypted messages (after test)
// Clear EEProm + Cache button
// Press BUT1 on Olimex ESP32 PoE module before (re)boot of node
// keep BUT1 pressed for at least 5 s
// After the release of BUT1 node will restart with empty EEProm and empty cache
#define CLEAR_EEPROM_AND_CACHE_BUTTON           (34)
#define CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED   (LOW)
#define MAX_WAIT_TIME_BUTTON_PRESSED            (4000)  // in ms

// #define MAX_WAIT_FOR_CARD_TIME                  (60) // in s = 60 seconds
// #define MAX_TIME_BEFORE_WARNING                 (60 * 60 * 4) // in s = 4 hour
// #define MAX_TIME_BEFORE_ALARM                   (60 * 60 * 2) // in s = 2 hour
// for test
#define MAX_WAIT_FOR_CARD_TIME                  (20) // in s = 20 seconds
#define MAX_TIME_BEFORE_WARNING                 (30) // in s = 30 seconds
#define MAX_TIME_BEFORE_ALARM                   (30) // in s = 30 seconds
// end for

#define CHECK_NFC_READER_AVAILABLE_TIME_WINDOW  (10000) // in ms  
#define GPIOPORT_I2C_RECOVER_RELAY              (15)       

#define CHECK_SWITCH1_INPUT_TIME_INTERVAL       (100) // in ms
#define CHECK_PRESSURE_TIME_INTERVAL            (100) // in ms

#define ERROR_LED_INTERVAL                      (500) // in ms

#define OPTO_COUPLER_INPUT1                     (32)
#define OPTO_COUPLER_INPUT2                     (33)  // in this node used as output
#define OPTO_COUPLER_INPUT3                     (36)

#define RED_LED                                 (OPTO_COUPLER_INPUT2)

#define PRESSURESENSOR                          (35)  // analog input
#define INFO_CALIBRATION_BUTTON                 (34)  // this is the same as the CLEAR_EEPROM_AND_CACHE_BUTTON, BUT1 of the ESP32-PoE
                                                      // pressing this button toggles Info / Calibration mode on of off. If on, IP address
                                                      // and calibration information will be logged via MQTT and telnet
// time window for calibration buttons
#define CALIB_WINDOW_TIME                       (5000) // in ms

bool checkCalibButtonsPressed = false;
long int checkCalibTimeOut = 0;
bool showInfoAndCalibration = false;

// Pressure limits, used to check if valve of gasbottle is open or not
#define PRESSURE_VALVE_OPEN                     (2.0) // Pressure if valve of Argon gas bottle is open
#define PRESSURE_VALVE_CLOSED                   (1.0) // If valve is cloded

#define BUTTON_INFO_CALIBRATION_PRESSED         (LOW) // the input level of te GPIO port used for button info and calibrations, if this button is pressed

// pressure sensor
PressureSensor thePressureSensor(PRESSURE_VALVE_OPEN, PRESSURE_VALVE_CLOSED);

// 230VAC optocoupler
OptoDebounce opto1(OPTO_COUPLER_INPUT1); // wired to the 230VAC of the welding equipment, to detect if it is switched on or not

ButtonDebounce buttonInfoCalibration(INFO_CALIBRATION_BUTTON, 150 /* mSeconds */); // buttonInfoCalibration is used to toggle Info / Calibration mode on/off

#ifdef WIFI_NETWORK
ACNode node = ACNode(MACHINE, WIFI_NETWORK, WIFI_PASSWD);
#else
ACNode node = ACNode(MACHINE);
#endif

#define USE_CACHE_FOR_TAGS true
#define USE_NFC_RFID_CARD true

// NTP update window
#define NTP_UPDATE_WINDOW (60000) // in ms
#define CHECK_ARGON_GAS_VALVE_CLOSED_HOUR       (1)

RFID reader = RFID(USE_CACHE_FOR_TAGS, USE_NFC_RFID_CARD); // use tags are stored in cache, to allow access in case the MQTT server is down; also use NFC RFID card

MqttLogStream mqttlogStream = MqttLogStream();
TelnetSerialStream telnetSerialStream = TelnetSerialStream();

#ifdef OTA_PASSWD
OTA ota = OTA(OTA_PASSWD);
#endif

WiFiUDP wifiUDP;
NTP ntp(wifiUDP);

char ntpBootDateTimeStr[64];
unsigned long NTPUpdatedTime = 0;
int previousHour = -1;

char ntpCurrentDateTimeStr[64];
int currentHour;
int currentMinutes;
int currentSecs;
int currentDay;
int currentMonth;
int currentYear;

bool errorLedIsOn = false;
unsigned long errorLedInterval = 0;
bool errorDeelnemersIsActive = false;
bool errorBestuurIsActive = false;
bool panicButtonWasPressed = false;
bool resetButtonWasPressed = false;

// LED aartLed = LED(SYSTEM_LED);    // defaults to the aartLed - otherwise specify a GPIO.

typedef enum {
  BOOTING,                  // starting up
  OUTOFORDER,               // device not functional.
  REBOOT,                   // forcefull reboot
  TRANSIENTERROR,           // hopefully goes away level error
  NOCONN,                   // sort of fairly hopless (though we can cache RFIDs!)
  NOTACTIVE,                // waiting for card.
  CHECKINGCARD,
  CLEARSTATUS,
  APPROVED,
  REJECTED,
  ACTIVENOTAPPROVED,
  SENDWARNINGACTIVENOTAPPROVED,
  WARNINGACTIVENOTAPPROVED,
  SENDALARMACTIVENOTAPPROVED,
  ALARMACTIVENOTAPPROVED,
  ACTIVEAPPROVED,
  SENDWARNINGACTIVEAPPROVED,
  WARNINGACTIVEAPPROVED,
  SENDALARMACTIVEAPPROVED,
  ALARMACTIVEAPPROVED,
} machinestates_t;


#define NEVER (0)

struct {
  const char * label;                   // name of this state
  LED::led_state_t ledState;            // flashing pattern for the aartLED. Zie ook https://wiki.makerspaceleiden.nl/mediawiki/index.php/Powernode_1.1.
  time_t maxTimeInMilliSeconds;         // how long we can stay in this state before we timeout.
  machinestates_t failStateOnTimeout;   // what state we transition to on timeout.
  unsigned long timeInState;
  unsigned long timeoutTransitions;
  unsigned long autoReportCycle;
} state[ALARMACTIVEAPPROVED + 1] =
{
  { "Booting",                           LED::LED_ERROR,                      120 * 1000, REBOOT,                       0 },
  { "Module out of order",               LED::LED_ERROR,                      120 * 1000, REBOOT,                       5 * 60 * 1000 },
  { "Rebooting",                         LED::LED_ERROR,                      120 * 1000, REBOOT,                       0 },
  { "Transient Error",                   LED::LED_ERROR,                        5 * 1000, NOTACTIVE,                    5 * 60 * 1000 },
  { "No network",                        LED::LED_FLASH,                           NEVER, NOCONN,                       0 },
  { "Not active",                        LED::LED_IDLE,                            NEVER, NOTACTIVE,                    0 },
  { "Checking card",                     LED::LED_PENDING,                      5 * 1000, REJECTED,                     0 },
  { "Clear status",                      LED::LED_PENDING,                         NEVER, NOTACTIVE,                    0 },
  { "Approved card",                     LED::LED_PENDING, MAX_WAIT_FOR_CARD_TIME * 1000, CLEARSTATUS,                  0 },
  { "Rejected",                          LED::LED_ERROR,                        5 * 1000, CLEARSTATUS,                  0 },
  { "Active, not approved",              LED::LED_FLASH,   MAX_WAIT_FOR_CARD_TIME * 1000, SENDWARNINGACTIVENOTAPPROVED, 0 },
  { "Send warning active, not approved", LED::LED_ERROR,                           NEVER, WARNINGACTIVENOTAPPROVED,     0 },
  { "Warning active, not approved",      LED::LED_ERROR,    MAX_TIME_BEFORE_ALARM * 1000, SENDALARMACTIVENOTAPPROVED,   0 },
  { "Send alarm active, not approved",   LED::LED_ERROR,                           NEVER, ALARMACTIVENOTAPPROVED,       0 },
  { "Alarm active, not approved",        LED::LED_ERROR,                           NEVER, ALARMACTIVENOTAPPROVED,       0 },
  { "Active and approved",               LED::LED_ON,     MAX_TIME_BEFORE_WARNING * 1000, SENDWARNINGACTIVEAPPROVED,    0 },
  { "Send warning active and approved",  LED::LED_ON,                              NEVER, WARNINGACTIVEAPPROVED,        0 },
  { "Warning active and approved",       LED::LED_ON,       MAX_TIME_BEFORE_ALARM * 1000, SENDALARMACTIVEAPPROVED,      0 },
  { "Send alarm active and approved",    LED::LED_ON,                              NEVER, ALARMACTIVEAPPROVED,          0 },
  { "Alarm active and approved",         LED::LED_ON,                              NEVER, ALARMACTIVEAPPROVED,          0 },
};

unsigned long laststatechange = 0, lastReport = 0;
static machinestates_t laststate = OUTOFORDER;
machinestates_t machinestate = BOOTING;

// to handle onconnect only once (only after reboot)
static bool firstOnConnectTime = true;

char reportStr[128];

unsigned long approvedCards = 0;
unsigned long rejectedCards = 0;

// For storing the local IP address of the node
IPAddress theLocalIPAddress;

unsigned long lastCheckNFCReaderTime = 0;

bool valveIsOpen = false;

bool valveIsOpenError = false;

bool previousWeldingIsOn = false;

bool previousPanicButtonPressed = false;

bool previousResetButtonPressed = false;

bool redLedIsOn = false;

bool nodeWasConnected = false;

void checkClearEEPromAndCacheButtonPressed(void) {
  unsigned long ButtonPressedTime;
  unsigned long currentSecs;
  unsigned long prevSecs;
  bool firstTime = true;

  // check CLEAR_EEPROM_AND_CACHE_BUTTON pressed
  pinMode(CLEAR_EEPROM_AND_CACHE_BUTTON, INPUT);
  // check if button is pressed for at least 3 s
  Log.println("Checking if the button is pressed for clearing EEProm and cache");
  ButtonPressedTime = millis();  
  prevSecs = MAX_WAIT_TIME_BUTTON_PRESSED / 1000;
  Log.print(prevSecs);
  Log.print(" s");
  while (digitalRead(CLEAR_EEPROM_AND_CACHE_BUTTON) == CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED) {
    if ((millis() - ButtonPressedTime) >= MAX_WAIT_TIME_BUTTON_PRESSED) {
      if (firstTime == true) {
        Log.print("\rPlease release button");
        firstTime = false;
      }
    } else {
      currentSecs = (MAX_WAIT_TIME_BUTTON_PRESSED - millis()) / 1000;
      if ((currentSecs != prevSecs) && (currentSecs >= 0)) {
        Log.print("\r");
        Log.print(currentSecs);
        Log.print(" s");
        prevSecs = currentSecs;
      }
    }
  }
  if (millis() - ButtonPressedTime >= MAX_WAIT_TIME_BUTTON_PRESSED) {
    Log.print("\rButton for clearing EEProm and cache was pressed for more than ");
    Log.print(MAX_WAIT_TIME_BUTTON_PRESSED / 1000);
    Log.println(" s, EEProm and Cache will be cleared!");
    // Clear EEPROM
    EEPROM.begin(1024);
    wipe_eeprom();
    Log.println("EEProm cleared!");
    // Clear cache
    prepareCache(true);
    Log.println("Cache cleared!");
    // wait until button is released, than reboot
    while (digitalRead(CLEAR_EEPROM_AND_CACHE_BUTTON) == CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED) {
      // do nothing here
    }
    Log.println("Node will be restarted");
    // restart node
    ESP.restart();
  } else {
    Log.println("\rButton was not (or not long enough) pressed to clear EEProm and cache");
  }
}

void setup_GPIO() {
  // for recovery relay I2C
  pinMode(GPIOPORT_I2C_RECOVER_RELAY, OUTPUT);
  digitalWrite(GPIOPORT_I2C_RECOVER_RELAY, 0);
  // for red LED
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);

  // pinMode(OPTO_COUPLER_INPUT1, INPUT);
}

void redLedOn() {
  if (!redLedIsOn) {
    digitalWrite(RED_LED, HIGH);
    redLedIsOn = true;
  }
}

void redLedOff() {
  if (redLedIsOn) {
    digitalWrite(RED_LED, LOW);
    redLedIsOn = false;
  }
}

#define NOT_APPROVED_WARNING_INTERVAL_TIME    (2000)
#define NOT_APPROVED_WARNING_TIME             (200)
#define NOT_APPROVED_ALARM_INTERVAL_TIME      (500)
#define NOT_APPROVED_ALARM_TIME               (500)
#define APPROVED_WARNING_INTERVAL_TIME        (4000)
#define APPROVED_WARNING_TIME                 (300)
#define APPROVED_ALARM_INTERVAL_TIME          (500)
#define APPROVED_ALARM_TIME                   (500)
#define BUZZER_NOT_APPROVED_WARNING_TIME      (400)
#define BUZZER_NOT_APPROVED_ALARM_TIME        (300)
#define BUZZER_APPROVED_ALARM_TIME            (300)
#define CONNECTION_ERROR_INTERVAL_TIME        (1000)
#define CONNECTION_ERROR_TIME                 (100)


bool notApprovedWarningIsActive = false;
bool notApprovedAlarmIsActive = false;
bool approvedWarningIsActive = false;
bool approvedAlarmIsActive = false;
bool buzzerIsActive = false;
bool connectionErrorIsActive = false;
bool signalIsOn = false;

unsigned long notApprovedWarningTime = 0;
unsigned long notApprovedAlarmTime = 0;
unsigned long approvedWarningTime = 0;
unsigned long approvedAlarmTime = 0;
unsigned long buzzerOnTime = 0;
unsigned long connectionErrorTime = 0;

void setNotApprovedWarning(void) {
  notApprovedWarningIsActive = true;
  notApprovedAlarmIsActive = false;
  approvedWarningIsActive = false;
  approvedAlarmIsActive = false;
  notApprovedWarningTime = millis();
  signalIsOn = true;
  signalSafeLampOff();
  signalActiveLampOn();
}

void setNotApprovedAlarm(void) {
  notApprovedWarningIsActive = false;
  notApprovedAlarmIsActive = true;
  approvedWarningIsActive = false;
  approvedAlarmIsActive = false;
  notApprovedAlarmTime = millis();
  signalIsOn = true;
  signalSafeLampOff();
  signalActiveLampOn();
}

void setApprovedWarning(void) {
  notApprovedWarningIsActive = false;
  notApprovedAlarmIsActive = false;
  approvedWarningIsActive = true;
  approvedAlarmIsActive = false;
  approvedWarningTime = millis();
  signalIsOn = true;
  signalSafeLampOff();
  signalActiveLampOn();
}

void setApprovedAlarm(void) {
  notApprovedWarningIsActive = false;
  notApprovedAlarmIsActive = false;
  approvedWarningIsActive = false;
  approvedAlarmIsActive = true;
  approvedAlarmTime = millis();
  signalIsOn = true;
  signalSafeLampOff();
  signalActiveLampOn();
}

void disableAllWarningsAndAlarms(void) {
  notApprovedWarningIsActive = false;
  notApprovedAlarmIsActive = false;
  approvedWarningIsActive = false;
  approvedAlarmIsActive = false;
  if ((machinestate == ACTIVENOTAPPROVED) || (machinestate == ACTIVEAPPROVED) || (machinestate == SENDWARNINGACTIVENOTAPPROVED) || (machinestate == SENDWARNINGACTIVEAPPROVED) ||
      (machinestate == WARNINGACTIVENOTAPPROVED) || (machinestate == WARNINGACTIVEAPPROVED) || (machinestate == SENDALARMACTIVENOTAPPROVED) || (machinestate == SENDALARMACTIVEAPPROVED) ||
      (machinestate == ALARMACTIVENOTAPPROVED) || (machinestate == ALARMACTIVEAPPROVED)) {
    signalActiveLampOn();
  } else {
    signalActiveLampOff();
    signalSafeLampOn();
  }
  buzzerIsActive = false;
  buzzerOff();
}

void warningsAndAlarms_Loop(void) {
  if (connectionErrorIsActive) {
    if ((machinestate != WARNINGACTIVENOTAPPROVED) && (machinestate != ALARMACTIVENOTAPPROVED) && (machinestate != ALARMACTIVEAPPROVED)) {
      if (buzzerIsActive) {
        if (millis() - buzzerOnTime > CONNECTION_ERROR_TIME) {
          buzzerIsActive = false;
          buzzerOff();
          buzzerOnTime += CONNECTION_ERROR_TIME;
        }
      } else {
        if (millis() - buzzerOnTime > CONNECTION_ERROR_INTERVAL_TIME) {
          buzzerIsActive = true;
          buzzerOn();
          buzzerOnTime += CONNECTION_ERROR_INTERVAL_TIME;
        }
      }
    }
  }

  if (notApprovedWarningIsActive) {
    if (signalIsOn) {
      if (millis() - notApprovedWarningTime > NOT_APPROVED_WARNING_TIME) {
        signalActiveLampOff();
        signalIsOn = false;
        notApprovedWarningTime += NOT_APPROVED_WARNING_TIME;
      }
    } else {
      if (millis() - notApprovedWarningTime > NOT_APPROVED_WARNING_INTERVAL_TIME) {
        signalActiveLampOn();
        signalIsOn = true;
        notApprovedWarningTime += NOT_APPROVED_WARNING_INTERVAL_TIME;
        buzzerOnTime = millis();
        buzzerOn();
        buzzerIsActive = true;
      }
    }
    if (buzzerIsActive) {
      if (millis() - buzzerOnTime > BUZZER_NOT_APPROVED_WARNING_TIME) {
        buzzerIsActive = false;
        buzzerOff();
      }
    }
    return;
  }

  if (notApprovedAlarmIsActive) {
    if (signalIsOn) {
      if (millis() - notApprovedAlarmTime > NOT_APPROVED_ALARM_TIME) {
        signalActiveLampOff();
        signalIsOn = false;
        notApprovedAlarmTime += NOT_APPROVED_ALARM_TIME;
      }
    } else {
      if (millis() - notApprovedAlarmTime > NOT_APPROVED_ALARM_INTERVAL_TIME) {
        signalActiveLampOn();
        signalIsOn = true;
        notApprovedAlarmTime += NOT_APPROVED_ALARM_INTERVAL_TIME;
        buzzerOnTime = millis();
        buzzerOn();
        buzzerIsActive = true;
      }
    }
    if (millis() - buzzerOnTime > BUZZER_NOT_APPROVED_ALARM_TIME) {
      buzzerIsActive = false;
      buzzerOff();
    }

    return;
  }

  if (approvedWarningIsActive) {
    if (signalIsOn) {
      if (millis() - approvedWarningTime > APPROVED_WARNING_TIME) {
        signalActiveLampOff();
        signalIsOn = false;
        approvedWarningTime += APPROVED_WARNING_TIME;
      }
    } else {
      if (millis() - approvedWarningTime > APPROVED_WARNING_INTERVAL_TIME) {
        signalActiveLampOn();
        signalIsOn = true;
        approvedWarningTime += APPROVED_WARNING_INTERVAL_TIME;
      }
    }
    return;
  }

  if (approvedAlarmIsActive) {
    if (signalIsOn) {
      if (millis() - approvedAlarmTime > APPROVED_ALARM_TIME) {
        signalActiveLampOff();
        signalIsOn = false;
        approvedAlarmTime += APPROVED_ALARM_TIME;
      }
    } else {
      if (millis() - approvedAlarmTime > APPROVED_ALARM_INTERVAL_TIME) {
        signalActiveLampOn();
        signalIsOn = true;
        approvedAlarmTime += APPROVED_ALARM_INTERVAL_TIME;
        buzzerOnTime = millis();
        buzzerOn();
        buzzerIsActive = true;
      }
    }
    if (buzzerIsActive) {
      if (millis() - buzzerOnTime > BUZZER_APPROVED_ALARM_TIME) {
        buzzerIsActive = false;
        buzzerOff();
      }
    }
    return;
  }

}

void resetNFCReader() {
  if (USE_NFC_RFID_CARD) {
    pinMode(RFID_SCL_PIN, OUTPUT);
    digitalWrite(RFID_SCL_PIN, 0);
    pinMode(RFID_SDA_PIN, OUTPUT);
    digitalWrite(RFID_SDA_PIN, 0);
    digitalWrite(GPIOPORT_I2C_RECOVER_RELAY, 1);
    delay(500);
    digitalWrite(GPIOPORT_I2C_RECOVER_RELAY, 0);
    reader.begin();
  }
}

void checkNFCReaderAvailable() {
  if (USE_NFC_RFID_CARD) {
    if (!reader.CheckPN53xBoardAvailable()) {
      // Error in communuication with RFID reader, try resetting communication
      pinMode(RFID_SCL_PIN, OUTPUT);
      digitalWrite(RFID_SCL_PIN, 0);
      pinMode(RFID_SDA_PIN, OUTPUT);
      digitalWrite(RFID_SDA_PIN, 0);
      digitalWrite(GPIOPORT_I2C_RECOVER_RELAY, 1);
      delay(500);
      digitalWrite(GPIOPORT_I2C_RECOVER_RELAY, 0);
      reader.begin();
    }
  }
}

void getCurrentNTPDateTime() {
  currentHour = ntp.hours();
  currentMinutes = ntp.minutes();
  currentSecs = ntp.seconds();
  currentDay = ntp.day();
  currentMonth = ntp.month();
  currentYear = ntp.year();
  snprintf(ntpCurrentDateTimeStr, sizeof(ntpCurrentDateTimeStr), "%4d-%02d-%02d %02d:%02d:%02d", currentYear, currentMonth, currentDay, currentHour, currentMinutes, currentSecs);
}

void initNTP(void) {
  ntp.ruleDST("CEST", Last, Sun, Mar, 2, 120); // last sunday in march 2:00, timezone +120min (+1 GMT + 1h summertime offset)
  ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60); // last sunday in october 3:00, timezone +60min (+1 GMT)
  ntp.begin();
  ntp.update();

  getCurrentNTPDateTime();

  snprintf(ntpBootDateTimeStr, sizeof(ntpBootDateTimeStr), "Last boot: %s", ntpCurrentDateTimeStr);
}

void checkDeelnemerErrorIsActive(void) {
  char mailPayload[255];
	if (errorDeelnemersIsActive) {
    getCurrentNTPDateTime();
    if (resetButtonWasPressed) {
      snprintf(mailPayload, sizeof(mailPayload), "{ \"nodename\": \"%s\",  \"subject\": \"Valve argon gas error solved? Valve argon gas error was solved by pressing the reset button\", \"DateTime\": \"%s\" }", MACHINE, ntpCurrentDateTimeStr);
    } else {
      snprintf(mailPayload, sizeof(mailPayload), "{ \"nodename\": \"%s\",  \"subject\": \"Valve argon gas error solved? Valve argon gas error is solved\", \"DateTime\": \"%s\" }", MACHINE, ntpCurrentDateTimeStr);
    }
    node.send("deelnemers", mailPayload, SEND_MAIL_MESSAGES_RAW);
    if (resetButtonWasPressed) {
      Serial.printf("%s Valve argon gas error was solved by pressing the reset button\n\r", ntpCurrentDateTimeStr);
    } else {
      Serial.printf("%s Valve argon gas error is solved\n\r", ntpCurrentDateTimeStr);
    }
    errorDeelnemersIsActive = false;
  }
}

void checkBestuurderErrorIsActive(void) {
  char mailPayload[255];
	if (errorBestuurIsActive) {
    getCurrentNTPDateTime();
    if (resetButtonWasPressed) {
      snprintf(mailPayload, sizeof(mailPayload), "{ \"nodename\": \"%s\",  \"subject\": \"Valve argon gas error solved? Valve argon gas error was solved by pressing the reset button\", \"DateTime\": \"%s\" }", MACHINE, ntpCurrentDateTimeStr);
    } else {
      snprintf(mailPayload, sizeof(mailPayload), "{ \"nodename\": \"%s\",  \"subject\": \"Valve argon gas error solved? Valve argon gas error is solved\", \"DateTime\": \"%s\" }", MACHINE, ntpCurrentDateTimeStr);
    }
    node.send("bestuur", mailPayload, SEND_MAIL_MESSAGES_RAW);
    if (resetButtonWasPressed) {
      Serial.printf("%s Valve argon gas error was solved by pressing the reset button\n\r", ntpCurrentDateTimeStr);
    } else {
      Serial.printf("%s Valve argon gas error is solved\n\r", ntpCurrentDateTimeStr);
    }
    errorBestuurIsActive = false;
  }
}

unsigned long lastSignalsCheck = 0;

void checkSignals_Loop(void) {
  unsigned long now = millis();

  if ((now - lastSignalsCheck) > 1000) {
    lastSignalsCheck = now;
    if (node.isUp() && node.isConnected()) {
      if (!nodeWasConnected) {
        nodeWasConnected = true;
        greenLedOn();
        redLedOff();
        connectionErrorIsActive = false;
        if ((machinestate != WARNINGACTIVENOTAPPROVED) && (machinestate != ALARMACTIVENOTAPPROVED) && (machinestate != ALARMACTIVEAPPROVED)) {
          buzzerOff();
        }
      }
    } else {
      if (nodeWasConnected) {
        nodeWasConnected = false;
        redLedOn();
        greenLedOff();
        connectionErrorIsActive = true;
        if ((machinestate != WARNINGACTIVENOTAPPROVED) && (machinestate != ALARMACTIVENOTAPPROVED) && (machinestate != ALARMACTIVEAPPROVED)) {
          buzzerOnTime = millis();
        }
      }
    }
  }

  bool tmpPanicButtonPressed = panicButtonPressed;
  if (tmpPanicButtonPressed != previousPanicButtonPressed) {
    previousPanicButtonPressed = tmpPanicButtonPressed;
    if (previousPanicButtonPressed) {
//      if ((machinestate == ACTIVENOTAPPROVED) || (machinestate == SENDWARNINGACTIVENOTAPPROVED) || (machinestate == WARNINGACTIVENOTAPPROVED) || (machinestate == SENDALARMACTIVENOTAPPROVED) || (machinestate == ALARMACTIVENOTAPPROVED)) {
      if ((machinestate == ACTIVENOTAPPROVED) || (machinestate == SENDWARNINGACTIVENOTAPPROVED) || (machinestate == WARNINGACTIVENOTAPPROVED)) {
        machinestate = SENDALARMACTIVENOTAPPROVED;
        panicButtonWasPressed = true;
      } else {
//        if ((machinestate == ACTIVEAPPROVED) || (machinestate == SENDWARNINGACTIVEAPPROVED) || (machinestate == WARNINGACTIVEAPPROVED) || (machinestate == SENDALARMACTIVEAPPROVED) || (machinestate == ALARMACTIVEAPPROVED)) {
        if ((machinestate == ACTIVEAPPROVED) || (machinestate == SENDWARNINGACTIVEAPPROVED) || (machinestate == WARNINGACTIVEAPPROVED)) {
          machinestate = SENDALARMACTIVEAPPROVED;
          panicButtonWasPressed = true;
        }
      }
    }
  }

  bool tmpResetButtonPressed = resetButtonPressed;
  if (tmpResetButtonPressed != previousResetButtonPressed) {
    previousResetButtonPressed = tmpResetButtonPressed;
    if (previousResetButtonPressed) {
      if ((machinestate == SENDWARNINGACTIVENOTAPPROVED) || (machinestate == WARNINGACTIVENOTAPPROVED) || (machinestate == SENDALARMACTIVENOTAPPROVED) || (machinestate == ALARMACTIVENOTAPPROVED)) {
        machinestate = ACTIVENOTAPPROVED;
        resetButtonWasPressed = true;
        disableAllWarningsAndAlarms();
      } else {
        if ((machinestate == SENDWARNINGACTIVEAPPROVED) || (machinestate == WARNINGACTIVEAPPROVED) || (machinestate == SENDALARMACTIVEAPPROVED) || (machinestate == ALARMACTIVEAPPROVED)) {
          machinestate = ACTIVEAPPROVED;
          resetButtonWasPressed = true;
        }
      }
      checkDeelnemerErrorIsActive();
      checkBestuurderErrorIsActive();
      resetButtonWasPressed = false;
      disableAllWarningsAndAlarms();
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n\n");
  Serial.println("Booted: " __FILE__ " " __DATE__ " " __TIME__ );
  
  setup_GPIO();

  setup_MCP23017();

  redLedOff();
  greenLedOff();
  blueLedOff();
  buzzerOff();
  signalSafeLampOff();
  signalActiveLampOff();

  checkClearEEPromAndCacheButtonPressed();

  node.set_mqtt_prefix("ac");
  node.set_master("master");

  node.onConnect([]() {
    redLedOff();
    greenLedOn();
    Log.println("Connected");
    if (firstOnConnectTime == true) {
      firstOnConnectTime = false;
      machinestate = NOTACTIVE;
    }
  });
  node.onDisconnect([]() {
    redLedOn();
    greenLedOff();
    Log.println("Disconnected");
  });
  node.onError([](acnode_error_t err) {
    Log.printf("Error %d\n", err);
    machinestate = NOTACTIVE;
  });
  node.onApproval([](const char * machine) {
    Debug.print("Got approve for using the argon gas bottle: \n\r");

    getCurrentNTPDateTime();

    Log.printf("%s => Approve received to use the argon gas bottle\n\r", ntpCurrentDateTimeStr);    
    
    checkDeelnemerErrorIsActive();
    checkBestuurderErrorIsActive();
    resetButtonWasPressed = false;
    Debug.println(machine);
    if ((machinestate == NOTACTIVE) || (machinestate == CHECKINGCARD) || (machinestate == ACTIVENOTAPPROVED)|| (machinestate == WARNINGACTIVENOTAPPROVED) || (machinestate == ALARMACTIVENOTAPPROVED)) {
      approvedCards++;
      if (valveIsOpen) {
        disableAllWarningsAndAlarms();
        signalActiveLampOn();
        signalSafeLampOff();
        machinestate = ACTIVEAPPROVED;
        Serial.print("User approved and gas is open!\n\r");
        blueLedOn();
      } else {
        machinestate = APPROVED;
        Serial.print("User approved, open valve now!\n\r");
        blueLedOn();
      }
      Log.println("User is approved");
      valveIsOpenError = false;
    }
  });

  node.onDenied([](const char * machine) {
    Debug.println("Got denied");
    Serial.print("User is denied!\n\r");

    getCurrentNTPDateTime();

    Log.printf("%s => Deny received for using the argon gas bottle\n\r", ntpCurrentDateTimeStr);    

    if ((machinestate != ACTIVEAPPROVED) && (machinestate != APPROVED)) {
      machinestate = REJECTED;
    }
    rejectedCards++;
  });

  // For test:  
  // node.set_report_period(10 * 1000);
  node.set_report_period(REPORT_PERIOD);
  node.onReport([](JsonObject  & report) {
    report["state"] = state[machinestate].label;

#ifdef OTA_PASSWD
    report["ota"] = true;
#else
    report["ota"] = false;
#endif
    snprintf(reportStr, sizeof(reportStr), "%lu", approvedCards);
    report["approved cards"] = reportStr;
    snprintf(reportStr, sizeof(reportStr), "%lu", rejectedCards);
    report["rejected cards"] = reportStr;

    theLocalIPAddress = node.localIP();
    report["IP_address"] = theLocalIPAddress.toString();
    report["Last_Reboot"] = ntpBootDateTimeStr;
  });

  buttonInfoCalibration.setCallback([](int state) {
    if (state > 0) {
      showInfoAndCalibration = !showInfoAndCalibration;
      if (showInfoAndCalibration) {
        Serial.print("Show calibration info = on\n\r");
      } else {
        Serial.print("Show calibration info = off\n\r");
      }
    }
  });


  reader.onSwipe([](const char * tag) -> ACBase::cmd_result_t {
    // avoid swithing messing with the swipe process
    if ((machinestate == ACTIVEAPPROVED) || (machinestate == APPROVED) || (machinestate == SENDWARNINGACTIVEAPPROVED) || (machinestate == WARNINGACTIVEAPPROVED) || (machinestate == SENDALARMACTIVEAPPROVED) || (machinestate == ALARMACTIVEAPPROVED)) {
      Debug.printf("Ignoring a normal swipe - gas is already open, or RFID card is already approved.");
      checkNFCReaderAvailable();
      return ACBase::CMD_CLAIMED;
    }

    // We'r declining so that the core library handle sending
    // an approval request, keep state, and so on.
    //
    Debug.printf("Detected a normal swipe.\n");
    checkNFCReaderAvailable();
    machinestate = CHECKINGCARD;
  //  buzz = CHECK;
    return ACBase::CMD_DECLINE;
  });


  // This reports things such as FW version of the card; which can 'wedge' it. So we
  // disable it unless we absolutely positively need that information.
  //
  reader.set_debug(false);
  node.addHandler(&reader);
 
#ifdef OTA_PASSWD
  node.addHandler(&ota);
#endif

  Log.addPrintStream(std::make_shared<MqttLogStream>(mqttlogStream));

  auto t = std::make_shared<TelnetSerialStream>(telnetSerialStream);
  Log.addPrintStream(t);
  Debug.addPrintStream(t);

  // node.set_debug(true);
  // node.set_debugAlive(true);

  redLedOn();

// if Olimex ESP32-PoE board is used
#ifdef ESP32_PoE  
  node.begin(BOARD_OLIMEX);
#endif

  initNTP();

// if default, board (POESP, board Aart) is used
#ifndef ESP32_PoE
  node.begin();
#endif
  Log.println("Booted: " __FILE__ " " __DATE__ " " __TIME__ );
  Log.println(ntpBootDateTimeStr);

  if (thePressureSensor.valveIsOpen()) {
    signalActiveLampOn();
    signalSafeLampOff();
  } else {
    signalSafeLampOn();
    signalActiveLampOff();
  }
  resetNFCReader();
}

unsigned long now;
unsigned long lastCheckSwitch1 = 0;
// int switch1Input = -1;
// int prevSwitch1Input = 0;
bool currentResetButtonPressed = false;

unsigned long lastCheckPressure = 0;
bool currentValveIsOpen = false;
bool previousValveIsOpen = false;

char mailPayload[255];

void checkArgonGasIsOpen(void) {
  int currentHour;

  currentHour = ntp.hours();
  if (previousHour != currentHour) {
    previousHour = currentHour;

    if (currentHour == CHECK_ARGON_GAS_VALVE_CLOSED_HOUR) {
      if (valveIsOpen) {
        // ****** sent error message, valve still open at end time
        getCurrentNTPDateTime();
        snprintf(mailPayload, sizeof(mailPayload), "{ \"nodename\": \"%s\",  \"subject\": \"Valve argon gas bottle still open? The gas pressure was not low at %02d:00 hr\", \"DateTime\": \"%s\" }", MACHINE, CHECK_ARGON_GAS_VALVE_CLOSED_HOUR, ntpCurrentDateTimeStr);
        node.send("deelnemers", mailPayload, SEND_MAIL_MESSAGES_RAW);
        Serial.printf("%s Valve argon gas bottle still open? The gas pressure was not low at %02d:00 hr\n\r", ntpCurrentDateTimeStr, CHECK_ARGON_GAS_VALVE_CLOSED_HOUR);
        valveIsOpenError = true;
        errorDeelnemersIsActive = true;
      }
    }
  }
}


void optocoupler_loop() {

  opto1.loop();

  bool tmpWeldingIsOn = opto1.state();
  if (tmpWeldingIsOn != previousWeldingIsOn) {
    previousWeldingIsOn = tmpWeldingIsOn;
    if (previousWeldingIsOn) {
        if ((machinestate == SENDWARNINGACTIVENOTAPPROVED) || (machinestate == WARNINGACTIVENOTAPPROVED) || (machinestate == SENDALARMACTIVENOTAPPROVED) || (machinestate == ALARMACTIVENOTAPPROVED)) {
        machinestate = ACTIVENOTAPPROVED;
        resetButtonWasPressed = true;
        disableAllWarningsAndAlarms();
      } else {
        if ((machinestate == SENDWARNINGACTIVEAPPROVED) || (machinestate == WARNINGACTIVEAPPROVED) || (machinestate == SENDALARMACTIVEAPPROVED) || (machinestate == ALARMACTIVEAPPROVED)) {
          machinestate = ACTIVEAPPROVED;
          resetButtonWasPressed = true;
        }
      }
      checkDeelnemerErrorIsActive();
      checkBestuurderErrorIsActive();
      resetButtonWasPressed = false;
      disableAllWarningsAndAlarms();
      Serial.print("Welding = on\n\r");
    } else {
      if (machinestate == ACTIVENOTAPPROVED) {
        machinestate = SENDWARNINGACTIVENOTAPPROVED;
      } else {
        if (machinestate == ACTIVEAPPROVED) {
          machinestate = SENDWARNINGACTIVEAPPROVED;
        }
      }
      Serial.print("Welding = off\n\r");      
    }
  }
}

unsigned long pressureSampleTime = 0;

void loop() {
  node.loop();

  now = millis();

  optocoupler_loop();

  checkSignals_Loop();

  thePressureSensor.loop();

  if (showInfoAndCalibration && thePressureSensor.newCalibrationInfoAvailable) {
      Log.println("Compressor Node Info");
      Log.print("Software version :");
      Log.println(SOFTWARE_VERSION);
      theLocalIPAddress = node.localIP();
      Log.print("IP address: ");
      Log.println(theLocalIPAddress.toString());
      thePressureSensor.logInfoCalibration();
      Log.println("");
    }

  if ((now - pressureSampleTime) > 5000) {
    pressureSampleTime = now;
    Serial.printf("Argon gas pressure = %0.5f bar\n\r", thePressureSensor.currentPressure());
  }


  if ((now - NTPUpdatedTime) > NTP_UPDATE_WINDOW) {
    ntp.update();
    NTPUpdatedTime = now;
  }

  if ((now - lastCheckSwitch1) > CHECK_SWITCH1_INPUT_TIME_INTERVAL) {
    loop_MCP23017();
    lastCheckSwitch1 = now;
/*
    currentResetButtonPressed = resetButtonPressed;
    if (currentResetButtonPressed != previousResetButtonPressed) {
      previousResetButtonPressed = resetButtonPressed;
    }
*/    
  }

  if ((now - lastCheckPressure) > CHECK_PRESSURE_TIME_INTERVAL) {
    lastCheckPressure = now;
    currentValveIsOpen = thePressureSensor.valveIsOpen();
    if (currentValveIsOpen != previousValveIsOpen) {
      previousValveIsOpen = currentValveIsOpen;

      getCurrentNTPDateTime();

      if (currentValveIsOpen) {
        valveIsOpen = true;

        Log.printf("%s => Valve is open\n\r", ntpCurrentDateTimeStr);

        if (machinestate == APPROVED) {
          Serial.print("Valve is open and user is already approved!\n\r");
          machinestate = ACTIVEAPPROVED;
        } else {
          Serial.print("Valve is open, while user is not approved yet, use RFID card now!\n\r");
          machinestate = ACTIVENOTAPPROVED;
        }
        signalActiveLampOn();
        signalSafeLampOff();
      } else {
        valveIsOpen = false;

        Log.printf("%s => Valve is closed\n\r", ntpCurrentDateTimeStr);

        machinestate = NOTACTIVE;

        disableAllWarningsAndAlarms();
        Serial.print("Valve is closed\n\r");
        checkDeelnemerErrorIsActive();
        checkBestuurderErrorIsActive();
        resetButtonWasPressed = false;
        blueLedOff();
      }
      valveIsOpenError = false;
    }
  }

  if (USE_NFC_RFID_CARD) {
    if ((now - lastCheckNFCReaderTime) > CHECK_NFC_READER_AVAILABLE_TIME_WINDOW) {
      lastCheckNFCReaderTime = now;
      Serial.print("Check Reader Available\n\r");
      checkNFCReaderAvailable();
    }
  }

  if (laststate != machinestate) {
    Debug.printf("Changed from state <%s> to state <%s>\n",
                 state[laststate].label, state[machinestate].label);

    state[laststate].timeInState += (millis() - laststatechange) / 1000;
    laststate = machinestate;
    laststatechange = millis();
    return;
  }

  warningsAndAlarms_Loop();

  if (state[machinestate].maxTimeInMilliSeconds != NEVER &&
      ((millis() - laststatechange) > state[machinestate].maxTimeInMilliSeconds))
  {
    state[machinestate].timeoutTransitions++;

    laststate = machinestate;
    machinestate = state[machinestate].failStateOnTimeout;

    Log.printf("Time-out; transition from <%s> to <%s>\n",
               state[laststate].label, state[machinestate].label);
    return;
  };

  if (state[machinestate].autoReportCycle && \
      (millis() - laststatechange) > state[machinestate].autoReportCycle && \
      (millis() - lastReport) > state[machinestate].autoReportCycle)
  {
    Log.printf("State: %s now for %lu seconds", state[laststate].label, (millis() - laststatechange) / 1000);
    lastReport = millis();
  };

  switch (machinestate) {
    case REBOOT:
      greenLedOff();
      redLedOff();
      signalActiveLampOff();
      signalSafeLampOff();
      node.delayedReboot();
      break;

    case NOTACTIVE:
    case CHECKINGCARD:
      break;
    case CLEARSTATUS:
      blueLedOff();
      machinestate = NOTACTIVE;
      break;
    case REJECTED:
      rejectedCards++;
      if (valveIsOpen) {
        machinestate = ACTIVENOTAPPROVED;

        getCurrentNTPDateTime();

        Log.printf("%s => RFID tag/card rejected\n\r", ntpCurrentDateTimeStr);    

        // ****** sent error message, valve is open, but card is rejected
        getCurrentNTPDateTime();
        snprintf(mailPayload, sizeof(mailPayload), "{ \"nodename\": \"%s\",  \"subject\": \"Valve argon gas bottle not closed, while RFID card is rejected\", \"DateTime\": \"%s\" }", MACHINE, ntpCurrentDateTimeStr);
        node.send("deelnemers", mailPayload, SEND_MAIL_MESSAGES_RAW);
        node.send("bestuur", mailPayload, SEND_MAIL_MESSAGES_RAW);
        Serial.printf("%s Valve argon gas bottle not closed, while RFID card is rejected\n\r", ntpCurrentDateTimeStr);
        valveIsOpenError = true;
        errorDeelnemersIsActive = true;
        errorBestuurIsActive = true;
      } else {
        machinestate = NOTACTIVE;
      }
      break;
    case ACTIVENOTAPPROVED:
      break;
    case SENDWARNINGACTIVENOTAPPROVED:
        if (valveIsOpen) {
          setNotApprovedWarning();
          // ****** sent error message, valve still open after max time
          getCurrentNTPDateTime();
          snprintf(mailPayload, sizeof(mailPayload), "{ \"nodename\": \"%s\",  \"subject\": \"Valve argon gas still open? While not approved, gas pressure detected for at least %d hours\", \"DateTime\": \"%s\" }", MACHINE, MAX_TIME_BEFORE_WARNING / 3600, ntpCurrentDateTimeStr);
          node.send("deelnemers", mailPayload, SEND_MAIL_MESSAGES_RAW);
          node.send("bestuur", mailPayload, SEND_MAIL_MESSAGES_RAW);
          Serial.printf("%s Valve argon gas still open? While not approved, gas pressure detected for at least %d hours\n\r", ntpCurrentDateTimeStr, MAX_TIME_BEFORE_WARNING / 3600);
          valveIsOpenError = true;
          errorDeelnemersIsActive = true;
          errorBestuurIsActive = true;
          machinestate = WARNINGACTIVENOTAPPROVED;
        } else {
          machinestate = NOTACTIVE;
        }
      break;
    case WARNINGACTIVENOTAPPROVED:
      break;
    case SENDALARMACTIVENOTAPPROVED:
        // ****** sent error message, valve still open after max time
      if (valveIsOpen) {
        setNotApprovedAlarm();
        // ****** sent error message, valve is open, but no card approved yet
        getCurrentNTPDateTime();
        if (panicButtonWasPressed) {
          snprintf(mailPayload, sizeof(mailPayload), "{ \"nodename\": \"%s\",  \"subject\": \"Panic button was pressed? Panic button was pressed while valve is open but no RFID card was seen \", \"DateTime\": \"%s\" }", MACHINE, ntpCurrentDateTimeStr);
        } else {
          snprintf(mailPayload, sizeof(mailPayload), "{ \"nodename\": \"%s\",  \"subject\": \"Valve argon gas bottle not closed? Valve argon gas bottle not closed, but no RFID card seen for at least %d seconds\", \"DateTime\": \"%s\" }", MACHINE, MAX_WAIT_FOR_CARD_TIME, ntpCurrentDateTimeStr);
        }
        node.send("bestuur", mailPayload, SEND_MAIL_MESSAGES_RAW);
        if (panicButtonWasPressed) {
          Serial.printf("%s Panic button was pressed while valve is open but no RFID card was seen\n\r", ntpCurrentDateTimeStr);
        } else {
          Serial.printf("%s Valve argon gas bottle not closed, but no RFID card seen in at least %d seconds\n\r", ntpCurrentDateTimeStr, MAX_WAIT_FOR_CARD_TIME);
        }
        panicButtonWasPressed = false;
        machinestate = WARNINGACTIVENOTAPPROVED;
        valveIsOpenError = true;
        errorBestuurIsActive = true;
        machinestate = ALARMACTIVENOTAPPROVED;
      } else {
        machinestate = NOTACTIVE;
      }
      break;
    case ALARMACTIVENOTAPPROVED:
      break;
    case APPROVED:
      break;
    case ACTIVEAPPROVED:
      break;
    case SENDWARNINGACTIVEAPPROVED:
        if (valveIsOpen) {
          setApprovedWarning();
          // ****** sent error message, valve still open after max time
          getCurrentNTPDateTime();
          snprintf(mailPayload, sizeof(mailPayload), "{ \"nodename\": \"%s\",  \"subject\": \"Valve argon gas still open? Gas pressure detected for at least %d hours\", \"DateTime\": \"%s\" }", MACHINE, MAX_TIME_BEFORE_WARNING / 3600, ntpCurrentDateTimeStr);
          node.send("deelnemers", mailPayload, SEND_MAIL_MESSAGES_RAW);
          Serial.printf("%s Valve argon gas still open? Gas pressure detected for at least %d hours\n\r", ntpCurrentDateTimeStr, MAX_TIME_BEFORE_WARNING / 3600);
          valveIsOpenError = true;
          errorDeelnemersIsActive = true;
          machinestate = WARNINGACTIVEAPPROVED;
        } else {
          machinestate = NOTACTIVE;
        }
      break;
    case WARNINGACTIVEAPPROVED:
      break;
    case SENDALARMACTIVEAPPROVED:
          // ****** sent error message, valve still open after max time
        if (valveIsOpen) {
          setApprovedAlarm();
          // ****** sent error message, valve still open after max time
          getCurrentNTPDateTime();
          if (panicButtonWasPressed) {
            snprintf(mailPayload, sizeof(mailPayload), "{ \"nodename\": \"%s\",  \"subject\": \"Panic button was pressed? Panic button was pressed while valve is open\", \"DateTime\": \"%s\" }", MACHINE, ntpCurrentDateTimeStr);
          } else {
            snprintf(mailPayload, sizeof(mailPayload), "{ \"nodename\": \"%s\",  \"subject\": \"Valve argon gas alarm? Gas pressure detected for at least %d hours after warning\", \"DateTime\": \"%s\" }", MACHINE, MAX_TIME_BEFORE_ALARM / 3600, ntpCurrentDateTimeStr);
          }
          node.send("deelnemers", mailPayload, SEND_MAIL_MESSAGES_RAW);
          node.send("bestuur", mailPayload, SEND_MAIL_MESSAGES_RAW);
          if (panicButtonWasPressed) {
            Serial.printf("%s Panic button was pressed while valve is open\n\r", ntpCurrentDateTimeStr);
          } else {
            Serial.printf("%s Valve argon gas alarm? Gas pressure detected for at least %d hours after warning\n\r", ntpCurrentDateTimeStr, MAX_TIME_BEFORE_ALARM / 3600);
          }
          panicButtonWasPressed = false;
          valveIsOpenError = true;
          errorDeelnemersIsActive = true;
          errorBestuurIsActive = true;
          machinestate = ALARMACTIVEAPPROVED;
        } else {
          machinestate = NOTACTIVE;
        }
      break; 
    case ALARMACTIVEAPPROVED:
      break;      
    case BOOTING:
    case OUTOFORDER:
    case TRANSIENTERROR:
    case NOCONN:
      break;
  };
}





