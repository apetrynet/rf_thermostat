// Hugely inspired by: https://github.com/GeorgeDewar/pid-thermostat/blob/master/thermostat/thermostat.ino
#include <Wire.h>
#include <PID_v1.h>
#include "tx433_Nexa.h"
#include <Time.h>
#include <EEPROM.h>
#include <Button.h>   // https://github.com/JChristensen/Button
#include <LiquidCrystal_I2C.h>
#include <RtcDS3231.h>  // https://github.com/Makuna/Rtc

#define ROT_CW_PIN 2
#define ROT_CCW_PIN 3
#define ROT_SW_PIN 4
#define TX_PIN 5

// RTC setup
RtcDS3231<TwoWire> Rtc(Wire);

// Create a menu button instance
Button menuButton(ROT_SW_PIN, true, true, 25);

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Kp = 45;
double Ki = 0.05;
double Kd = 0;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long windowSize = 600000; // 10 Minutes
unsigned long windowStartTime;

float tempC;
unsigned long lastTempRead = 0;
unsigned long lastAlarmCheck = 0;

bool heaterState = false;

// Updated by the ISR (Interrupt Service Routine)
volatile uint8_t virtualInt;
volatile float virtualFloat;

// Alarms for when we want heating
uint8_t alarms[4][2];
/*uint8_t alarms[4][2] = {
  {23, 30}, {5, 30},  //night
  {8, 30}, {15, 0}  //work
};*/

// Run modes
enum modes {
  normal,
  daytime,
  night,
  allwayson,
  allwaysoff,
  tweakmode
};

modes mode = normal;

enum menuroots {
  SpNml,
  AlwOn,
  AlwOf,
  AlNOn,
  AlNOf,
  AlDOn,
  AlDOf,
  SetTm
};

menuroots menuroot = SpNml;
volatile uint8_t menurootPos = 0;
volatile bool rootLevel = true;

// Iterate over these in checkButton() or other function check hydroponics.ino
// Use index to compare against enum values as guide for cursor positions
uint8_t setpointPos = 0;
uint8_t spMenuOrder[5] = {0, 1, NULL, NULL, NULL};
uint8_t alarmDayPos = 0;
uint8_t alarmDayMenuOrder[5] = {0, 2, 3, 1, NULL};
uint8_t alarmNightPos = 0;
uint8_t alarmNightMenuOrder[5] = {0, 2, 3, NULL, NULL};
uint8_t setTimePos = 0;
uint8_t setTimeMenuOrder[5] = {0, 2, 3, NULL, NULL};

// LCD coordinates
const uint8_t coordinates[4][2] = {
  {11, 1},  // M:<
  {3, 1},   // Sp:<
  {0, 0},   // >HH
  {3, 0}    // >MM
};

// Settings
struct Settings {
  uint8_t magic;
  float setpointHigh;
  float setpointLow;
  uint8_t alarmNightIn[2];
  uint8_t alarmNightOut[2];
  uint8_t alarmDayIn[2];
  uint8_t alarmDayOut[2];
};

bool settingsLoaded = false;
Settings settings;

// Nexa ID
String tx_nexa = "1010101010101010101010101010101010101010101010101010";
String ch_nexa = "1010";

// RF transmit Pin 3
Tx433_Nexa Nexa(TX_PIN, tx_nexa, ch_nexa);

void setup() {
  pinMode(ROT_CW_PIN, INPUT);
  pinMode(ROT_CCW_PIN, INPUT);
  pinMode(ROT_SW_PIN, INPUT_PULLUP);
  pinMode(TX_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ROT_CW_PIN), rotationInterrupt, LOW);
  Serial.begin(115200);

  // initialize the LCD
	lcd.init();
  lcd.noCursor();
  lcd.noBlink();
  lcd.backlight();

  // RTC setup
  Rtc.Begin();
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);
  //setSyncProvider(RTC.get);

  // Make sure relay is OFF
  Nexa.Device_Off(0);

  //tell the PID to range between 0 and 100 (%)
  myPID.SetOutputLimits(0, 100);
  myPID.SetSampleTime(1000);

  //turn the PID off
  myPID.SetMode(MANUAL);

}

void loop() {
  if (!settingsLoaded) {
    loadSettings();
    if (!settingsLoaded) {
      setDefaultSettings();
      saveSettings();
    }
  }

  if (settingsLoaded && myPID.GetMode() == MANUAL){
    windowStartTime = millis();
    myPID.SetMode(AUTOMATIC);
  }

  checkButton();

  readTemp();
  myPID.Compute();

  checkAlarms();
  if (mode == daytime){
    Setpoint = settings.setpointLow;

  } else {
    Setpoint = settings.setpointHigh;
  }

  adjustHeater();
  refreshLCD();
}

void rotationInterrupt ()  {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (interruptTime - lastInterruptTime > 5) {
    if (mode != tweakmode)
      return;

    if (rootLevel) {
      if (menurootPos > 7) {
        menuroot = SpNml;
        menurootPos = 0;

      } else {
        menuroot = (menuroots) menurootPos;
        if (digitalRead(ROT_CCW_PIN) == LOW) {
          menurootPos-- ; // Could be -5 or -10
        } else {
          menurootPos++;
        }
        menurootPos = min(7, max(0, menurootPos));
        Serial.print("menu root pos: ");
        Serial.println(menurootPos);
      }
    } else {
      switch (menuroot) {
        case SpNml:
          virtualFloat = settings.setpointHigh;
          if (digitalRead(ROT_CCW_PIN) == LOW) {
            virtualFloat -= 0.5 ; // Could be -5 or -10
          } else {
            virtualFloat += 0.5;
          }
          settings.setpointHigh = min(30.0, max(15.0, virtualFloat));
          break;

        case AlDOn:
          switch (alarmDayPos){
            case 1:
              virtualInt = settings.alarmDayIn[0];
              break;
            case 2:
              virtualInt = settings.alarmDayIn[1];
              break;
            case 3:
              virtualFloat = settings.setpointLow;
              break;
          }
          if (digitalRead(ROT_CCW_PIN) == LOW) {
            virtualInt--;
            virtualFloat -= 0.5;
          } else {
            virtualInt++;
            virtualFloat += 0.5;
          }
          switch (alarmDayPos){
            case 1:
              settings.alarmDayIn[0] = min(23, max(0, virtualInt));
              alarms[2][0] = settings.alarmDayIn[0];
              break;
            case 2:
              settings.alarmDayIn[1] = min(59, max(0, virtualInt));
              alarms[2][1] = settings.alarmDayIn[1];
              break;
            case 3:
              settings.setpointLow = min(30.0, max(15.0, virtualFloat));
              break;
          }
          break;

          case AlDOf:
            switch (alarmDayPos){
              case 1:
                virtualInt = settings.alarmDayOut[0];
                break;
              case 2:
                virtualInt = settings.alarmDayOut[1];
                break;
              case 3:
                virtualFloat = settings.setpointLow;
                break;
            }
            if (digitalRead(ROT_CCW_PIN) == LOW) {
              virtualInt--;
              virtualFloat -= 0.5;
            } else {
              virtualInt++;
              virtualFloat += 0.5;
            }
            switch (alarmDayPos){
              case 1:
                settings.alarmDayOut[0] = min(23, max(0, virtualInt));
                alarms[3][0] = settings.alarmDayOut[0];
                break;
              case 2:
                settings.alarmDayOut[1] = min(59, max(0, virtualInt));
                alarms[3][1] = settings.alarmDayOut[1];
                break;
              case 3:
                settings.setpointLow = min(30.0, max(15.0, virtualFloat));
                break;
            }
            break;

        case AlNOn:
          switch (alarmNightPos){
            case 1:
              virtualInt = settings.alarmNightIn[0];
              break;
            case 2:
              virtualInt = settings.alarmNightIn[1];
              break;
          }
          if (digitalRead(ROT_CCW_PIN) == LOW) {
            virtualInt--;
          } else {
            virtualInt++;
          }
          switch (alarmNightPos){
            case 1:
              settings.alarmNightIn[0] = min(23, max(0, virtualInt));
              alarms[0][0] = settings.alarmNightIn[0];
              break;
            case 2:
              settings.alarmNightIn[1] = min(59, max(0, virtualInt));
              alarms[0][1] = settings.alarmNightIn[1];
              break;
          }
          break;

        case AlNOf:
          switch (alarmNightPos){
            case 1:
              virtualInt = settings.alarmNightOut[0];
              break;
            case 2:
              virtualInt = settings.alarmNightOut[1];
              break;
          }
          if (digitalRead(ROT_CCW_PIN) == LOW) {
            virtualInt--;
          } else {
            virtualInt++;
          }
          switch (alarmNightPos){
            case 1:
              settings.alarmNightOut[0] = min(23, max(0, virtualInt));
              alarms[1][0] = settings.alarmNightOut[0];
              break;
            case 2:
              settings.alarmNightOut[1] = min(59, max(0, virtualInt));
              alarms[1][1] = settings.alarmNightOut[1];
              break;
          }
          break;

        case SetTm:
          uint8_t h;
          uint8_t m;
          RtcDateTime timeNow = Rtc.GetDateTime();
          switch (setTimePos){
            case 1:
              virtualInt = timeNow.Hour();
              break;
            case 2:
              virtualInt = timeNow.Minute();
              break;
          }
          if (digitalRead(ROT_CCW_PIN) == LOW) {
            virtualInt--;
          } else {
            virtualInt++;
          }
          switch (setTimePos){
            case 1:
              h = min(23, max(0, virtualInt));
              break;
            case 2:
              m = min(59, max(0, virtualInt));
              break;
          }
          RtcDateTime newTime = RtcDateTime(
                                        timeNow.Year(),
                                        timeNow.Month(),
                                        timeNow.Day(),
                                        h,
                                        m,
                                        0
                                      );
          Rtc.SetDateTime(newTime);
          break;
      }
    }
    // Keep track of when we were here last (no more than every 5ms)
    lastInterruptTime = interruptTime;
  }
}

void placeCursor(uint8_t (& order)[5], uint8_t & pos) {
  uint8_t coord_index = order[pos];

  if (coord_index == NULL){
    pos = 0;
    coord_index = order[pos];
  }

  if (pos == 0) {
    rootLevel = true;
  } else {
    rootLevel = false;
  }

  uint8_t x = coordinates[coord_index][0];
  uint8_t y = coordinates[coord_index][1];

  lcd.setCursor(x, y);
}

void checkButton() {
  // Long press while not in tweakmode enters tweakmode
  // Long press while in tweakmode saves settings.
  static long lastLongPress = 0;
  static uint8_t prevMode;

  menuButton.read();

  if (mode == tweakmode && menuButton.wasReleased() && millis() - lastLongPress > 500) {

    // Iterater over menu roots/items
    Serial.println("Button On");
    if (mode == tweakmode) {
      // iterate over menu items
      Serial.print("rootlevel: ");
      Serial.println(rootLevel ? "true" : "false");

      switch (menuroot) {
        case SpNml:
          setpointPos++;
          Serial.print("setpointPos: ");
          Serial.println(setpointPos);
          break;

        case AlNOn:
        case AlNOf:
          alarmNightPos++;
          Serial.print("alarmNightPos: ");
          Serial.println(alarmNightPos);
          break;

        case AlDOn:
        case AlDOf:
          alarmDayPos++;
          Serial.print("alarmDayPos: ");
          Serial.println(alarmDayPos);
          break;

        case SetTm:
          setTimePos++;
          Serial.print("setTimePos: ");
          Serial.println(setTimePos);
          break;

        case AlwOn:
          if (prevMode == allwayson){
            mode = normal;
          } else {
            mode = allwayson;
          }
          break;

        case AlwOf:
          if (prevMode == allwaysoff){
            mode = normal;
          } else {
            mode = allwaysoff;
          }
          break;

        default:
          break;
      }
    }
  } else if (menuButton.pressedFor(1000) && millis() - lastLongPress > 1000) {

    // Initiate or exit tweak mode
    Serial.println("Button Hold");
    if (mode == tweakmode){
      mode = prevMode;
      //rootLevel = true;
      setpointPos = 0;
      alarmNightPos = 0;
      alarmDayPos = 0;
      setTimePos = 0;
      lcd.noCursor();
      lcd.noBlink();
      saveSettings();

    } else {
      prevMode = mode;
      mode = tweakmode;
      //rootLevel = true;
      lcd.setCursor(11, 1);  //M:< position of LCD
      lcd.cursor();
      lcd.blink();
    }
    Serial.print("mode: ");
    Serial.println(mode == tweakmode ? "tweakmode" : "normal");
    lastLongPress = millis();
  }
}

void setDefaultSettings() {
  settings.magic = 42;
  settings.setpointHigh = 24.5;
  settings.setpointLow = 18;
  settings.alarmNightIn[0] = 23;
  settings.alarmNightIn[1] = 30;
  settings.alarmNightOut[0] = 5;
  settings.alarmNightOut[1] = 30;
  settings.alarmDayIn[0] = 8;
  settings.alarmDayIn[1] = 30;
  settings.alarmDayOut[0] = 15;
  settings.alarmDayOut[1] = 30;

  settingsLoaded = true;
  Serial.println("Default settings applied");
}

void saveSettings() {
  Settings oldSettings = EEPROM.get(0, oldSettings);
  bool update = false;

  if (oldSettings.setpointHigh != settings.setpointHigh)
    update = true;
  else if (oldSettings.setpointLow != settings.setpointLow)
    update = true;
  else if (oldSettings.alarmNightIn[0] != settings.alarmNightIn[0])
    update = true;
  else if (oldSettings.alarmNightIn[1] != settings.alarmNightIn[1])
    update = true;
  else if (oldSettings.alarmNightOut[0] != settings.alarmNightOut[0])
    update = true;
  else if (oldSettings.alarmNightOut[1] != settings.alarmNightOut[1])
    update = true;
  else if (oldSettings.alarmDayIn[0] != settings.alarmDayIn[0])
    update = true;
  else if (oldSettings.alarmDayIn[1] != settings.alarmDayIn[1])
    update = true;
  else if (oldSettings.alarmDayOut[0] != settings.alarmDayOut[0])
    update = true;
  else if (oldSettings.alarmDayOut[1] != settings.alarmDayOut[1])
    update = true;

  if (update) {
    EEPROM.put(0, settings);
    Serial.println("Settings saved");
  } else {
    Serial.println("No changes");

  }
}

void loadSettings() {
  EEPROM.get(0, settings);

  // Check if we have settings stored
  if (settings.magic != 42) {
    settingsLoaded = false;

  } else {
    alarms[0][0] = settings.alarmNightIn[0];
    alarms[0][1] = settings.alarmNightIn[1];
    alarms[1][0] = settings.alarmNightOut[0];
    alarms[1][1] = settings.alarmNightOut[1];
    alarms[2][0] = settings.alarmDayIn[0];
    alarms[2][1] = settings.alarmDayIn[1];
    alarms[3][0] = settings.alarmDayOut[0];
    alarms[3][1] = settings.alarmDayOut[1];

    settingsLoaded = true;
    Serial.println("Settings loaded");
  }
}

void adjustHeater() {
  unsigned long now = millis();

  if (now - windowStartTime > windowSize) {
    //time to shift the Relay Window
    windowStartTime += windowSize;
  }

  if (Output * windowSize > ((now - windowStartTime) * 100)) {
    // Make sure heater is ON
    switchHeater(true);

  } else {
    // Turn OFF heater
    switchHeater(false);
  }
}

void switchHeater(bool state) {
  // Override state depending on modes
  switch (mode) {
    case allwaysoff: state = false;
      break;
    case night: state = false;
      break;
    default:
      break;
  }

  if (state && heaterState) {
    return;

  } else if (!state && !heaterState) {
    return;

  } else if (!state && heaterState) {
    Serial.println("Heater OFF");
    Nexa.Device_Off(0);
    heaterState = false;

  } else if (state && !heaterState) {
    Serial.println("Heater ON");
    Nexa.Device_On(0);
    heaterState = true;
  }
}

uint16_t timeToMin(uint8_t (& t)[2]) {
  uint16_t hours_as_minutes = t[0] * 60;

  return uint16_t(t[1] + hours_as_minutes);
}

// Check if time is within an alarm
bool checkTimeRange(uint8_t (& in)[2], uint8_t (& out)[2]) {
  RtcDateTime timeNow = Rtc.GetDateTime();
  uint16_t on = timeToMin(in);
  uint16_t off = timeToMin(out);
  uint8_t n[2] = {timeNow.Hour(), timeNow.Minute()};
  uint16_t now = timeToMin(n);

  if (on < off)
    if (now >= on  && now < off)
      return true;
    else
      return false;

  if (on > off)
    if (off == 0)
      if (now >= on)
        return true;
      else
        return false;
    else
      if (now >= on || now < off)
        return true;
      else
        return false;
}

void checkAlarms() {
  unsigned long now = millis();

  if (lastAlarmCheck > 0 && now - lastAlarmCheck < 1000) {
    return;
  }
  if (mode == allwayson || mode == allwaysoff || mode == tweakmode)
    return;

  // Night?
  if (checkTimeRange(alarms[0], alarms[1])) {
    mode = night;
  // Daytime ?
  } else if (checkTimeRange(alarms[2], alarms[3])) {
    mode = daytime;
  } else {
    mode = normal;
  }
}

void readTemp() {
  unsigned long now = millis();

  if (lastTempRead == 0 || now - lastTempRead >= 1000) {
    RtcTemperature temp = Rtc.GetTemperature();
    tempC = temp.AsFloatDegC();

    // Update PID Input
    Input = tempC;

    // Update timer
    lastTempRead = now;
  }
}

void refreshLCD() {
  static unsigned long lastStatus = 0;
  static uint16_t statusTimeout = 1000;

  unsigned long now = millis();

  if (mode == tweakmode) {
    //Set lower timeout to display menu/tweak changes quicker
    statusTimeout = 50;

  } else {
    statusTimeout = 1000;
  }

  if (lastStatus == 0 || now - lastStatus >= statusTimeout) {
    lcd.setCursor(0, 0);
    printTime();

    lcd.print(" T:");
    lcd.print(tempC);
    lcd.setCursor(0, 1);

    //Setpoint
    lcd.print("Sp:");
    if (mode == tweakmode) {
      switch (menuroot) {
        case SpNml: lcd.print(settings.setpointHigh);
          break;
        case AlDOn: lcd.print(settings.setpointLow);
          break;
        case AlDOf: lcd.print(settings.setpointLow);
          break;
        default: lcd.print(Setpoint);
      }
    } else {
      lcd.print(Setpoint);
    }

    // Mode
    lcd.print(" M:");
    switch (mode) {
      case allwayson: lcd.print("AllOn");
        break;
      case allwaysoff: lcd.print("AlOff");
        break;
      case night: lcd.print("Night");
        break;
      case daytime: lcd.print("Day  ");
        break;
      case normal: lcd.print("Norml");
        break;
      case tweakmode:
        lcd.cursor();
        lcd.blink();
        switch (menuroot) {
          case SpNml:
            lcd.print("SpNml");
            placeCursor(spMenuOrder, setpointPos);
            break;
          case AlwOn: lcd.print("AlwOn");
            break;
          case AlwOf: lcd.print("AlwOf");
            break;
          case AlNOn:
            lcd.print("AlNOn");
            placeCursor(alarmNightMenuOrder, alarmNightPos);
            break;
          case AlNOf:
            lcd.print("AlNOf");
            placeCursor(alarmNightMenuOrder, alarmNightPos);
            break;
          case AlDOn:
            lcd.print("AlDOn");
            placeCursor(alarmDayMenuOrder, alarmDayPos);
            break;
          case AlDOf:
            lcd.print("AlDOf");
            placeCursor(alarmDayMenuOrder, alarmDayPos);
            break;
          case SetTm:
            lcd.print("SetTm");
            placeCursor(setTimeMenuOrder, setTimePos);
            break;
        }
    }
    lastStatus = now;
  }
}

void printTime() {
  if (mode == tweakmode) {
    switch (menuroot){
      case AlNOn:
        printDigits(settings.alarmNightIn[0]);
        lcd.print(":");
        printDigits(settings.alarmNightIn[1]);
        lcd.print(":");
        printDigits(0);
        return;

      case AlNOf:
        printDigits(settings.alarmNightOut[0]);
        lcd.print(":");
        printDigits(settings.alarmNightOut[1]);
        lcd.print(":");
        printDigits(0);
        return;

      case AlDOn:
        printDigits(settings.alarmDayIn[0]);
        lcd.print(":");
        printDigits(settings.alarmDayIn[1]);
        lcd.print(":");
        printDigits(0);
        return;

      case AlDOf:
        printDigits(settings.alarmDayOut[0]);
        lcd.print(":");
        printDigits(settings.alarmDayOut[1]);
        lcd.print(":");
        printDigits(0);
        return;
    }
  }
  // digital clock display of the time
  RtcDateTime timeNow = Rtc.GetDateTime();
  printDigits(timeNow.Hour());
  lcd.print(":");
  printDigits(timeNow.Minute());
  lcd.print(":");
  printDigits(timeNow.Second());
}

void printDigits(int digits)
{
  if(digits < 10)
    lcd.print('0');
  lcd.print(digits);
}
