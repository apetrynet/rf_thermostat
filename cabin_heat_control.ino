// Hugely inspired by: https://github.com/GeorgeDewar/pid-thermostat/blob/master/thermostat/thermostat.ino
#include <Wire.h>
#include <PID_v1.h>
#include "tx433_Nexa.h"
#include <Time.h>
#include <DS3232RTC.h>
#include <EEPROM.h>
#include <Button.h>   // https://github.com/JChristensen/Button
#include <LiquidCrystal_I2C.h>

#define ROT_CW_PIN 2
#define ROT_CCW_PIN 3
#define ROT_SW_PIN 4
#define TX_PIN 5

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
static unsigned long lastTempRead = 0;
static unsigned long lastAlarmCheck = 0;

bool heaterState = false;

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
  SpLow,
  AlwOn,
  AlwOf,
  AlNOn,
  AlNOf,
  AlDOn,
  AlDOf,
  SetTm,
  LAST
};

menuroots menuroot = SpNml;
uint8_t menurootPos = 0;

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

  // Menu button settings
  //menuButton.assign(ROT_SW_PIN);
  //menuButton.setMode(OneShotTimer);
  //menuButton.setTimer(1000);
  //menuButton.turnOnPullUp();

  Serial.begin(115200);

  // initialize the LCD
	lcd.init();
  lcd.home();
  lcd.noCursor();
  lcd.noBlink();
  lcd.backlight();

  // RTC setup
  setSyncProvider(RTC.get);

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

  //TODO Add a checkButton function that cycles throug menu setings
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

void placeCursor(uint8_t (& order)[5], uint8_t & pos) {
  uint8_t coord_index = order[pos];

  if (coord_index == NULL){
    pos = 0;
    coord_index = order[pos];

  } else {
    pos++;
  }

  uint8_t x = coordinates[coord_index][0];
  uint8_t y = coordinates[coord_index][1];

  lcd.setCursor(x, y);
}

void checkButton() {
  // Long press while not in tweakmode enters tweakmode
  // Long press while in tweakmode saves settings.
  menuButton.read();

  if  (menuButton.pressedFor(1000)) {
    static uint8_t prevMode;

    // Initiate or exit tweak mode
    Serial.println("Button Hold");
    if (mode == tweakmode){
      mode = prevMode;
      lcd.noCursor();
      lcd.noBlink();
      saveSettings();

    } else {
      prevMode = mode;
      mode = tweakmode;
      lcd.setCursor(11, 1);  //M:< position of LCD
      lcd.cursor();
      lcd.blink();
    }

  } else if (menuButton.wasReleased()) {
    // Iterater over menu roots/items
    Serial.println("Button On");
    if (mode == tweakmode) {
      // iterate over menu items
      switch (menuroot) {
        case SpNml:
        case SpLow:
          placeCursor(spMenuOrder, setpointPos);
          break;

        case AlNOn:
        case AlNOf:
          placeCursor(alarmNightMenuOrder, alarmNightPos);
          break;

        case AlDOn:
        case AlDOf:
          placeCursor(alarmDayMenuOrder, alarmDayPos);
          break;

        case SetTm:
          placeCursor(setTimeMenuOrder, setTimePos);
          break;

        default:
          break;
      }
    }
  }
}

/* Use this in rotation logic
if (menuroot == LAST) {
  menuroot = SpNml;
  menurootPos = 0;

} else {
  menuroot = (menuroots) menurootPos;
  menurootPos++;
}
*/

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
  EEPROM.put(0, settings);
  Serial.println("Settings saved");
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
    case allwayson: state = true;
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
  uint16_t on = timeToMin(in);
  uint16_t off = timeToMin(out);
  uint8_t n[2] = {hour(), minute()};
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
    int temp = RTC.temperature();
    tempC = temp / 4.0;

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
    lcd.home();
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
        switch (menuroot) {
          case SpNml: lcd.print("SpNml");
            break;
          case SpLow: lcd.print("SpLow");
            break;
          case AlwOn: lcd.print("AlwOn");
            break;
          case AlwOf: lcd.print("AlwOf");
            break;
          case AlNOn: lcd.print("AlNOn");
            break;
          case AlNOf: lcd.print("AlNOf");
            break;
          case AlDOn: lcd.print("AlDOn");
            break;
          case AlDOf: lcd.print("AlDOf");
            break;
          case SetTm: lcd.print("SetTm");
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
  printDigits(hour());
  lcd.print(":");
  printDigits(minute());
  lcd.print(":");
  printDigits(second());
}

void printDigits(int digits)
{
  if(digits < 10)
    lcd.print('0');
  lcd.print(digits);
}
