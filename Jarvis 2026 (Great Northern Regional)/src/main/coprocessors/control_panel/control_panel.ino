#include <Joystick.h>

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   16, 0,                 // Button Count, Hat Switch Count
                   false, false, false,   // No X, Y, or Z axes
                   false, false, false,   // No Rx, Ry, or Rz
                   false, false,          // No rudder or throttle
                   false, false, false);  // No accelerator, brake, or steering

#include <HID_Buttons.h>  // Must import AFTER Joystick.h

//index=joystick, value=physical pinout; first 4 values are height, remaining 12 are reef location
//todo fill in according to actual button layout; currently populated on setup
uint8_t inputTable[] = { 13, 25, 24, 23, 10, 11, 12, 2, 3, 22, 5, 26, 6, 7, 9, 8 };
uint8_t outputTable[] = { 45, 34, 36, 38, 33, 31, 35, 30, 32, 44, 42, 40, 43, 41, 39, 37 };

//These are array indices and not actual locations
uint8_t reefLocation = 4;
uint8_t previousReefLocation = 4;
uint8_t heightLocation = 0;
uint8_t previousHeightLocation = 0;

void setup() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  for (int i = 5; i <= 13; i++) {
    pinMode(i, INPUT);
  }
  for (int i = 22; i <= 26; i++) {
    pinMode(i, INPUT);
  }
  for (int i = 0; i < 16; i++) {
    pinMode(outputTable[i], OUTPUT);
  }

  Serial.begin(9600);
  Joystick.begin();
  inputChanged();
}

void inputChanged() {
  Joystick.setButton(previousReefLocation, 0);
  Joystick.setButton(previousHeightLocation, 0);
  digitalWrite(outputTable[previousReefLocation], LOW);
  digitalWrite(outputTable[previousHeightLocation], LOW);

  Joystick.setButton(reefLocation, 1);
  Joystick.setButton(heightLocation, 1);
  digitalWrite(outputTable[reefLocation], HIGH);
  digitalWrite(outputTable[heightLocation], HIGH);
}

uint8_t outputI = 0;

void testPinout() {
  Serial.print("Input ");
  for (int i = 0; i < 16; i++) {
    if (digitalRead(inputTable[i])) {
      Serial.print(inputTable[i]);
      Serial.print(", ");
    }
  }

  Serial.print("Output ");
  Serial.print(outputI);
  Serial.print(":");
  Serial.println(outputTable[outputI]);
  if (Serial.available()) {
    digitalWrite(outputTable[outputI], LOW);
    outputI++;
    if (outputI > 15) outputI = 0;
    digitalWrite(outputTable[outputI], HIGH);
    while (Serial.available()) Serial.read();
  }
}

void loop() {
  //testPinout();
  for (int i = 0; i < 16; i++) {
    if (digitalRead(inputTable[i])) {
      if (i < 4) heightLocation = i;
      else reefLocation = i;
    }
  }

  if (previousReefLocation != reefLocation || previousHeightLocation != heightLocation) {
    inputChanged();
  }

  previousHeightLocation = heightLocation;
  previousReefLocation = reefLocation;
}
