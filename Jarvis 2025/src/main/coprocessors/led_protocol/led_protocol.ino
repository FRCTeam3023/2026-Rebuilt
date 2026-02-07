#include<Adafruit_NeoPixel.h>

#define LED_COUNT 32
#define LED_PIN 2
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
byte LED_COLOR_RX[7];

//code for limit switches, basically just sets up pins
const int buttonPinLimUp = 3;    // Elevator up
const int buttonPinLimDwn = 4;   // Elevator down
const int buttonPinLimIn = 5;    // Climber in
const int buttonPinIntake = 6;   // Coral intake
int buttonStateLimUp = 0;  
int buttonStateLimDwn = 0;    // variable for reading the pushbutton status
int buttonStateLimIn = 0;     // variable for reading the pushbutton status
int buttonStateIntake = 0;    // variable for reading the pushbutton status

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  LED_COLOR_RX[0] = 0x00;
  LED_COLOR_RX[1] = 0x00;
  LED_COLOR_RX[2] = LED_COUNT;
  LED_COLOR_RX[3] = LED_COUNT >> 8;
  LED_COLOR_RX[4] = 0x00;
  LED_COLOR_RX[5] = 0x00;
  LED_COLOR_RX[6] = 0x00;

  strip.begin();
//sets up pins as inputs or ouputs
  pinMode(buttonPinLimUp, INPUT_PULLUP);
  pinMode(buttonPinLimDwn, INPUT_PULLUP);
  pinMode(buttonPinLimIn, INPUT_PULLUP);
  pinMode(buttonPinIntake, INPUT_PULLUP);

}

/*
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
*/

void loop() {
  if (Serial.available()) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.readBytes(LED_COLOR_RX, 7);
    Serial.flush();
    receiveSignal();
    digitalWrite(LED_BUILTIN, LOW);
  }
switchStatusUpdate();
}

void setLedColor(uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
  strip.setPixelColor(i, g, r, b);
}

void setLedColors(uint16_t start, uint16_t end, uint8_t r, uint8_t g, uint8_t b) {
  if (start < end) {
    for (uint16_t i = start; i <= end; i++)
      setLedColor(i, r, g, b);
    strip.show();
  }
}

void receiveSignal() {
  uint16_t start = (uint16_t)LED_COLOR_RX[1] << 8 | (uint16_t)LED_COLOR_RX[0];
  uint16_t end = (uint16_t)LED_COLOR_RX[3] << 8 | (uint16_t)LED_COLOR_RX[2];
  setLedColors(start, end, LED_COLOR_RX[4], LED_COLOR_RX[5], LED_COLOR_RX[6]);
}

void switchStatusUpdate() {
  // limit switch code
 setLedColors(0, 10, 0xFF, 0xFF, 0xFF);
 setLedColors(15, 32, 0xFF, 0xFF, 0xFF);
 buttonStateLimUp = digitalRead(buttonPinLimUp);
  if (buttonStateLimUp == HIGH) {
    setLedColor(11, 0x00, 0xFF, 0x00);
  } else {
    setLedColor(11, 0xFF, 0x00, 0x00);
  }
 buttonStateLimDwn = digitalRead(buttonPinLimDwn);
  if (buttonStateLimDwn == HIGH) {
    setLedColor(12, 0xFF, 0x00, 0x00);
  } else {
    setLedColor(12, 0x00, 0xFF, 0x00);
  }
buttonStateLimIn = digitalRead(buttonPinLimIn);
  if (buttonStateLimIn == HIGH) {
    setLedColor(13, 0xFF, 0x00, 0x00);
  } else {
    setLedColor(13, 0x00, 0xFF, 0x00);
  }
buttonStateIntake = digitalRead(buttonPinIntake);
  if (buttonStateIntake == HIGH) {
    setLedColor(14, 0x00, 0xFF, 0x00);
  } else {
    setLedColor(14, 0xFF, 0x00, 0x00);
  }
}

/*
void switchStatusUpdate() {
  //limit switch code
 buttonStateLimUp = digitalRead(buttonPinLimUp);
  // put your main code here, to run repeatedly:
  if (buttonStateLimUp == HIGH) {
    digitalWrite(4, HIGH);
  } else {
    digitalWrite(4, LOW);
  }
 buttonStateLimDwn = digitalRead(buttonPinLimDwn);
  // put your main code here, to run repeatedly:
  if (buttonStateLimDwn == HIGH) {
    digitalWrite(5, HIGH);
  } else {
    digitalWrite(5, LOW);
  }
buttonStateLimIn = digitalRead(buttonPinLimIn);
  // put your main code here, to run repeatedly:
  if (buttonStateLimIn == HIGH) {
    digitalWrite(6, HIGH);
  } else {
    digitalWrite(6, LOW);
  }
buttonStateLimOut = digitalRead(buttonPinLimOut);
  // put your main code here, to run repeatedly:
  if (buttonStateLimOut == HIGH) {
    digitalWrite(7, HIGH);
  } else {
    digitalWrite(7, LOW);
  }
}
*/
