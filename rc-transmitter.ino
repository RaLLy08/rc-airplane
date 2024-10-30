
// Code 1 : Sending Text (Transmitter)
// Library: TMRh20/RF24 (https://github.com/tmrh20/RF24/)

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

RF24 radio(9, 8); // CE, CSN
const byte address[10] = "ADDRESS01";

// test
#include<Servo.h>

Servo esc;
//


// joystick
#define analogPinX A0  // Analog pin for X axis
#define analogPinY A1  // Analog pin for Y axis
// 

#define potentiometer1 A2
#define ledIndicator1 2

int s1State;
int s1LastState;


void setup() {
  esc.attach(A5, 1000, 2000);

  pinMode(potentiometer1, INPUT);

  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
};


int8_t dataToSend[3] = {0, 0, 0}; // x, y, speed, key


void loop() {
  int8_t xValue = map(analogRead(analogPinX), 0, 1023, -128, 127);
  int8_t yValue = map(analogRead(analogPinY), 0, 1023, -128, 127);
  int8_t potentiometer = map(analogRead(potentiometer1), 0, 1023, 0, 100);

  dataToSend[0] = xValue;
  dataToSend[1] = yValue;
  dataToSend[2] = potentiometer;

  // logs
  int isNotSent = radio.available();
  if (isNotSent) {
    digitalWrite(ledIndicator1, LOW);
  } else {
    digitalWrite(ledIndicator1, HIGH);
  }

  radio.write(&dataToSend, sizeof(dataToSend));

  // 

  digitalWrite(ledIndicator1, LOW);

  delay(20);
};
