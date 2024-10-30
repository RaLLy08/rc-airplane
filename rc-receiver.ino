#include <Servo.h>
// Code 1 : Sending Text (Receiver)
// Library: TMRh20/RF24 (https://github.com/tmrh20/RF24/)

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

RF24 radio(9, 8); // CE, CSN
const byte address[10] = "ADDRESS01";

#define SERVO1_PIN 3
#define SERVO2_PIN 5
#define ESC_MOTOR_PIN A0

#define MAX_SERVO_ANGLE 180

Servo servo1;
Servo servo2;

Servo esc_motor;

// setting servos to mid position!
int servo1_pos = MAX_SERVO_ANGLE / 2;
int servo2_pos = MAX_SERVO_ANGLE / 2;
int esc_motor_speed = 0; // max 100;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  esc_motor.attach(ESC_MOTOR_PIN, 1000, 2000);

  servo1.write(servo1_pos);
  servo1.write(servo2_pos);
};

unsigned long gotRequestTime = 0;
int requestDelay = 0;

const int RESPONSE_LENGTH = 3;
const int MAX_SAFE_MODE_DELAY = 1000;

int8_t saved_response[RESPONSE_LENGTH];

void loop() {
  if (radio.available()) {

    int8_t response[RESPONSE_LENGTH];

    radio.read(&response, sizeof(response));

    // write to global response
    for (int i = 0; i < RESPONSE_LENGTH; i++) {
      saved_response[i] = response[i];
    }

    gotRequestTime = millis();
  } else {
  };

  int8_t xJoystick = saved_response[0];
  int8_t yJoystick = saved_response[1];
  esc_motor_speed = saved_response[2];

  if (requestDelay > MAX_SAFE_MODE_DELAY) {
    esc_motor_speed = 0;
  }

  servo1_pos = map(xJoystick, -128, 127, 0, 180);
  servo2_pos = map(yJoystick, -128, 127, 0, 180);

  servo1.write(
    servo1_pos
  );
  servo2.write(
    servo2_pos
  );

  esc_motor.write(esc_motor_speed);

  Serial.print(servo1_pos);
  Serial.print(',');
  Serial.print(servo2_pos);
  Serial.print(',');
  Serial.print(esc_motor_speed);
  Serial.print(" delay: ");
  Serial.print(requestDelay);
  Serial.println("");

  requestDelay = millis() - gotRequestTime;
};
