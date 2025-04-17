#include <Servo.h>
// Code 1 : Sending Text (Receiver)
// Library: TMRh20/RF24 (https://github.com/tmrh20/RF24/)

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

RF24 radio(9, 8); // CE, CSN
const byte address[10] = "ADDRESS01";

struct ServoAngleBoundaries {
  const uint8_t min;
  const uint8_t initial;
  const uint8_t max;
};

class ControlServo: Servo {
  const uint8_t pin;
  const ServoAngleBoundaries angleBoundaries;
        
  uint8_t convertInt8ToAileronBoundaries(int8_t angle) {
    if (angle < -128 || angle > 127) {
      // for safety
      return angleBoundaries.initial;
    }

    if (angle < 0) {
      uint8_t lowerBounder = map(angle, -128, -1, angleBoundaries.min, angleBoundaries.initial);
      return lowerBounder;
    }

    if (angle > 0) {
      uint8_t upperBounder = map(angle, 1, 127, angleBoundaries.initial, angleBoundaries.max);

      return upperBounder;
    }

    // angle 0;
    return angleBoundaries.initial;
  }

  uint8_t convertInt8ToMotorBoundaries(int8_t speed) {
    if (speed < -128 || speed > 127) {
      // for safety
      return angleBoundaries.initial;
    }

    uint8_t boundary = map(speed, -128, 127, angleBoundaries.min, angleBoundaries.max);

    return boundary;
  }
    
  public:
    int16_t currentAngle;
    enum Type {
      MOTOR,
      AILERON
    };
    const Type type;

    ControlServo(uint8_t pin, ServoAngleBoundaries angleBoundaries, Type type = AILERON): 
      pin(pin), 
      angleBoundaries(angleBoundaries), 
      currentAngle(angleBoundaries.initial),
      type(type) {
    }

    void init() {
      if (type == MOTOR) {
        Servo::attach(pin, 1000, 2000);
      } else {
        Servo::attach(pin);
      }

      Servo::write(angleBoundaries.initial);
    }

    void setAngle(int8_t angle) {
      if (type == MOTOR) {
        currentAngle = convertInt8ToMotorBoundaries(angle);
        Servo::write(currentAngle);

        return;
      }

      currentAngle = convertInt8ToAileronBoundaries(angle);
      Servo::write(currentAngle);
    }

};
  
ControlServo leftAileron = {6, {60, 90, 145}};
ControlServo rightAileron = {10, {40, 87, 120}};
ControlServo rudder = {5, {50, 78, 125}};
ControlServo elevator = {3, {20, 70, 130}};
ControlServo escMotor = {A0, {0, 0, 180}, ControlServo::MOTOR};

// #define ESC_MOTOR_PIN A0


unsigned long gotRequestTime = 0;
unsigned long requestDelay = 0;

const int RESPONSE_LENGTH = 5;
const int MAX_SAFE_MODE_DELAY = 5000;
// x_right, y_right, x_left, y_left, rotor;
int8_t saved_response[RESPONSE_LENGTH];

void setup() {
  Serial.println("Initialization...");

  saved_response[0] = 0;
  saved_response[1] = 0;
  saved_response[2] = 0;
  saved_response[3] = 0;
  saved_response[4] = -128;//  initial value for rotor is not 0!;

  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  leftAileron.init();
  rightAileron.init();
  rudder.init();
  elevator.init();
  escMotor.init();
 
  delay(2000); 
  Serial.println("Initialization Done");
};


// TESTING
struct TestAileron {
  ControlServo& controlServo;
  const char* name;
  int16_t currentAngle = 0;
  int16_t step = 10;
  bool done = false;
  bool returnedBack = false;

  const int8_t delayMs = 0;

  // Constructor
  TestAileron(ControlServo& s, const char* n) 
    : controlServo(s), name(n) {}

  bool completed() {
    return done && returnedBack;
  }

  void run() {
    if (done) {
      if (!returnedBack) {
        currentAngle += 10;

        controlServo.setAngle(currentAngle);

        if (currentAngle >= 0) {
          returnedBack = true;
          controlServo.setAngle(0);
        }
      }

      return;
    }

    if (delayMs > 0 ) {
      delay(delayMs);
    }

    Serial.print("Send ");
    Serial.print(name);
    Serial.print(" angle: ");
    Serial.println(currentAngle);

    controlServo.setAngle(currentAngle);

    Serial.print("Actual ");
    Serial.print(name);
    Serial.print(" angle: ");
    Serial.println(controlServo.currentAngle);

    currentAngle += step;

    if (currentAngle > 127) {
      currentAngle = 127;
      step = -1 * step;
    } else if (currentAngle < -128) {
      currentAngle = -128;
      done = true;
    }
  }
};

bool testingFinished = false; // change to enable testing 

void testAirplane() {
  static TestAileron testLeftAileron = {leftAileron, "leftAileron"};
  testLeftAileron.run();

  if (!testLeftAileron.completed()) return;

  static TestAileron testRightAileron = {rightAileron, "rightAileron"};
  testRightAileron.run();

  if (!testRightAileron.completed()) return;

  static TestAileron testRudder = {rudder, "rudder"};
  testRudder.run();

  if (!testRudder.completed()) return;

  static TestAileron testElevator = {elevator, "elevator"};
  testElevator.run();

  if (!testElevator.completed()) return;

  testingFinished = true;
  Serial.println("TESTING COMPLETED");
}
// -----------

void loop() {
  // TESTING
  if (!testingFinished) {
    testAirplane();

    return;
  };
  //  ----

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

  int8_t xRightJoystick = saved_response[0];
  int8_t yRightJoystick = saved_response[1];

  int8_t xLeftJoystick = saved_response[2];
  int8_t yLeftJoystick = saved_response[3];

  int8_t motorSlider = saved_response[4];

  // esc_motor_speed = saved_response[4]; //aware 0 response and 90 set error

  if (requestDelay > MAX_SAFE_MODE_DELAY) {
    // enabled SAFE MODE
    // esc_motor_speed = -128;
  }

  leftAileron.setAngle(xRightJoystick);
  rightAileron.setAngle(yRightJoystick);

  // esc_motor.write(esc_motor_speed);

  Serial.print("xJoystick Right:");
  Serial.print(xRightJoystick);
  Serial.print("| ");

  Serial.print("yJoystick Right:");
  Serial.print(yRightJoystick);
  Serial.print("| ");

  Serial.print("xJoystick Left:");
  Serial.print(xLeftJoystick);
  Serial.print("| ");

  Serial.print("yJoystick Left:");
  Serial.print(yLeftJoystick);
  Serial.print("| ");

  Serial.print("motorSlider:");
  Serial.print(motorSlider);
  Serial.print("| ");

  Serial.print(" delay: ");
  Serial.print(requestDelay);
  Serial.println("");

  requestDelay = millis() - gotRequestTime;
};
