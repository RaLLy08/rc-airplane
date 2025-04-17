
// Code 1 : Sending Text (Transmitter)
// Library: TMRh20/RF24 (https://github.com/tmrh20/RF24/)

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>


// oled
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>



#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//




RF24 radio(9, 8); // CE, CSN
const byte address[10] = "ADDRESS01";


// joysticks
#define analogRightX A0  // Analog pin for X axis
#define analogRightY A1  // Analog pin for Y axis

#define analogLeftX A3  // Analog pin for X axis
#define analogLeftY A7  // Analog pin for Y axis
// 

#define potentiometer1 A2
#define voltmeterPin A3

int s1State;
int s1LastState;


void setup() {
  pinMode(potentiometer1, INPUT);

  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();


  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  screensaver();
};

void screensaver(void) {
  display.clearDisplay();

  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(20, 10);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  display.print("RC 3000");

  display.display();
  delay(300);
}

#define screen_x_left_margin 92
#define screen_y_gap 8
#define screen_y_top_margin 4

void displayLocalPositions(int hor, int ver, int8_t rot) {
  display.setTextSize(1);
  
  display.setCursor(screen_x_left_margin, screen_y_top_margin);

  display.print("|:");
  display.print(hor);

  display.setCursor(screen_x_left_margin, screen_y_gap + screen_y_top_margin);
  display.print("--:");
  display.print(ver);

  display.setCursor(screen_x_left_margin, screen_y_gap*2 + screen_y_top_margin);
  display.print("r:");
  display.print(rot);
}

void displayNRFInfo(unsigned long &receiveDelay, float &inputVoltage) {
  display.drawLine(86, 0, 86, display.height() - 1, SSD1306_WHITE);

  display.setTextSize(1);
  
  display.setCursor(0, 0);

  display.print("Delay ");
  display.drawLine(36, 0, 36, 8, SSD1306_WHITE);
  display.drawLine(36, 8, 85, 8, SSD1306_WHITE);
  display.print(" V:");
  display.print(inputVoltage);
  display.setTextSize(2);

  display.setCursor(0, 12);
  display.print((float) receiveDelay / 1000); 

  display.setTextSize(1);
  display.print("sec");

  // display.print(receiveDelay / 1000); // quotient 
  // display.print('.');
  // display.print((receiveDelay / 10) % 100); // remainder 
}

int8_t dataToSend[5] = {0, 0, 0, 0, 0}; // x1, y1, x2, y2, speed, key

#define voltageCalibratedValue 11.44  //= current mesure / other voltmeter measure
unsigned long previousMillis = 0; 
const int throttleUpdateDisplayEachMS = 20;       

unsigned long lastReceived = millis();
unsigned long receiveDelay = 0;
float inputVoltage = 0.0;  // The voltage at the input (0-12V)

void loop() {
  int xRightRead = map(analogRead(analogRightX), 0, 1023, -131, 127) + 4;
  int yRightRead = map(analogRead(analogRightY), 0, 1023, -128, 127) + 1;

  int xLeftRead = map(analogRead(analogLeftX), 0, 1023, -131, 127) + 4; // ?
  int yLeftRead = map(analogRead(analogLeftY), 0, 1023, -128, 127) + 1; // ?

  int rotRead = map(analogRead(potentiometer1), 0, 1023, -128, 127);
  float voltageRead = analogRead(voltmeterPin);

  // inputVoltage = (voltageRead * voltageCalibratedValue) / 1023.0;
  inputVoltage = voltageRead * voltageCalibratedValue / 1023.0;
  int8_t xRightValue = constrain(xRightRead, -128, 127);
  int8_t yRightValue = constrain(yRightRead, -128, 127);

  int8_t xLeftValue = constrain(xLeftRead, -128, 127);
  int8_t yLeftValue = constrain(yLeftRead, -128, 127);

  int8_t rotValue = constrain(rotRead, -128, 127);


  unsigned long currentMillis = millis();  // Get current time

  if (currentMillis - previousMillis >= throttleUpdateDisplayEachMS) {
    previousMillis = currentMillis; 

    display.clearDisplay();

    displayLocalPositions(
      map(xRightValue, -128, 127, 0, 180), 
      map(yRightValue, -128, 127, 0, 180), 
      map(rotValue, -128, 127, 0, 100)
    );

    displayNRFInfo(receiveDelay, inputVoltage);            

    display.display();
  }

  dataToSend[0] = xRightValue;
  dataToSend[1] = yRightValue;
  dataToSend[2] = xLeftValue;
  dataToSend[3] = yLeftValue;
  
  dataToSend[4] = rotValue;

  radio.writeFast(&dataToSend, sizeof(dataToSend));
  bool isReceived = radio.txStandBy();

  if (isReceived) {
    receiveDelay =  millis() - lastReceived;
    lastReceived = millis();
  } else {
    receiveDelay = millis() - lastReceived;
  }
};
