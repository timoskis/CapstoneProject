/*
* @File - GMerry-Firmware.ino
* @Brief - Capstone Project Firmware
* @Author - Olusodo Timothy
* @Version - V1.0
* @Date - 2024-12-01
*/


//-Dabble BLE App Init-//
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>


//-TOF VL530X Sensor Init-//
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
DFRobot_VL53L0X TOFsensor;  //TOF sensor object initialization


//-ADXL345 & NeoPixel Ring Init-//
#include <SPI.h>
#include <Adafruit_NeoPixel.h>  // Include NeoPixel Library
// Variables for storing accel readings
int X_axis = 0, Y_axis = 0, Z_axis = 0;
int samplesize = 10;
long int previousT = 0;
// NeoPixel configuration
#define LED_PIN 22   // Pin where NeoPixel is connected
#define NUM_LEDS 24  // Number of NeoPixels
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
// Variables to control animation
int currentPixel = 0;              // Tracks which LED is currently lit
int snakeLength = 8;               // Length of the "snake" (number of lit LEDs)
int direction = 1;                 // Movement direction: 1 = forward, -1 = backward
unsigned long previousMillis = 0;  // Tracks time for animation timing
const int interval = 100;          // Speed of the snake (lower = faster)


//-Motor PWM & Direction Pins Init-//
const int E1 = 3;     ///<Motor1 Speed
const int E2 = 11;    ///<Motor2 Speed
const int E3 = 5;     ///<Motor3 Speed
const int E4 = 6;     ///<Motor4 Speed
const int M1 = 4;     ///<Motor1 Direction
const int M2 = 12;    ///<Motor2 Direction
const int M3 = 8;     ///<Motor3 Direction
const int M4 = 7;     ///<Motor4 Direction
float MAXPWM = 2.55;  //MAXPWM*SPEED (0 - 100)


//-Linear Actuator Init-//
bool mastState = false;          //Variable to track the position of the mast
bool prevTriangleState = false;  //Variable to track the previous triangle button state
const int PWM1 = 10;
const int DIR1 = 9;


//-DYSV5W Sound Module Init-//
byte commandLength;  //Variable to track length of command sent to the register
byte command[6];     //Array used to store commands being sent to the register
int checkSum = 0;    //
unsigned long lastPressTime = 0;
const int debounceDelay = 200;  // 200ms debounce delay





//****SETUP FUNCTION****//
//-------------------//
void setup() {
  //Initialize Serial Communication
  Serial.begin(115200);
  Dabble.begin(115200);
  Serial1.begin(9600);


  //TOF Sensor Setup
  Wire.begin();                                               //Join I2C bus
  TOFsensor.begin(0x50);                                      //Set I2C sub-device address
  TOFsensor.setMode(TOFsensor.eContinuous, TOFsensor.eHigh);  //Set to Back-to-back mode and high precision mode
  TOFsensor.start();                                          //Laser rangefinder begins to work


  //ADXL345 & NeoPixel Setup
  DDRB &= 0XF7;
  DDRB |= 0X07;  // Sets SPI port bits
  SPI.begin();   // Initialize SPI communication
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3));
  writeRegister(0x31, 0x00);  // Configure DataFormat Register
  writeRegister(0x2D, 0x08);  // Configure PowerControl Register

  strip.begin();             // Initialize NeoPixel
  strip.setBrightness(128);  // Set brightness to 50% (range: 0-255)
  strip.show();              // Initialize NeoPixel (all LEDs off)


  //Setup DC Motor Pins
  for (int i = 3; i < 9; i++)
    pinMode(i, OUTPUT);
  for (int i = 11; i < 13; i++)
    pinMode(i, OUTPUT);


  //Setup Linear Actuator Pins
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  digitalWrite(DIR1, LOW);


  //DYSV5W SETUP
  setVolume(0x17);    //Set DYSV5W volume to 20
  setPlayMode(0x04);  //Put DYSV5W in directory loop mode (0x04)
}





//****LOOP FUNCTION****//
//-------------------//
void loop() {
  Dabble.processInput();  //Process input from Dabble app

  //--SHIP CONTROL--//
  bool movementTriggered = false;          //Flag to track if any movement button is pressed
  int distance = TOFsensor.getDistance();  //Read the distance from the ToF sensor

  //Check if there's an OBSTACLE within 150mm
  if (distance < 150) {
    //OBSTACLE DETECTED: only allow backing up
    if (GamePad.isDownPressed()) {
      backup(50);  //Move backward
      movementTriggered = true;
    } else {
      halt();                    //Halt the ship
      movementTriggered = true;  //Prevent halt logic below from running again
    }
  } else {
    //NO OBSTACLE DETECTED: normal movement
    if (GamePad.isUpPressed()) {
      sail(80);  //Move forward
      movementTriggered = true;
    } else if (GamePad.isRightPressed()) {
      turnRight(70);  //Turn right
      movementTriggered = true;
    } else if (GamePad.isLeftPressed()) {
      turnLeft(70);  //Turn left
      movementTriggered = true;
    } else if (GamePad.isDownPressed()) {
      backup(70);  //Move backward
      movementTriggered = true;
    }
  }
  //If no movement button is pressed, halt the motors
  if (!movementTriggered) {
    halt();
  }


  //--MAST CONTROL--//
  bool isTrianglePressed = GamePad.isTrianglePressed();
  if (isTrianglePressed && !prevTriangleState) {
    mastState = !mastState;
    digitalWrite(DIR1, mastState ? HIGH : LOW);
    analogWrite(PWM1, 255);
  }
  prevTriangleState = isTrianglePressed;


  //--DYSV5W CONTROL--//
  unsigned long gamepadCurrentTime = millis();
  if (gamepadCurrentTime - lastPressTime > debounceDelay) {
    if (GamePad.isStartPressed()) {
      playTrack();
      lastPressTime = gamepadCurrentTime;
    } else if (GamePad.isCrossPressed()) {
      pauseTrack();
      lastPressTime = gamepadCurrentTime;
    } else if (GamePad.isSquarePressed()) {
      prevTrack();
      lastPressTime = gamepadCurrentTime;
    } else if (GamePad.isCirclePressed()) {
      nextTrack();
      lastPressTime = gamepadCurrentTime;
    }
  }


  //--ADXL345 & NeoPixel CONTROL--//
  //------------------//
  ACLR8_Read();  // Read accelerometer data

  // Check if ship is ready to begin sailing
  float totalAccel = sqrt(X_axis * X_axis + Y_axis * Y_axis + Z_axis * Z_axis);
  if (totalAccel < 1000) {   // As soon as the slightest motion is sensed
    snakePattern(224, 17, 95);  // Trigger NeoPixel movement animation (Purple)
  }
}





//*** FUNCTIONS***//
//---------------//

//---MOTOR FUNCTIONS---//
void sail(int Speed) {
  ///<Motor1 Advance
  digitalWrite(M1, LOW);
  analogWrite(E1, Speed * MAXPWM);
  ///<Motor2 Advance
  digitalWrite(M2, HIGH);
  analogWrite(E2, Speed * MAXPWM);
  ///<Motor3 Advance
  digitalWrite(M3, HIGH);
  analogWrite(E3, Speed * MAXPWM);
  ///<Motor4 Advance
  digitalWrite(M4, LOW);
  analogWrite(E4, Speed * MAXPWM);
}

void backup(int Speed) {
  ///<Motor1 Reverse
  digitalWrite(M1, HIGH);
  analogWrite(E1, Speed * MAXPWM);
  ///<Motor2 Reverse
  digitalWrite(M2, LOW);
  analogWrite(E2, Speed * MAXPWM);
  ///<Motor3 Reverse
  digitalWrite(M3, LOW);
  analogWrite(E3, Speed * MAXPWM);
  ///<Motor4 Reverse
  digitalWrite(M4, HIGH);
  analogWrite(E4, Speed * MAXPWM);
}

void turnRight(int Speed) {
  ///<Motor1 Advance
  digitalWrite(M1, LOW);
  analogWrite(E1, Speed * MAXPWM);
  ///<Motor2 Advance
  digitalWrite(M2, HIGH);
  analogWrite(E2, Speed * MAXPWM);
}

void turnLeft(int Speed) {
  ///<Motor3 Advance
  digitalWrite(M3, HIGH);
  analogWrite(E3, Speed * MAXPWM);
  ///<Motor4 Advance
  digitalWrite(M4, LOW);
  analogWrite(E4, Speed * MAXPWM);
}

void halt() {
  ///<Motor1 Stop
  analogWrite(E1, 0);
  ///<Motor2 Stop
  analogWrite(E2, 0);
  ///<Motor3 Stop
  analogWrite(E3, 0);
  ///<Motor4 Stop
  analogWrite(E4, 0);
}


//---SOUND MODULE FUNCTIONS---//
//Serially send commands to the DYSV5W register table in order.
//Start Code
//Command Type
//Data Length
//Data
//Check Bit (Only used for commands that set the device's functionality)
//E.G - Play Command is [0xAA 0x02 0x00 0xAC]
void sendCommand() {
  for (int q = 0; q < commandLength; q++) {
    Serial1.write(command[q]);
    Serial.print(command[q], HEX);
  }
  Serial.println(" End");
}

//Set DYSV5W Volume
void setVolume(byte vol) {
  command[0] = 0xAA;  //first byte says it's a command
  command[1] = 0x13;
  command[2] = 0x01;
  command[3] = vol;  //volume
  checkSum = 0;
  for (int q = 0; q < 4; q++) {
    checkSum += command[q];
  }
  command[4] = lowByte(checkSum);  //SM check bit... low bit of the sum of all previous values
  commandLength = 5;
  sendCommand();
}

//Set DYSV5W Play Mode
void setPlayMode(byte mode) {
  command[0] = 0xAA;  //first byte says it's a command
  command[1] = 0x18;
  command[2] = 0x01;
  command[3] = mode;  //play mode
  checkSum = 0;
  for (int q = 0; q < 4; q++) {
    checkSum += command[q];
  }
  command[4] = lowByte(checkSum);  //SM check bit... low bit of the sum of all previous values
  commandLength = 5;
  sendCommand();
}

//DYSV5W - Play Track
void playTrack() {
  command[0] = 0xAA;  //first byte says it's a command
  command[1] = 0x02;
  command[2] = 0x00;
  command[3] = 0xAC;
  commandLength = 4;
  sendCommand();
}

//DYSV5W - Pause Track
void pauseTrack() {
  command[0] = 0xAA;  //first byte says it's a command
  command[1] = 0x03;
  command[2] = 0x00;
  command[3] = 0xAD;
  commandLength = 4;
  sendCommand();
}

//DYSV5W - Play Next Track
void nextTrack() {
  command[0] = 0xAA;  //first byte says it's a command
  command[1] = 0x06;
  command[2] = 0x00;
  command[3] = 0xB0;
  commandLength = 4;
  sendCommand();
}

//DYSV5W - Play Previous Track
void prevTrack() {
  command[0] = 0xAA;  //first byte says it's a command
  command[1] = 0x05;
  command[2] = 0x00;
  command[3] = 0xAF;
  commandLength = 4;
  sendCommand();
}


//---ADXL345 FUNCTIONS---//
//----------------------//
//Function to write data to a specific register
void writeRegister(char registerAddress, char data) {
  PORTB &= 0xFE;
  SPI.transfer(registerAddress);  // Point to the required ACLR8 internal address
  SPI.transfer(data);             // Send the data to the internal register
  PORTB |= 0x01;                  // Set PB0 high (idle state)
  SPI.endTransaction();
}


//Read accelerometer data and compute average of 10 samples
void ACLR8_Read() {
  int X_LSB = 0, X_MSB = 0;
  int Y_LSB = 0, Y_MSB = 0;
  int Z_LSB = 0, Z_MSB = 0;

  long int LX_axis = 0, LY_axis = 0, LZ_axis = 0;
  char startingAddress = 0xF2;

  for (int i = 0; i < samplesize; i++) {
    PORTB &= 0xFE;
    SPI.transfer(startingAddress);
    X_LSB = SPI.transfer(0x00);
    X_MSB = SPI.transfer(0x00);
    Y_LSB = SPI.transfer(0x00);
    Y_MSB = SPI.transfer(0x00);
    Z_LSB = SPI.transfer(0x00);
    Z_MSB = SPI.transfer(0x00);
    PORTB |= 0x01;
    SPI.endTransaction();

    LX_axis = (X_MSB << 8) | (X_LSB & 0x00FF);
    LY_axis = (Y_MSB << 8) | (Y_LSB & 0x00FF);
    LZ_axis = (Z_MSB << 8) | (Z_LSB & 0x00FF);
  }
  X_axis = LX_axis / samplesize;
  Y_axis = LY_axis / samplesize;
  Z_axis = LZ_axis / samplesize;
}



//---NEOPIXEL RING FUNCTIONS---//
//----------------------//
// Function to create a snake-like animation
// Parameters:
//   r: Red color component (0-255)
//   g: Green color component (0-255)
//   b: Blue color component (0-255)
void snakePattern(int r, int g, int b) {
  unsigned long currentMillis = millis();  // Get the current time
  
  // Run the animation at the specified interval
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Update the time
    
    // Clear all LEDs
    strip.clear();
    
    // Light up the snake LEDs
    for (int i = 0; i < snakeLength; i++) {
      int pixelIndex = (currentPixel - i + NUM_LEDS) % NUM_LEDS;  // Calculate pixel index (wrap around at the ends)
      strip.setPixelColor(pixelIndex, strip.Color(r, g, b));          // Set color for the current pixel
    }
    
    strip.show();  // Update the NeoPixel strip
    
    // Move the snake forward
    currentPixel = (currentPixel + 1) % NUM_LEDS;  // Increment the head position and wrap around if needed
  }
}





