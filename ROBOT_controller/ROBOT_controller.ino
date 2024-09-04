#include <SPI.h>  // include needed libs
#include <nRF24L01.h>
#include <RF24.h>

#define LX A1  // define sticks pins
#define LY A0
#define LP 2
#define RX A3
#define RY A2
#define RP 3

class Joystick {  // define joystick class
public:
  Joystick(byte xPin, byte yPin, byte pressPin) {
    this->xPin = xPin;
    this->yPin = yPin;
    this->pressPin = pressPin;
    initialize();
  }

  byte xPin;
  byte yPin;
  byte pressPin;
  int lastX;
  int lastY;

  void initialize() {
    pinMode(pressPin, INPUT);
  }

  int readLX() {  // function to read joystick x value
    int rawValue = analogRead(this->xPin);
    int value = map(rawValue, 0, 1023, 1023, 0);
    this->lastX = value;
    return value;
  }

  int readLY() {  // function to read joystick y value
    int rawValue = analogRead(this->yPin);
    int value = map(rawValue, 0, 1023, 1023, 0);
    this->lastY = value;
    return value;
  }

  bool readLP() {  // function to read joystick button
    switch (digitalRead(this->pressPin)) {
      case LOW:
        return false;
        break;
      case HIGH:
        return true;
        break;
    }
  }
};

struct transmitData {  // structure of data to transmit to robot
  byte leftX;          // left stick data
  byte leftY;
  byte leftP;

  byte rightX;  // right stick data
  byte rightY;
  byte rightP;
};


Joystick leftStick(LX, LY, LP);   // create left stick instance
Joystick rightStick(RX, RY, RP);  // create right stick instance

RF24 radio(7, 8);  // create radio instance, pins: CE, CSN

const byte address[6] = "00001";  // adress of controller

transmitData data;  // data to send

void setup() {
  radio.begin();                   // init radio
  radio.openWritingPipe(address);  // set adress
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
}

void loop() {
  // get sticks values
  data.leftX = map(analogRead(LX), 0, 1023, 0, 255);
  data.leftY = map(analogRead(LY), 0, 1023, 0, 255);
  data.leftP = digitalRead(LP);

  data.rightX = map(analogRead(RX), 0, 1023, 0, 255);
  data.rightY = map(analogRead(RY), 0, 1023, 0, 255);
  data.rightP = digitalRead(RP);

  //send data
  radio.write(&data, sizeof(transmitData));
}
