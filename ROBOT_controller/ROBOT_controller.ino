#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define LX A1
#define LY A0
#define LP 2
#define RX A3
#define RY A2
#define RP 3

class Joystick {
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
    
    int readLX() {
      int rawValue = analogRead(this->xPin);
      int value = map(rawValue, 0, 1023, 1023, 0);
      this->lastX = value;
      return value;
    }

    int readLY() {
      int rawValue = analogRead(this->yPin);
      int value = map(rawValue, 0, 1023, 1023, 0);
      this->lastY = value;
      return value;
    }

    bool readLP() {
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

Joystick leftStick(LX, LY, LP);
Joystick rightStick(RX, RY, RP);

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  Serial.begin(9600);
}

void loop() {
  // Serial.print("LX:");
  // Serial.print(leftStick.readLX());
  // Serial.print(",");
  // Serial.print("LY:");
  // Serial.println(rightStick.readLY());

  const char text[] = "Hello World!";
  radio.write(&text, sizeof(text));
  delay(2000);
}
