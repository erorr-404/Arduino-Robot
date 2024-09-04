#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint8_t AIN1 = 0;  // constants for first driver
const uint8_t AIN2 = 1;
const uint8_t BIN1 = 2;
const uint8_t BIN2 = 3;
#define SLP 8

const uint8_t AIN1_2 = 4;  // constants for second driver
const uint8_t AIN2_2 = 5;
const uint8_t BIN1_2 = 6;
const uint8_t BIN2_2 = 7;
#define SLP_2 4

Adafruit_PWMServoDriver pwmDriver;  // define PCA9685 object

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

RF24 radio(10, 9);  // nRF24L01 (CE, CSN)
const byte address[6] = "00001";

struct receiveData {  // structure of data to transmit to robot
  uint8_t leftX;      // left stick data
  uint8_t leftY;
  uint8_t leftP;

  uint8_t rightX;  // right stick data
  uint8_t rightY;
  uint8_t rightP;
};

receiveData data;

class Driver {  // motor driver class
public:
  Driver(uint8_t ain1, uint8_t ain2, uint8_t bin1, uint8_t bin2, short sleep) {
    this->ain1 = ain1;
    this->ain2 = ain2;
    this->bin1 = bin1;
    this->bin2 = bin2;
    this->sleep = sleep;
    this->initialize();
  }

  uint8_t ain1;  // driver ain1 pin
  uint8_t ain2;  // driver ain2 pin
  uint8_t bin1;  // driver bin1 pin
  uint8_t bin2;  // driver bin2 pin
  short sleep;   // driver stby pin

  void initialize() {
    // pwmDriver.setPin(ain1, 0);
    // pwmDriver.setPin(ain2, 0);
    // pwmDriver.setPin(bin1, 0);
    // pwmDriver.setPin(bin2, 0);

    this->awakeDriver();  // turn driver onn
  }

  void setMotorOneSpeed(short mot_speed) {
    if (mot_speed >= 0) {
      this->setMotorSpeed(1, uint16_t(mot_speed), false);
    } else {
      this->setMotorSpeed(1, uint16_t(mot_speed), true);
    }
  }

  void setMotorTwoSpeed(short mot_speed) {
    if (mot_speed >= 0) {
      this->setMotorSpeed(2, uint16_t(mot_speed), false);
    } else {
      this->setMotorSpeed(2, uint16_t(mot_speed), true);
    }
  }

  void setMotorSpeed(uint8_t mot_num, uint16_t mot_speed, bool reversed) {  // set speed of motor
    if (mot_num == 1) {                                                     // if we want to control motor 1
      this->setMotorPWM(mot_speed, ain1, ain2, reversed);                   // set motor speed using analogWrite func
    } else if (mot_num == 2) {                                              // if we want to control motor 2
      this->setMotorPWM(mot_speed, bin1, bin2, reversed);                   // set motor speed using analogWrite func
    } else {                                                                // if motor number is not 1 or 2
      Serial.println("Invalid motor number");                               // print error to console
    }
  }

  void driverSleep() {         // function to turn driver off
    digitalWrite(sleep, LOW);  // set low signal to stby pin
  }

  void awakeDriver() {          // function to turn driver on
    digitalWrite(sleep, HIGH);  // set high signal to stby pin
  }

private:
  void setMotorPWM(uint16_t pwm, uint8_t in1, uint8_t in2, bool reversed) {  // private func to control motor speed using analogWrite
    if (reversed) {                                                          // reverse speeds
      pwmDriver.setPin(in1, 0);
      pwmDriver.setPin(in2, pwm);

    } else {  // stop or forward
      pwmDriver.setPin(in1, pwm);
      pwmDriver.setPin(in2, 0);
    }
  }
};

Driver frontDriver(AIN1, AIN2, BIN1, BIN2, SLP);           // define front driver
Driver rearDriver(AIN1_2, AIN2_2, BIN1_2, BIN2_2, SLP_2);  // define rear driver, where 2 pins are not pwm

void moveForward(short speed = 4095) {
  frontDriver.setMotorOneSpeed(speed);
  frontDriver.setMotorTwoSpeed(speed);
  rearDriver.setMotorOneSpeed(speed);
  rearDriver.setMotorTwoSpeed(speed);
}

void moveBackward(short speed = 4095) {
  frontDriver.setMotorOneSpeed(-speed);
  frontDriver.setMotorTwoSpeed(-speed);
  rearDriver.setMotorOneSpeed(-speed);
  rearDriver.setMotorTwoSpeed(-speed);
}

void moveRight(short speed = 4095) {
  frontDriver.setMotorOneSpeed(-speed);
  frontDriver.setMotorTwoSpeed(speed);
  rearDriver.setMotorOneSpeed(speed);
  rearDriver.setMotorTwoSpeed(-speed);
}

void moveLeft(short speed = 4095) {
  frontDriver.setMotorOneSpeed(speed);
  frontDriver.setMotorTwoSpeed(-speed);
  rearDriver.setMotorOneSpeed(-speed);
  rearDriver.setMotorTwoSpeed(speed);
}

void moveForwardRight(short speed = 4095) {
  frontDriver.setMotorOneSpeed(0);
  frontDriver.setMotorTwoSpeed(speed);
  rearDriver.setMotorOneSpeed(speed);
  rearDriver.setMotorTwoSpeed(0);
}

void moveForwardLeft(short speed = 4095) {
  frontDriver.setMotorOneSpeed(speed);
  frontDriver.setMotorTwoSpeed(0);
  rearDriver.setMotorOneSpeed(0);
  rearDriver.setMotorTwoSpeed(speed);
}

void moveBackwardRight(short speed = 4095) {
  frontDriver.setMotorOneSpeed(-speed);
  frontDriver.setMotorTwoSpeed(0);
  rearDriver.setMotorOneSpeed(0);
  rearDriver.setMotorTwoSpeed(-speed);
}

void moveBackwardLeft(short speed = 4095) {
  frontDriver.setMotorOneSpeed(0);
  frontDriver.setMotorTwoSpeed(-speed);
  rearDriver.setMotorOneSpeed(-speed);
  rearDriver.setMotorTwoSpeed(0);
}

void turnRight(short speed = 4095) {
  frontDriver.setMotorOneSpeed(-speed);
  frontDriver.setMotorTwoSpeed(speed);
  rearDriver.setMotorOneSpeed(-speed);
  rearDriver.setMotorTwoSpeed(speed);
}

void turnLeft(short speed = 4095) {
  frontDriver.setMotorOneSpeed(speed);
  frontDriver.setMotorTwoSpeed(-speed);
  rearDriver.setMotorOneSpeed(speed);
  rearDriver.setMotorTwoSpeed(-speed);
}

void stop() {
  frontDriver.setMotorOneSpeed(0);
  frontDriver.setMotorTwoSpeed(0);
  rearDriver.setMotorOneSpeed(0);
  rearDriver.setMotorTwoSpeed(0);
}

void resetData() {
  data.leftX = 127;  // left stick data
  data.leftY = 127;
  data.leftP = 0;

  data.rightX = 127;  // right stick data
  data.rightY = 127;
  data.rightP = 0;
}

void setup() {
  Serial.begin(9600);  // connect to serial
  Serial.println("Robot starting...");
  pwmDriver.begin();  // init PCA9685
  Serial.println("PCA9685 inited.");
  pwmDriver.setPWMFreq(50);  // set PCA9685 pmw freq
  Serial.println("Robot started.");
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();  //  Set the module as receiver
  Serial.println("Radio started.");
  resetData();
}

void loop() {
  if (radio.available()) {  // Check whether there is data to be received
    Serial.println("Radio available!");
    radio.read(&data, sizeof(receiveData));  // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis();              // At this moment we have received the data
    Serial.println(data.leftX);
  }

  currentTime = millis();
  if (currentTime - lastReceiveTime > 500) {  // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData();                              // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone has a throttle up and we lose connection, it can keep flying unless we reset the values
  }

  if (data.rightX > 170) {
    turnRight();
  } else if (data.rightX < 85) {
    turnLeft();
  } else {
    if (data.leftX > 170) {
      if (data.leftY > 170) {
        moveForwardRight();
      } else if (data.leftY < 85) {
        moveBackwardRight();
      } else {
        moveRight();
      }
    } else if (data.leftX < 85) {
      if (data.leftY > 170) {
        moveForwardLeft();
      } else if (data.leftY < 85) {
        moveBackwardLeft();
      } else {
        moveLeft();
      }
    } else {
      if (data.leftY > 170) {
        moveForward();
      } else if (data.leftY < 85) {
        moveBackward();
      } else {
        stop();
      }
    }
  }



  // frontDriver.setMotorOneSpeed(4095);
  // frontDriver.setMotorTwoSpeed(4095);
  // rearDriver.setMotorOneSpeed(4095);
  // rearDriver.setMotorTwoSpeed(4095);
  // delay(5000);
  // frontDriver.setMotorOneSpeed(0);
  // frontDriver.setMotorTwoSpeed(0);
  // rearDriver.setMotorOneSpeed(0);
  // rearDriver.setMotorTwoSpeed(0);
  // delay(5000);

  // moveForward(2000);
  // delay(3000);
  // stop();
  // delay(5000);

  // moveBackward();
  // delay(3000);
  // stop();
  // delay(1000);

  // moveRight();
  // delay(3000);
  // stop();
  // delay(1000);

  // moveLeft();
  // delay(3000);
  // stop();
  // delay(1000);

  // moveForwardRight();
  // delay(3000);
  // stop();
  // delay(1000);

  // moveForwardLeft();
  // delay(3000);
  // stop();
  // delay(1000);

  // moveBackwardRight();
  // delay(3000);
  // stop();
  // delay(1000);

  // moveBackwardLeft();
  // delay(3000);
  // stop();
  // delay(1000);

  // turnRight();
  // delay(3000);
  // stop();
  // delay(1000);

  // turnLeft();
  // delay(3000);
  // stop();
  // delay(1000);



  // speed = map(analogRead(LY), 1023, 0, -255, 255);
  // turn = map(analogRead(RX), 0, 1023, -255, 255);
  // strafe = map(analogRead(LX), 0, 1023, -255, 255);

  // FL = speed + turn + strafe;
  // FR = speed - turn - strafe;
  // RL = speed + turn - strafe;
  // RR = speed - turn + strafe;

  // frontDriver.setMotorOneSpeed(FL);
  // frontDriver.setMotorTwoSpeed(FR);
  // rearDriver.setMotorOneSpeed(RL);
  // rearDriver.setMotorTwoSpeed(RR);
  // Serial.println("TEST");
}
