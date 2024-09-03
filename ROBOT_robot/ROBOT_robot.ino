#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

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

  void setMotorOneSpeed(uint16_t mot_speed) {
    this->setMotorSpeed(1, mot_speed);
  }

  void setMotorTwoSpeed(uint16_t mot_speed) {
    this->setMotorSpeed(2, mot_speed);
  }

  void setMotorSpeed(uint8_t mot_num, uint16_t mot_speed) {  // set speed of motor
    if (mot_num == 1) {                                      // if we want to control motor 1
      this->setMotorPWM(mot_speed, ain1, ain2);              // set motor speed using analogWrite func
    } else if (mot_num == 2) {                               // if we want to control motor 2
      this->setMotorPWM(mot_speed, bin1, bin2);              // set motor speed using analogWrite func
    } else {                                                 // if motor number is not 1 or 2
      Serial.println("Invalid motor number");                // print error to console
    }
  }

  void driverSleep() {         // function to turn driver off
    digitalWrite(sleep, LOW);  // set low signal to stby pin
  }

  void awakeDriver() {          // function to turn driver on
    digitalWrite(sleep, HIGH);  // set high signal to stby pin
  }

private:
  void setMotorPWM(uint16_t pwm, uint8_t in1, uint8_t in2) {  // private func to control motor speed using analogWrite
    if (pwm < 0) {                                            // reverse speeds
      pwmDriver.setPin(in1, -pwm);
      pwmDriver.setPin(in2, 0);

    } else {  // stop or forward
      pwmDriver.setPin(in1, 0);
      pwmDriver.setPin(in2, pwm);
    }
  }
};

Driver frontDriver(AIN1, AIN2, BIN1, BIN2, SLP);           // define front driver
Driver rearDriver(AIN1_2, AIN2_2, BIN1_2, BIN2_2, SLP_2);  // define rear driver, where 2 pins are not pwm

void moveForward(uint16_t speed = 4095) {
  frontDriver.setMotorOneSpeed(speed);
  frontDriver.setMotorTwoSpeed(speed);
  rearDriver.setMotorOneSpeed(speed);
  rearDriver.setMotorTwoSpeed(speed);
}

void moveBackward(uint16_t speed = 4095) {
  frontDriver.setMotorOneSpeed(-speed);
  frontDriver.setMotorTwoSpeed(-speed);
  rearDriver.setMotorOneSpeed(-speed);
  rearDriver.setMotorTwoSpeed(-speed);
}

void moveRight(uint16_t speed = 4095) {
  frontDriver.setMotorOneSpeed(-speed);
  frontDriver.setMotorTwoSpeed(speed);
  rearDriver.setMotorOneSpeed(speed);
  rearDriver.setMotorTwoSpeed(-speed);
}

void moveLeft(uint16_t speed = 4095) {
  frontDriver.setMotorOneSpeed(speed);
  frontDriver.setMotorTwoSpeed(-speed);
  rearDriver.setMotorOneSpeed(-speed);
  rearDriver.setMotorTwoSpeed(speed);
}

void moveForwardRight(uint16_t speed = 4095) {
  frontDriver.setMotorOneSpeed(0);
  frontDriver.setMotorTwoSpeed(speed);
  rearDriver.setMotorOneSpeed(speed);
  rearDriver.setMotorTwoSpeed(0);
}

void moveForwardLeft(uint16_t speed = 4095) {
  frontDriver.setMotorOneSpeed(speed);
  frontDriver.setMotorTwoSpeed(0);
  rearDriver.setMotorOneSpeed(0);
  rearDriver.setMotorTwoSpeed(speed);
}

void moveBackwardRight(uint16_t speed = 4095) {
  frontDriver.setMotorOneSpeed(-speed);
  frontDriver.setMotorTwoSpeed(0);
  rearDriver.setMotorOneSpeed(0);
  rearDriver.setMotorTwoSpeed(-speed);
}

void moveBackwardLeft(uint16_t speed = 4095) {
  frontDriver.setMotorOneSpeed(0);
  frontDriver.setMotorTwoSpeed(-speed);
  rearDriver.setMotorOneSpeed(-speed);
  rearDriver.setMotorTwoSpeed(0);
}

void turnRight(uint16_t speed = 4095) {
  frontDriver.setMotorOneSpeed(-speed);
  frontDriver.setMotorTwoSpeed(speed);
  rearDriver.setMotorOneSpeed(-speed);
  rearDriver.setMotorTwoSpeed(speed);
}

void turnLeft(uint16_t speed = 4095) {
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

void setup() {
  Serial.begin(9600);  // connect to serial
  delay(500);
  Serial.println("Robot starting...");
  pwmDriver.begin();  // init PCA9685
  Serial.println("PCA9685 inited.");
  pwmDriver.setPWMFreq(50);  // set PCA9685 pmw freq
  Serial.println("Robot started.");
}

void loop() {
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
