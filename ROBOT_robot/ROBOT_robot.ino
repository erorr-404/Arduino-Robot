#include <SoftPWM.h> // lib for software pwm
#include <SoftPWM_timer.h>

#define AIN1 10 // constants for first driver
#define AIN2 9
#define BIN1 6
#define BIN2 5
#define SLP 8

#define AIN1_2 3 // constants for second driver
#define AIN2_2 11
#define BIN1_2 12
#define BIN2_2 13
#define SLP_2 4

class Driver { // motor driver class
  public:
    Driver(short ain1, short ain2, short bin1, short bin2, short sleep, bool softPwmMot1, bool softPwmMot2) {
      this->ain1 = ain1;
      this->ain2 = ain2;
      this->bin1 = bin1;
      this->bin2 = bin2;
      this->sleep = sleep;
      this->softPwmMot1 = softPwmMot1;
      this->softPwmMot2 = softPwmMot2;
      this->initialize();
    }

    short ain1; // driver ain1 pin
    short ain2; // driver ain2 pin
    short bin1; // driver bin1 pin
    short bin2; // driver bin2 pin
    short sleep; // driver stby pin
    bool softPwmMot1; // pins for motor 1 are not pwm
    bool softPwmMot2; // pins for motor 2 are not pwm

    void initialize() {
      pinMode(ain1, OUTPUT); // set all pins to output
      pinMode(ain2, OUTPUT);
      pinMode(bin1, OUTPUT);
      pinMode(bin2, OUTPUT);
      pinMode(sleep, OUTPUT);

      digitalWrite(ain1, LOW);
      digitalWrite(ain2, LOW);
      digitalWrite(bin1, LOW);
      digitalWrite(bin2, LOW);
      
      this->awakeDriver(); // turn driver onn
    }

    void setMotorOneSpeed(short mot_speed) {
      this->setMotorSpeed(1, mot_speed);
    }

    void setMotorTwoSpeed(short mot_speed) {
      this->setMotorSpeed(2, mot_speed);
    }

    void setMotorSpeed(short mot_num, short mot_speed) { // set speed of motor
      if (mot_num == 1) { // if we want to control motor 1
        if (this->softPwmMot1) { // if these pins are not pwm
          this->setSoftMotorPWM(mot_speed, ain1, ain2); // set motor speed using SoftPWM
        } else { // else
          this->setMotorPWM(mot_speed, ain1, ain2); // set motor speed using analogWrite func
        }
        
      } else if (mot_num == 2) { // if we want to control motor 2
        if (this->softPwmMot2) { // if these pins are not pwm
          this->setSoftMotorPWM(mot_speed, bin1, bin2); // set motor speed using SoftPWM
        } else { // else 
          this->setMotorPWM(mot_speed, bin1, bin2); // set motor speed using analogWrite func
        }

      } else { // if motor number is not 1 or 2
        Serial.println("Invalid motor number"); // print error to console
      }
    }

    void driverSleep() { // function to turn driver off
      digitalWrite(sleep, LOW); // set low signal to stby pin
    }

    void awakeDriver() { // function to turn driver on
      digitalWrite(sleep, HIGH); // set high signal to stby pin
    }

  private:
    void setMotorPWM(short pwm, short in1, short in2) { // private func to control motor speed using analogWrite
      if (pwm < 0) {  // reverse speeds
        analogWrite(in1, -pwm);
        digitalWrite(in2, LOW);

      } else { // stop or forward
        digitalWrite(in1, LOW);
        analogWrite(in2, pwm);
      }
    }

    void setSoftMotorPWM(short pwm, short in1, short in2) { // private func to control motor speed using SoftPWM
      if (pwm < 0) {  // reverse speeds
        SoftPWMSet(in1, -pwm);
        digitalWrite(in2, LOW);

      } else { // stop or forward
        digitalWrite(in1, LOW);
        SoftPWMSet(in2, pwm);
      }
    }
};

Driver frontDriver(AIN1, AIN2, BIN1, BIN2, SLP, false, false); // define front driver
Driver rearDriver(AIN1_2, AIN2_2, BIN1_2, BIN2_2, SLP_2, false, true); // define rear driver, where 2 pins are not pwm

void setup() {
  Serial.begin(9600); // connect to serial
  SoftPWMBegin(); // init SoftPWM lib
}

void loop() {
  frontDriver.setMotorOneSpeed(255);
  frontDriver.setMotorTwoSpeed(255);
  rearDriver.setMotorOneSpeed(255);
  rearDriver.setMotorTwoSpeed(255);
  delay(5000);
  frontDriver.setMotorOneSpeed(0);
  frontDriver.setMotorTwoSpeed(0);
  rearDriver.setMotorOneSpeed(0);
  rearDriver.setMotorTwoSpeed(0);
  delay(5000);
}

// void loop() {
//   // Generate a fixed motion sequence to demonstrate the motor modes.

//   // Ramp speed up.
//   for (int i = 0; i < 11; i++) {
//     spin_and_wait(25 * i, 25 * i, 500);
//   }
//   // Full speed forward.
//   spin_and_wait(255, 255, 2000);

//   // Ramp speed into full reverse.
//   for (int i = 0; i < 21 ; i++) {
//     spin_and_wait(255 - 25 * i, 255 - 25 * i, 500);
//   }

//   // Full speed reverse.
//   spin_and_wait(-255, -255, 2000);

//   // Stop.
//   spin_and_wait(0, 0, 2000);

//   // Full speed, forward, turn, reverse, and turn for a two-wheeled base.
//   spin_and_wait(255, 255, 2000);
//   spin_and_wait(0, 0, 1000);
//   spin_and_wait(-255, 255, 2000);
//   spin_and_wait(0, 0, 1000);
//   spin_and_wait(-255, -255, 2000);
//   spin_and_wait(0, 0, 1000);
//   spin_and_wait(255, -255, 2000);
//   spin_and_wait(0, 0, 1000);
// }
