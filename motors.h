// this #ifndef stops this file
// from being included more than
// once by the compiler.
#ifndef _MOTORS_H
#define _MOTORS_H

#include <Arduino.h>

int L_PWM_PIN = 10;
int L_DIR_PIN = 16;
int R_PWM_PIN = 9;
int R_DIR_PIN = 15;
int lowerLimit = 15;
int upperLimit = 200;

#define FWD  LOW
#define REV  HIGH



// Class to operate the motor(s).
class Motors_c {
  public:

    // Constructor, must exist.
    Motors_c() {

    }

    // Use this function to
    // initialise the pins and
    // state of your motor(s).
    void initialise() {
      pinMode(L_PWM_PIN, OUTPUT);
      pinMode(L_DIR_PIN, OUTPUT);
      pinMode(R_PWM_PIN, OUTPUT);
      pinMode(R_DIR_PIN, OUTPUT);
      digitalWrite(L_DIR_PIN, HIGH);
      digitalWrite(R_DIR_PIN, HIGH);
//      Serial.println("Motor pins have been initalised :)");
    }

    // Write a function to operate
    // your motor(s)
    // ...

    void activate(String chosenMotor, float value_PWM) {
       if (chosenMotor == "RIGHT") {
        setMotor(R_PWM_PIN, R_DIR_PIN, value_PWM);
//        Serial.println("Right Motor Selected");
       }else if (chosenMotor == "LEFT") {
        setMotor(L_PWM_PIN, L_DIR_PIN, value_PWM);
//        Serial.println("Left motor selected");
       }
    }

    void setMotorValues( float leftPWM,float rightPWM) {
               if (leftPWM > 0){
                digitalWrite(L_DIR_PIN, FWD);
               } else if (leftPWM < 0) {
                digitalWrite(L_DIR_PIN, REV);
               }

               if (rightPWM > 0){
                digitalWrite(R_DIR_PIN, FWD);
               } else if (rightPWM < 0) {
                digitalWrite(R_DIR_PIN, REV);
               }


               if (abs(leftPWM) > upperLimit) {
                analogWrite(L_PWM_PIN, upperLimit);
               } else {
                analogWrite(L_PWM_PIN, abs(leftPWM));
               }
               
                if (abs(rightPWM) > upperLimit) {
                analogWrite(R_PWM_PIN, upperLimit);
               } else {
                analogWrite(R_PWM_PIN, abs(rightPWM));
               }
    }
    
        void setMotor(int motorPin, int motorDirection, float value_PWM) {
          if (value_PWM > 0) {
            digitalWrite(motorDirection, FWD);
//            Serial.println("FWD");
          } else if (value_PWM < 0) {
            digitalWrite(motorDirection, REV);
//            Serial.println("REV");
          } else {
//           Serial.println("Do nothing!");
           }
            
           
          if ( abs(value_PWM) > lowerLimit && abs(value_PWM) < upperLimit){
            analogWrite(motorPin, abs(value_PWM));
            
        } else if (abs(value_PWM) <= lowerLimit) {
            analogWrite(motorPin, 0);
//            Serial.println("PWM value is too low, motors are therefore stopped.");
            
        } else if (abs(value_PWM) >= upperLimit){
            analogWrite(motorPin, upperLimit);
            Serial.println("PWM value exceeded upper limit, therefore held at it.");
    }
}

  void spin() {
    digitalWrite(L_DIR_PIN, FWD);
    digitalWrite(R_DIR_PIN, REV);
    analogWrite(L_PWM_PIN, 40);
    analogWrite(R_PWM_PIN, 40);
  }
};



#endif
