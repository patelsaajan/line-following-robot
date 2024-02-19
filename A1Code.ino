// files that contain the other classes needed for the file to complie
#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
# define BUZZER_PIN 6

// creating intances of each class that is going to be used
Motors_c motors;
LineSensor_c ls;
Kinematics_c km;
PID_c straightPidRight;
PID_c straightPidLeft;
PID_c spd_pid_left;
PID_c spd_pid_right;
PID_c header_pid;

float straightSpeedDemand = 0.6;
float maxTurnPWM = 80;
float motorSpeedStraight = 25;
float motorSpeedCorner = 25;

// creating global variables for any time steps that are going to be used
unsigned long pid_test_ts;
unsigned long sensorUpdate_ts; // updates timestamp
unsigned long speedUpdate_ts;
unsigned long loop_ts;
unsigned long rotation_ts;
unsigned long lostLine_ts;
unsigned long atStart_ts;
unsigned long rotDelay_ts;;

// global low past filter variables
float ave_er_spd; // make a low pass filter of speed
float ave_el_spd; // make a low pass filter of speed

float eline;
float homeAngle;
float finalX;
float finalY;

long count_er_last; // for difference in encoder counts
long count_el_last; // for difference in encoder counts

// bool state locks defined
bool onLine = false;
bool rotation = false;
bool rotatePlus90 = false;
bool rotateRight = false;
bool rotateLeft = false;
bool spin = false;
bool spin180 = false;
bool findLine = true;
bool lostLine = false;
bool doneSpin = false;
bool turningDelay = false;
bool startMode = true;
bool returnHome = false;
bool stage1 = false;
bool stage2 = false;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(500);
  Serial.println("***Reset***");
  delay(500);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  setupEncoder0();
  setupEncoder1();
  motors.initialise();
  km.initialise();
  ls.initialise();

  straightPidRight.initialise(55, 0.35, 150);
  straightPidLeft.initialise(55, 0.35, 150);

  spd_pid_left.initialise(0.8, 0.0003, 200);
  spd_pid_right.initialise(0.8, 0.0003, 200);
  header_pid.initialise(0.5, 0, 4);

  count_er_last = count_er;
  count_el_last = count_el;

  // reset the pid
  straightPidRight.reset();
  straightPidLeft.reset();

  ave_er_spd = 0.0;
  ave_el_spd = 0.0;

  loop_ts = millis();
  sensorUpdate_ts = millis();
  speedUpdate_ts = millis();
  rotation_ts = millis();
  atStart_ts = millis();
}

void loop() {
  // ************************************************************************** SENSOR UPDATE CODE
  if (millis() - sensorUpdate_ts > 10) {
    ls.readLineSensor();
    getWeights();
    sensorUpdate_ts = millis;
    km.update();
  }
  // ************************************************************************** FINDING THE LINE FIRST
  if (findLine) {
    if (sensor_read[3] > 1300 && sensor_read[4] > 1300) {
      turningDelay = true;
    } else if (turningDelay && sensor_read[3] < 1300 && sensor_read[4] < 1300) {
      motors.setMotorValues(0, 0);
      findLine = false;
      rotatePlus90 = true;
      rotation = true;
    } else {
      straightPIDController();
    }
  }

  // ************************************************************************** ONLINE CODE
  if (onLine && not rotation && not spin && not rotateLeft && not rotateRight) {
    digitalWrite(LED_BUILTIN, LOW);
    lineFollowingBehaviour();
    if (sensor_read[1] < 1400) {
      if ((millis() - atStart_ts) < 5000 && not doneSpin) {
        motors.setMotorValues(0, 0);
        spin = true;
        spin180 = true;
      }
      if (sensor_read[3] > 1100) {
        rotateLeft = true;
        onLine = false;
        rotDelay_ts = millis();
        digitalWrite(LED_BUILTIN, HIGH);

      } else if (sensor_read[4] > 1100) {
        rotateRight = true;
        onLine = false;
        rotDelay_ts = millis();
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        motors.setMotorValues(0, 0);
        lostLine = true;
        lostLine_ts = millis();
        resetPIDs();

      }
    }
  }

  // ************************************************************************** 90 DEGREE CODE

  if ((millis() - rotDelay_ts) > 200) {
    if (rotateRight) {
      motors.setMotorValues(20, 0);
      if (sensor_read[4] < 1100 && sensor_read[1] > 1500) {
        rotateRight = false;
        onLine = true;
        motors.setMotorValues(0, 0);
      }
    }

    if (rotateLeft) {
      motors.setMotorValues(0, 20);
      if (sensor_read[3] < 1100 && sensor_read[1] > 1500) {
        rotateLeft = false;
        onLine = true;
        motors.setMotorValues(0, 0);
      }
    }
  }

  // ************************************************************************** LOST LINE CODE
  if (lostLine) {
    onLine = false;
    straightPIDController();
    if (sensor_read[1] > 1300) {
      lostLine = false;
      onLine = true;
    } else if (millis() - lostLine_ts > 1300 && sensor_read[1] < 1300) {
      motors.setMotorValues(0, 0);
      lostLine = false;
      returnHome = true;
      stage1 = true;
      km.update();
      finalX = X_i;
      finalY = Y_i;
      homeAngle = atan2(finalY, finalX) * 180/M_PI;
      resetPIDs();
    }
  }

  // ************************************************************************** ROTATION CODE
  if (rotation) {
    float rotateAngle;
    if (rotatePlus90) {
      rotateAngle = thetaDotI + 90;
      resetPIDs();
      rotatePlus90 = false;
    }
    rotationFunction(rotateAngle);
  }

  if (spin) {
    float rotateAngle;
    if (spin180) {
      rotateAngle = thetaDotI - 180;
      resetPIDs();
      spin180 = false;
      doneSpin = true;
    }
    rotationFunction(rotateAngle);
  }

  if (returnHome) {
    if (millis() - rotation_ts > 20) {
      rotation_ts = millis();
      float returnAngle;
      aveSpeedUpdate();
      km.update();
      if (stage1) {
        // to deal with the growing error in the kinematics 
        if (abs(homeAngle >= 7)) {
          returnAngle = -180 + homeAngle + 4.2; 
        } else if (abs(homeAngle < 7)) {
          returnAngle = -180 + homeAngle + 2.1; 
        }
        if (thetaDotI < returnAngle) {
          motors.setMotorValues(0, 0);
          resetPIDs();
          stage1 = false;
          stage2 = true;
        }
        else {
          motors.setMotorValues(-16, 16);
        }
      } else if (stage2) {
        if (X_i < 10) {
          motors.setMotorValues(0, 0);
          digitalWrite(LED_BUILTIN, HIGH);
          stage2 = false;
        } else {
          straightPIDController();
        }
      }
    }
  }
}

// ************************************************************************** CODE FUNCTIONS

void straightPIDController() {
  aveSpeedUpdate();
  float leftPwm = straightPidLeft.update(ave_el_spd, straightSpeedDemand);
  float rightPwm = straightPidRight.update(ave_er_spd, straightSpeedDemand);
  motors.setMotorValues(leftPwm, rightPwm);
}

void resetPIDs() {
  ave_er_spd = 0;
  ave_el_spd = 0;
  spd_pid_left.reset();
  spd_pid_right.reset();
  straightPidLeft.reset();
  straightPidRight.reset();
}

void rotationFunction(float desiredAngle) {
  aveSpeedUpdate();
  km.update();
  if (millis() - rotation_ts > 100) {
    if (sensor_read[1] > 1100 && (abs(desiredAngle - thetaDotI) < 15)) {
      motors.setMotorValues(0, 0);
      rotation = false;
      spin = false;
      if (millis() - atStart_ts < 5000) {
        onLine = true;
      }
    } else {
      float headerFeedback = header_pid.update(desiredAngle, thetaDotI);
      float Rpwm = spd_pid_right.update(headerFeedback * -1, ave_er_spd);
      float Lpwm = spd_pid_left.update((headerFeedback), ave_el_spd);
      motors.setMotorValues(Lpwm, Rpwm);
      rotation_ts = millis();
    }
  }
}



void lineFollowingBehaviour() {
  float turnPWM = maxTurnPWM * eline;
  // Serial.println(turnPWM);
  if (abs(turnPWM) < 3) {
    //  Serial.println("Straight");
    motors.setMotorValues( (motorSpeedStraight), (motorSpeedStraight));
  } else {
    motors.setMotorValues( (motorSpeedCorner - turnPWM), (motorSpeedCorner + turnPWM));
  }
}


void aveSpeedUpdate() {
  unsigned long elapsed; // calculate difference in time

  //    last time, current time
  elapsed = millis() - speedUpdate_ts;

  if (elapsed > 20) {
    speedUpdate_ts = millis();

    long diff_er; // calculates the difference
    float er_speed; // speed calculated for right encoder
    long diff_el; // calculates the difference
    float el_speed; // speed calculated for left encoder

    diff_er = (count_er * -1) - count_er_last;
    count_er_last = (count_er * -1);

    diff_el = (count_el * -1) - count_el_last;
    count_el_last = (count_el * -1);

    er_speed = (float)diff_er;
    er_speed /= (float)elapsed; // actial elapsed ms
    el_speed = (float)diff_el;
    el_speed /= (float)elapsed; // actial elapsed ms

    // low pass filter that uses average combined with the new reading
    ave_er_spd = (ave_er_spd * 0.7) + (er_speed * 0.3);
    ave_el_spd = (ave_el_spd * 0.7) + (el_speed * 0.3);
  }
}


void getWeights() {
  float leftSensorReading = sensor_read[0];
  //  Serial.println(sensor_read[0]);
  float centreSensorReading = sensor_read[1];
  float rightSensorReading = sensor_read[2];
  float normalisationValue = (leftSensorReading + rightSensorReading + centreSensorReading);
  float normLeftSR = leftSensorReading / normalisationValue;
  float normRightSR = rightSensorReading / normalisationValue;
  float normCentreSR = centreSensorReading / normalisationValue;
  float Wleft = normLeftSR + (0.5 * normCentreSR);
  float Wright = normRightSR + (0.5 * normCentreSR);
  eline = Wleft - Wright;
  //  Serial.println(eline);
}
