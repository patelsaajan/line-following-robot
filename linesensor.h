// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H
#include <Arduino.h>

#define LS_LEFTLEFT_IN_PIN A11
#define LS_RIGHTRIGHT_IN_PIN A4
#define LS_LEFT_IN_PIN      A0
#define LS_CENTRE_IN_PIN    A2
#define LS_RIGHT_IN_PIN     A3
#define LED_POWER           11 // IR LED power

unsigned long start_time;
unsigned long end_time;
unsigned long sensor_time;
unsigned long timeout = 5000;
unsigned long current_time;
unsigned long elapsed_time;

// size of the arrary
# define NB_LS_PINS 5

// creates a list of the sensor and the recorded data
int ls_pin[NB_LS_PINS] = { LS_LEFT_IN_PIN, LS_CENTRE_IN_PIN, LS_RIGHT_IN_PIN, LS_LEFTLEFT_IN_PIN, LS_RIGHTRIGHT_IN_PIN };
unsigned long sensor_read[ NB_LS_PINS ];
unsigned long sensorMin[ NB_LS_PINS ];
unsigned long sensorMax[ NB_LS_PINS ];
unsigned long sensorRange[ NB_LS_PINS ];

// index variable 
int which;

// count for how many sensors have returned
int remaining = NB_LS_PINS;

// Class to operate the linesensor(s).
class LineSensor_c {
  public:
  
    // Constructor, must exist.
    LineSensor_c() {

    }
    // sets the input pins for the line sensor
    void initialise() {
      pinMode(LS_LEFT_IN_PIN, INPUT);
      pinMode(LS_RIGHT_IN_PIN, INPUT);
      pinMode(LS_CENTRE_IN_PIN, INPUT);
      pinMode(LS_LEFTLEFT_IN_PIN, INPUT);
      pinMode(LS_RIGHTRIGHT_IN_PIN, INPUT);
      pinMode(LED_POWER, OUTPUT);
      digitalWrite( LS_LEFT_IN_PIN, HIGH);
      digitalWrite(LED_POWER, HIGH);
      Serial.println("Line sensors have been initalised :)");
    }

    //a run function that collects the line sensor infomation
    void readLineSensor() {
      resetSensorArrary();
      chargeCapacitors();
      delayMicroseconds(10);
      sensorInputState();
      start_time = micros();
      checkSensors();
    }
    
    // function that toggles the led on [1] and off [0]
    void ledToggle(int powerState){
      if(powerState == 1) {
         pinMode(LED_POWER, OUTPUT);
         digitalWrite(LED_POWER, HIGH);
      }else if (powerState == 0) {
        pinMode(LED_POWER, INPUT);
      } else {
        Serial.print("Invalid entry please either enter a 1 or 0 as power state");
      }
      
    }

   // function that prints the the infomation cleanly in the serial
  void sensorPrint(int pickedSensor) {
      Serial.print("Sensor ");
      Serial.print(pickedSensor);
      Serial.print(":");
      Serial.println(sensor_read[pickedSensor]);
  }

  // a loop that charges all of the capacitors of the sensors in the arrary
  void chargeCapacitors() {
    for (int i=0; i<NB_LS_PINS; i++) {
      pinMode(ls_pin[i], OUTPUT);
      digitalWrite(ls_pin[i], HIGH);
    }
  }
  // changes the sensors to input in the arrary
  void sensorInputState(){
     for (int i=0; i<NB_LS_PINS; i++) {
      pinMode(ls_pin[i], INPUT);
    }
  }

  // resets the sesnsor arrary to 0 to collect new data for thr the checkSesnors functions 
  // so that it does not auto finish.
  void resetSensorArrary() {
    for (int i = 0; i < NB_LS_PINS; i++) {
      sensor_read[i] = 0;
  }
  }

  // the while loop that checks for the data in almost parallel
    void checkSensors() {
    // while loop to collect all of the sensor data in parallel
    remaining = NB_LS_PINS;
    while ( remaining > 0 ) {
  
      // for loop to iterate through the pins until all have returned
      for ( which = 0; which < NB_LS_PINS; which++){
   
      // if the sensor has returned low and checks if something has been saved previously
        if(digitalRead(ls_pin[which]) == LOW) {
  
          if (sensor_read[which] == 0) {
            current_time = micros();
            elapsed_time = current_time - start_time;
            sensor_read[which] = elapsed_time;
            remaining = remaining - 1; 
          }
        } // end of if statement that checks for lows 
  
      } // end of for loop
  
        // check if the elapsed_time has been passed
        current_time = micros();
        elapsed_time = current_time - start_time;
        if( elapsed_time >= timeout) {
          remaining = 0;
        }
      
      } // end of while loop
  
      sensor_time = micros()-start_time;
//      Serial.print("Total sensor time: ");
//      Serial.println(sensor_time);

      // adds data to the missing list index if the system times out
      for (int i = 0 ; i<NB_LS_PINS; i++){
        if (sensor_read[i] != 0){
//          sensorPrint(i);
        }else if (sensor_read[i] == 0){
          sensor_read[i] = timeout;
//          Serial.print("Sensor ");
//          Serial.print(i);
//          Serial.println(" timed out!");
      }
      
      }
  
}

};



#endif
