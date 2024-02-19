// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

float X_i = 0;
float Y_i = 0;
float thetaDotI = 0;
float thetaDotIrad = 0;
float kmCon = (2*M_PI)/(358);
unsigned long prevTime;
float prevRotRight = count_er;
float prevRotLeft = count_el;
float r = 17;



// Class to track robot position.
class Kinematics_c {
  public:
  
    // Constructor, must exist.
    Kinematics_c() {

    } 

    // Use this function to update
    // your kinematics

    void initialise(){
//      Serial.println("Kinematics has been initalised! :)");
      prevTime = millis();
    }

    void update(){
      // if this changes make sure to update the kmCon Variable
      int updateTime = 10;
      float X_dotr;
      if (millis()-prevTime > updateTime) {
        float newRotRight = count_er;
        float newRotLeft = count_el;
        prevTime = millis();
        float angularRateRight = ((newRotRight-prevRotRight)*kmCon);
        float angularRateLeft = ((newRotLeft-prevRotLeft)*kmCon);
        float X_r = (r/2)*(angularRateRight+angularRateLeft)*(-1);
        float thetaDotR = r/(2*44.6)*(angularRateLeft-angularRateRight)*(-1)*180/M_PI;
        float thetaDotRrad = r/(2*44.6)*(angularRateLeft-angularRateRight)*(-1);
        X_i = X_i + (X_r*cos(thetaDotIrad));
        Y_i = Y_i + (X_r*sin(thetaDotIrad));
        thetaDotI = thetaDotI + thetaDotR;
        thetaDotIrad = thetaDotIrad + thetaDotRrad;
        prevRotRight = newRotRight;
        prevRotLeft = newRotLeft;
      }
    }

};



#endif
