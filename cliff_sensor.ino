#include "cliff_sensor.h"
/*******************************************************************************
* Cliff IR Sensor
*******************************************************************************/
void setup() {
 Serial.begin(9600); // start the serial port
}
 void CliffIRSensor(void)
 {
 ARR = analogRead(sensorR);// value from right sensor 
 ARL = analogRead(sensorL);// value from left sensor 

    //Serial.print("right analog read ");
    //Serial.println(ARR);
    //Serial.print("left analog read ");
    //Serial.println(ARL);
   
   if (ARR >= right_IR_val) // 7cm value
    right_IR=false; // right ir sensor find deck
    
    else
    right_IR=true; // right ir sensor find cliff
    
    if (ARL >= left_IR_val)  // 7cm value
    left_IR=false; // left ir sensor find deck
    
    else
    left_IR=true; // left ir sensor find cliff

  }
