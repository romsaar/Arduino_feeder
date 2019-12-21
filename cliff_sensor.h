
// define cliff IR sensor TOF // 5V
#define sensorR A0 // Sharp IR Right
#define right_IR_val                      400 // 5V, value for 7cm 
#define sensorL A1 // sharp IR Left
#define left_IR_val                       400 // 5V, value for 7cm 

/*******************************************************************************
* IR cliff sensor
*******************************************************************************/
float ARR = 0.0;// value from right sensor 
bool right_IR = false;
float ARL = 0.0;// value from left sensor 
bool left_IR = false; 
// 
void CliffIRSensor(void);
