
/*********************************************************************
* Arduino feeder
* 
**********************************************************************/


//#ifndef FEEDER_CONFIG_H_
#define FEEDER_CONFIG_H_

#include <Servo.h>
#include <math.h>
#include <LiquidCrystal.h>

// Firmware version
#define FIRMWARE_VER "0.0.1"

// Tasks frequencies
#define CONTROL_MOTOR_SPEED_FREQUENCY          50   //hz

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

// define PWM pins
#define PWM_PIN_PUMP                     3  // paint pump pin
#define PWM_PIN_BUFFER                   6  // buffer pin
#define BUFFER_OFF_FILTER                3  // [seconds]
#define PUMP_IGNITION_TIME               5  // [seconds]
#define PUMP_CYCLE_ON_TIME               2  // [seconds]
#define PUMP_CYCLE_OFF_TIME              3  // [seconds] 

// define HVLP sensor initial angles
#define HVLP_CLOSE_POS                   (uint32_t)120 // closed position angle (deg)
#define HVLP_OPEN_POS                    (uint32_t)80 // open position angle (deg) - corresponds to 180deg

/*******************************************************************************
* SoftwareTimer
*******************************************************************************/
static uint32_t tTime[10];

// Pins Definitions
#define LCD_RS        8
#define LCD_E         9
#define LCD_D4        4
#define LCD_D5        5
#define LCD_D6        6
#define LCD_D7        7

#define MOTOR_ODOMETRY 2
#define MOTOR_PWMF 11
#define MOTOR_PWMR 3

//#define GEAR_RATIO  82
//#define DIAMETER 150
//#define PULSES_2_MM (DIAMETER * 3.1415) / GEAR_RATIO

// Function prototypes
float calculateVelocity(void);
void menu_handler(void);

/*******************************************************************************
* Declaration for GUI
*******************************************************************************/
//LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); // initialize the library with the numbers of the interface pins
char hello[13] = "hello world!";
int display_id = 0;

/*******************************************************************************
* Declaration for velocity measurement
*******************************************************************************/
int motor_pulses = 0;
int prev_motor_pulses = 0;
int motor_odo_counter = 0;
float motor_prev_time =0;
float motor_time_taken=0;
float motor_rpm=0;
float motor_velocity;


/*******************************************************************************
* Declaration for HVLP servo
*******************************************************************************/
Servo myservo;  // create servo object to control a servo

//Servo HVLP_servo;  // create servo object to control the HVLP servo
uint8_t servo_angle = 0;
int32_t hvlp_close_angle = HVLP_CLOSE_POS; 
int32_t hvlp_open_angle = HVLP_CLOSE_POS;  

/*******************************************************************************
* Declaration for jig parameters
*******************************************************************************/
int required_velocity = 20; // [cm/sec]
