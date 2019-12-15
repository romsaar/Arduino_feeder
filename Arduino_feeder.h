
/*********************************************************************
* Arduino feeder
* 
**********************************************************************/


//#ifndef FEEDER_CONFIG_H_
#define FEEDER_CONFIG_H_

#include <Servo.h>
#include <LiquidCrystal.h>
//#include <math.h>

// Firmware version
#define FIRMWARE_VER "0.0.1"

// Tasks frequencies
#define CONTROL_MOTOR_SPEED_FREQUENCY          50   //hz

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define MAX_REQ_VEL             25
#define REQ_VEL_STEP            1
#define MAX_SERVO_DELAY         2000
#define SERVO_DELAY_STEP        200

// HVLP parameters
#define HVLP_CLOSE_POS                   (uint32_t)120 // closed position angle (deg)
#define HVLP_OPEN_POS                    (uint32_t)80 // open position angle (deg) - corresponds to 180deg
#define HVLP_MAX                         (uint32_t)170
#define HVLP_MIN                         (uint32_t)30
#define HVLP_ANGLE_STEP                   5 

// Menu places
#define MENU_REQ_VELOCITY        0
#define MENU_OPEN_ANGLE          1
#define MENU_CLOSE_ANGLE         2
#define MENU_SERVO_DELAY         3
#define MENU_HVLP_EN             4
#define MENU_FEEDER_EN           5
#define MAX_MENU_PLACES          6

// Buttons
#define NO_BUTTON                0
#define RIGHT_BUTTON             1
#define LEFT_BUTTON              2
#define UP_BUTTON                3
#define DOWN_BUTTON              4
#define SELECT_BUTTON            5

// Feeder modes
//#define FEEDER_IDLE              0
//#define FEEDER_ON                1

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

#define MOTOR_ODOMETRY        2
#define MOTOR_PWMF            11
#define MOTOR_PWMR            3

#define GEAR_RATIO            60 //82
#define PPR                   5  //1
#define DIAMETER              85 //150
#define TICKS_FOR_ROTATION    (GEAR_RATIO*PPR)

// Function prototypes
float calculateVelocity(void);
void menu_handler_on(int velocity);
void print_idle_screen(void); 
void print_on_screen(void); 
void get_button(void);
void menu_handler(void);
void update_screen(int);
void motor_control(void);    

/*******************************************************************************
* Declaration for GUI & menu
*******************************************************************************/
//LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7); // initialize the library with the numbers of the interface pins
int menu_place_id = MENU_REQ_VELOCITY;
int current_button = NO_BUTTON;

bool is_pressed = false;            // remove this parameter !!!!!!!!!!!!!!!!!!!!!!

/*******************************************************************************
* Declaration for velocity ISR measurement
*******************************************************************************/
int odo_intr = 0;
int motor_rpm=0;
float dTime = 0;
unsigned long time_for_rotation;

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
bool is_feeder_active = false;      // Feeder starts inactive
int req_velocity = 20;              // required feeder velocity [cm/sec]
int open_angle = HVLP_OPEN_POS;     // HVLP active position
int close_angle = HVLP_CLOSE_POS;   // HVLP idle position
int servo_delay = 1000;             // wait 1000ms brfore activating the HVLP
bool is_feeder_enabled = true;      // Feeder is enabled 
bool is_hvlp_enabled = true;        // HVLP is enabled
