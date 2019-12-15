#include "Arduino_feeder.h" 

void setup() {

   // setup right wheel pins 
  //pinMode(MOTOR_ODOMETRY, INPUT);  // make pin 1 an input (odometry)  
  attachInterrupt(digitalPinToInterrupt(MOTOR_ODOMETRY), odo_ISR, RISING); 
     
  pinMode(MOTOR_PWMF,OUTPUT); // make pin 11 an output (PWM forward)
  pinMode(MOTOR_PWMR,OUTPUT); // make pin 3 an output (PWM reverse)
       
  Serial.begin(9600);
  
  //set PWM frequency to 31KHz for pin 3 (and 11)
  TCCR2B=(TCCR2B&0xF8) | 1; 

  // Setup motor PWM
  //analogWrite(MOTOR_PWMF,120); //do 50% PWM on pin 11 at the frequency set in TCCR2B(right wheel) 

  // Setup the LCD  
  lcd.begin(16, 2);
  
  Serial.begin(9600);
 
}

/*******************************************************************************
* Loop function
*******************************************************************************/

void loop() 
{
  uint32_t t = millis();
  float current_velocity = 0.0;

  // Poll motor odometry
  current_velocity = calculateVelocity();
  
  // Call RPM controller
  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {  
    motor_control();    
    tTime[0] = t;
  }  

  // Handle the menu
  menu_handler();

  // Update the LCD
  update_screen(current_velocity);
   
}


/*******************************************************************************
* Menu handler
* Buttons:
*           Right - scrolls right the menu
*           Left - scrolls left the menu
*           Up - increment current menu place
*           Down - decrement current menu place
*           Select - transiotion to On mode           
* Menu places: 
*           0=required velocity
*           1=Open angle
*           2=Close angle
*           3=servo_delay
*           4=HVLP enable
*           5=Feeder enable
*******************************************************************************/

void menu_handler()
{
  // poll the buttons
  get_button();

  if (current_button != NO_BUTTON)
  {
    // handle pressed button
    lcd.setCursor(0,1);
    lcd.print("               ");

    switch (current_button) {
      case RIGHT_BUTTON:
        // handle right button - update the menu place       
        menu_place_id = (menu_place_id+1) % MAX_MENU_PLACES;
        //lcd.print ("Right ");
        break;
      case LEFT_BUTTON:
        // Left button - update the menu place
        menu_place_id = (menu_place_id-1) % MAX_MENU_PLACES;
        if (menu_place_id<0)
          menu_place_id += MAX_MENU_PLACES;
        //lcd.print (menu_place_id);
        break;
      case UP_BUTTON:
        // Up button - increase/toggle the relevant menu place value
        switch (menu_place_id) {
          case MENU_REQ_VELOCITY:
            req_velocity = min(req_velocity+REQ_VEL_STEP, MAX_REQ_VEL);
            break;
          case MENU_OPEN_ANGLE:
            open_angle = min(open_angle+HVLP_ANGLE_STEP, HVLP_MAX);
            break;
          case MENU_CLOSE_ANGLE:
            close_angle = min(close_angle+HVLP_ANGLE_STEP, HVLP_MAX);
            break;
          case MENU_SERVO_DELAY:
            servo_delay = min(servo_delay+SERVO_DELAY_STEP,MAX_SERVO_DELAY);
            break;
          case MENU_HVLP_EN:
           is_hvlp_enabled = not(is_hvlp_enabled);
            break;
          case MENU_FEEDER_EN:
            is_feeder_enabled = not(is_feeder_enabled);
            break;          
          default:
            // statements
            break;
        }        
        break;
      case DOWN_BUTTON:
        // Down button - decrease/toggle the relevant menu place value  
        switch (menu_place_id) {
          case MENU_REQ_VELOCITY:
            req_velocity = max(req_velocity-REQ_VEL_STEP, 0);
            break;
          case MENU_OPEN_ANGLE:
            open_angle = max(open_angle-HVLP_ANGLE_STEP, HVLP_MIN);
            break;
          case MENU_CLOSE_ANGLE:
            close_angle = max(close_angle-HVLP_ANGLE_STEP, HVLP_MIN);
            break;
          case MENU_SERVO_DELAY:
            servo_delay = max(servo_delay-SERVO_DELAY_STEP,0);
            break;
          case MENU_HVLP_EN:
           is_hvlp_enabled = not(is_hvlp_enabled);
            break;
          case MENU_FEEDER_EN:
            is_feeder_enabled = not(is_feeder_enabled);
            break;       
          default:
            // statements
            break;
        }   
        break;
      case SELECT_BUTTON:
        // Select button - toggle feeder mode        
        is_feeder_active = not(is_feeder_active);             
        break;          
      default:
        // statements
        break;
    }

    // reset current button
    current_button = NO_BUTTON;    
         
  }
}

/*******************************************************************************
* Loop function
*******************************************************************************/

void update_screen(float velocity)
{
    // upper row
    lcd.setCursor(0,0);
    if (is_feeder_active)
    {
      // Active feeder display
      lcd.print ("Active ");      
      lcd.print (velocity);
      lcd.print (" cm/s");
    }
    else
    {
      // Inactive feeder display
      lcd.print ("Inactice      ");
    }
    
    // lower row
    lcd.setCursor(0,1);
    switch (menu_place_id) {
        case MENU_REQ_VELOCITY:
          lcd.print ("Vel = ");
          lcd.print (req_velocity);
          lcd.print (" cm/s");
          break;
        case MENU_OPEN_ANGLE:
          lcd.print ("Open = ");
          lcd.print (open_angle);
          lcd.print (" deg");
          break;
        case MENU_CLOSE_ANGLE:
          lcd.print ("Close = ");
          lcd.print (close_angle);
          lcd.print (" deg");
          break;
        case MENU_SERVO_DELAY:
          lcd.print ("Delay = ");
          lcd.print (servo_delay);
          lcd.print (" ms");
          break;
        case MENU_HVLP_EN:
          if (is_hvlp_enabled)
            lcd.print("HVLP enabled");
           else
            lcd.print("HVLP disabled");
          break;
        case MENU_FEEDER_EN:
          if (is_feeder_enabled)
            lcd.print("Feeder enabled");
           else
            lcd.print("Feeder disabled");
          break;       
        default:
          // statements
          break;
      }  
}


/*******************************************************************************
* Get button
*   Indicates if a new button was pressed (right after the button was released)
*   The using function is expected to set current_button to NO_BUTTON
*******************************************************************************/
void get_button() 
{
  
  static bool is_pressed_internal = false;
  static int last_button = NO_BUTTON;
  int x;

  // sample button analog pin
  x = analogRead (0); 

  if (is_pressed_internal)
  {
    // Wait for button release
    if (x>=800)
    {
      // button release - update current button
      current_button = last_button;
      last_button = NO_BUTTON;
      is_pressed_internal = false;     
    }
  }
  else // check if new button is pressed
  {     
    if (x<100)
      // Right button
      last_button = RIGHT_BUTTON;
    else if (x < 200)
      // Up button 
      last_button = UP_BUTTON;
    else if (x<400)
      // Down button
      last_button = DOWN_BUTTON;
    else if (x<600)
      // Left button
      last_button = LEFT_BUTTON;
    else if (x<800)
      // Select button
      last_button = SELECT_BUTTON;

     // indicate a button is pressed
     if (last_button > 0)
      is_pressed_internal = true;
  }
}

void motor_control()
{

  if (is_feeder_active)
  {
    // Setup motor PWM
    analogWrite(MOTOR_PWMF,120); 
  }
  else
  {
    // Setup motor PWM
    analogWrite(MOTOR_PWMF,0); 
  }
}

/*******************************************************************************
* Odometry ISR
*******************************************************************************/

void odo_ISR()
{
  static unsigned long prev_time = 0;
  static volatile int rotation; // variale for interrupt function - must be volatile!
  
  odo_intr++;delay(10);
  dTime = millis(); 
  rotation++;
  if(rotation>=TICKS_FOR_ROTATION)
  {
    time_for_rotation = (millis()-prev_time); //time_for_rotation in millisec     
    prev_time = millis();
    rotation=0;
  }
}

/*******************************************************************************
* Left Odometry ISR
*******************************************************************************/

/*void Left_ISR()
{
  left_intr++;delay(10);
}*/

/*******************************************************************************
* Calculate Velocity
*******************************************************************************/ 

float calculateVelocity()
{ 

  float rpm_float;
  float motor_velocity_float;
  int motor_velocity_10;
  
  if (is_feeder_active)
  {
    rpm_float = ((60000)/time_for_rotation)/GEAR_RATIO;
    motor_rpm = int(rpm_float);
    motor_velocity_float = ((rpm_float * 2 * 3.12415 * DIAMETER)/10)/60;   
    motor_velocity_10 = int(motor_velocity_float*10);
    motor_velocity_float = float(motor_velocity_10)/10;
    //Serial.println(time_for_rotation);
    //Serial.print("RPM= ");
    //Serial.println(motor_rpm);
    return motor_velocity_float;
  }
  else
    return 0;
}

/*
float calculateVelocity()
{  

  unsigned long current_time;
  unsigned int time_diff;
  int odometer_val = 0;
  int static prev_odometer_val = 0;
  int static motor_odo_counter = 0;
  float static motor_prev_time =0; 
  
  // Poll the motor odometry pin 
  odometer_val= digitalRead(MOTOR_ODOMETRY);
  current_time = millis();
  
  // Detect tranbsition and update odometry counter
  if(odometer_val==0 && prev_odometer_val== 1 )  //the point where high goes to low(end of the pulse) 
  {
    // Count a new pulse (signal transition from 1 to 0)
    motor_odo_counter = motor_odo_counter + 1;
    Serial.println(motor_odo_counter);
    lcd.setCursor(10,0);   
  }
  // Update prev motor pulses
  prev_odometer_val=odometer_val;  
  
  if(motor_odo_counter>=2)
  {    
    // Calculate RPM & speed
    time_diff = int(current_time-motor_prev_time); //time difference in millisec 
    motor_rpm = int(long(60*1000)/time_diff); 
    motor_velocity = (float(motor_rpm)*150*3.1415)/60; 

    // Init for next time
    motor_prev_time = current_time;
    motor_odo_counter=0;

    Serial.print("time diff: ");
    Serial.println(time_diff);    
    
    
    Serial.print("rpm: ");
    Serial.println(motor_rpm);    
    Serial.print("mm/s of right wheel:");
    Serial.println(motor_velocity);
    
  } 

  return motor_velocity;
}*/

/*int calculateDistance(int pulses)
{
 return int(pulses*PULSES_2_MM);
}*/
