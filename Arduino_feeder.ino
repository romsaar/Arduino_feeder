#include "Arduino_feeder.h" 

void setup() {

   // setup right wheel pins 
  pinMode(MOTOR_ODOMETRY, INPUT);  // make pin 1 an input (odometry)  
  //digitalWrite(MOTOR_ODOMETRY, INPUT_PULLDOWN); // Configure odometry as pull_down
  pinMode(MOTOR_PWMF,OUTPUT); // make pin 11 an output (PWM forward)
  pinMode(MOTOR_PWMR,OUTPUT); // make pin 3 an output (PWM reverse)
     
  Serial.begin(9600);
  
  //set PWM frequency to 3906 Hz for pin 3 (and 11)
  TCCR2B=(TCCR2B&0xF8) | 1; 

  // Setup motor PWM
  analogWrite(MOTOR_PWMF,120); //do 50% PWM on pin 11 at the frequency set in TCCR2B(right wheel) 

  // Setup the LCD  
  lcd.begin(16, 2);
  
  // Print a message to the LCD 
  print_idle_screen(); 
  
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
    tTime[0] = t;
  }  

  // Handle GUI
  menu_handler();
  
  /*if (feeder_mode == FEEDER_IDLE)
    menu_handler_idle();
  else
    menu_handler_on(int(current_velocity));     */
  
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
        // Select button - start/stop        
        feeder_mode = FEEDER_ON;        
        // On mode display 
        lcd.setCursor(0,1);
        lcd.print ("Start!");        
        break;          
      default:
        // statements
        break;
    }

    // reset current button
    current_button = NO_BUTTON;    

    // Handle the new display
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
}


void menu_handler_on(int velocity)
{
  
  //char my_str[16] = "Motor Vel =    "; // starts at 0  
  int x;

  //my_str[12] = '0';
  //my_str[13] = '.';
  //my_str[14] = '0';
  
  // Read buttons
  x = analogRead (0);

  // LCD
  lcd.setCursor(0,0);
  lcd.print ("Feeder On     ");
  lcd.setCursor(0,1);
  lcd.print ("Vel = ");
  lcd.print (velocity);
  lcd.print (" cm/s");
    
  if (x < 800)
  {
      // One of the buttons pressed  
      is_pressed = true; 
  }

  if ((is_pressed) && (x>=800))
  {
    //button released - return to idle
    feeder_mode = FEEDER_IDLE;
    print_idle_screen();
   }  
}

/*******************************************************************************
* print idle screen
*******************************************************************************/

void print_idle_screen()
{  
  lcd.setCursor(0,0); 
  lcd.print("Setup params:");
  lcd.setCursor(0,1);
  lcd.print ("Vel = ");
  lcd.print (req_velocity);
  lcd.print (" cm/s");
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


/*******************************************************************************
* Calculate Velocity
*******************************************************************************/ 

float calculateVelocity()
{  

  float current_time;
  float time_diff; 
  
  // Poll the motor odometry pin 
  motor_pulses= digitalRead(MOTOR_ODOMETRY);
  current_time = millis();
  
  // Detect tranbsition and update odometry counter
  if(motor_pulses==0 && prev_motor_pulses== 1 )  //the point where high goes to low(end of the pulse) 
  {
    motor_odo_counter = motor_odo_counter + 1;
    //Serial.print(motor_odo_counter);
  }
  // Update prev motor pulses
  prev_motor_pulses=motor_pulses;

  
  
  if(motor_odo_counter>=82)
  {    
    // Calculate RPM & speed
    time_diff = current_time-motor_prev_time; //time difference in millisec 
    motor_rpm = (1000/time_diff)*60; 
    motor_velocity = (motor_rpm/60)*150*3.1415; 

    // Init for next time
    motor_prev_time = current_time;
    motor_odo_counter=0;
    
    /*Serial.print("rpm: ");
    Serial.println(motor_rpm);    
    Serial.print("mm/s of right wheel:");
    Serial.println(motor_velocity);
    */
  } 

  return motor_velocity;
}

/*int calculateDistance(int pulses)
{
 return int(pulses*PULSES_2_MM);
}*/
