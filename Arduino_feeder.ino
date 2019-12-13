#include "Arduino_feeder.h" 

void setup() {

  char my_str[16] = "Motor Vel =    "; // starts at 0

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
  // Print a message to the LCD.  
  lcd.setCursor(0,0);
  my_str[12] = '0';
  my_str[13] = '.';
  my_str[14] = '0';
  //lcd.print("Motor Vel = 0"); 
  lcd.print(my_str); 
  lcd.setCursor(0,1);
  lcd.print("Set Req vel:");
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

  // Poll buttons
  menu_handler();
    
  // Call RPM controller
  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {      
    tTime[0] = t;
  }  
}

/*******************************************************************************
* Menu handler
* Buttons:
*           Up - scrolls up the menu
*           Down - scrolls down the menu
*           Right - increment current menu place
*           Left - decrement current menu place
*           Select - Start/Stop*           
* Menu places: 
*           0=required velocity
*           1=Open angle
*           2=Close angle
*           3=servo_delay
*******************************************************************************/

void menu_handler()
{
  int x;
  x = analogRead (0);
  lcd.setCursor(10,1);

  if (x>=800)
  {
    // No button is pressed
    is_pressed = false;
  }
  else
  {
    if (is_pressed)
    {
      // wait for the button to be released
    }
    else
    {
      is_pressed = true;
      Serial.print("The value at pim A0 is  :");
      Serial.println(x,DEC);      
      if (x < 100) {
        lcd.print ("Right ");  
      }
      else if (x < 200) {
        // Up button
        menu_place_id = (menu_place_id+1) % MAX_MENU_PLACES;
        lcd.print (menu_place_id);
      }
      
      else if (x < 400){
        // Down button
        menu_place_id = (menu_place_id-1) % MAX_MENU_PLACES;
        if (menu_place_id<0)
          menu_place_id += MAX_MENU_PLACES;
        lcd.print (menu_place_id);
      }
      else if (x < 600){
        lcd.print ("Left  ");
      }
      else if (x < 800){
        lcd.print ("Select");
      }
    }    
  }
  
 
  /*
  if (x < 100) 
  {
    lcd.print ("Right ");
    Serial.print("The value at pim A0 Right key pressed is  :");
    Serial.println(x,DEC);
    is_pressed = true;
  }
  else if (x < 200) {
    // Up button
    menu_place_id = (menu_place_id+1) % MAX_MENU_PLACES;
    lcd.print (menu_place_id);
    //lcd.print ("Up    ");
    Serial.print("The value at pim A0 UP key pressed is  :");
    Serial.println(x,DEC);
    is_pressed = true;
  }
  
  else if (x < 400){
    // Down button
    menu_place_id = (menu_place_id-1) % MAX_MENU_PLACES;
    lcd.print (menu_place_id);
    //lcd.print ("Down  ");
    Serial.print("The value at pim A0 Down key pressed is  :");
    Serial.println(x,DEC);
    is_pressed = true;
  }
  else if (x < 600){
    lcd.print ("Left  ");
    Serial.print("The value at pim A0 Left key pressed is  :");
    Serial.println(x,DEC);
    is_pressed = true;
  }
  else if (x < 800){
    lcd.print ("Select");
    Serial.print("The value at pim A0 Select key pressed is  :");
    Serial.println(x,DEC);
    is_pressed = true;
  }
  else {
    is_pressed = false;
  }
  */
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
