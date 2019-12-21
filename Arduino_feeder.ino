#include "Arduino_feeder.h" 

void setup() {

   // setup right wheel pins 
  //pinMode(MOTOR_ODOMETRY, INPUT);  // make pin 1 an input (odometry) 
  
  pinMode(MOTOR_ODOMETRY, INPUT_PULLUP); // Use this only for no gear motor !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
  //attachInterrupt(digitalPinToInterrupt(MOTOR_ODOMETRY), odo_ISR, RISING); 
  attachInterrupt(digitalPinToInterrupt(MOTOR_ODOMETRY), odo_ISR_opt, RISING); 
     
  pinMode(MOTOR_PWMF,OUTPUT); // make pin 11 an output (PWM forward)
  pinMode(MOTOR_PWMR,OUTPUT); // make pin 3 an output (PWM reverse)
       
  Serial.begin(9600);
  
  //set PWM frequency to 31KHz for pin 3 (and 11) - need to get the frequency below 25kHz !!!!!!!!!!!!!!!!!!!!!!!!!!!
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
  static float current_velocity = 0.0; 
    
  // Call RPM controller
  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {  
    // Poll motor odometry
    //current_velocity = calculateVelocity();
    current_velocity = calculate_velocity_opt(current_velocity);
            
    motor_control(req_velocity, int(current_velocity));    
    tTime[0] = t;
  }  

  // Handle the menu
  menu_handler();

  // Update the LCD
  update_screen(current_velocity);
   
}




/*******************************************************************************
* PID motor control 
*******************************************************************************/

void PID_motor_control(int required_velocity, float current_velocity, float P, float D, float I)
{

  unsigned long t=millis();
  static int prev_pwm = 0;
  static float prev_velocity = 0;
  static float prev_diff = 0;  
  static unsigned long prev_time=0;
  static int required_pwm=0;
  int delta_pwm = 0;
  float vel_diff;  
  float differntial = 0;
  float delta_P_pwm, delta_D_pwm, delta_I_pwm;  
   
  // P part
  delta_P_pwm = (required_velocity - current_velocity) * P;

  // D part
  delta_D_pwm = (current_velocity - prev_velocity)*1000*D/float(t-prev_time);

  // I part
  delta_I_pwm = 0.0;

  // Calculate new PWM
  delta_pwm = int(delta_P_pwm + delta_D_pwm);

  // Hard limit the max pwm change
  if (delta_pwm > MAX_PWM_CHANGE)
    delta_pwm = MAX_PWM_CHANGE;
  else if (delta_pwm < -MAX_PWM_CHANGE)
    delta_pwm = -MAX_PWM_CHANGE;

  // update the pwm
  required_pwm += delta_pwm;
         
  // Limit the actual pwm to be applied
  if (required_pwm>MAX_PWM)
    required_pwm = MAX_PWM;
  else if (required_pwm<MIN_PWM)
    required_pwm=MIN_PWM;

  // take into account the min PWM in which the wheels start to spin in the air!!!!!!!

  // Update the PWM
  analogWrite(MOTOR_PWMF,required_pwm); 
  //analogWrite(MOTOR_PWMF,120); 

    // Update parameters
  prev_time = t;
  prev_velocity = current_velocity; 


  // Add debug strings .....................
}

/*******************************************************************************
* motor control 
*******************************************************************************/

void motor_control(int required_velocity, int current_velocity)
{

  static int required_pwm = 0;
  int delta_pwm = 0;

  if (is_feeder_active)
  {
    // P controller
    delta_pwm = (required_velocity - current_velocity) * 1;

    // Limit max pwm change
    if (delta_pwm > 20)
      delta_pwm = 20;
    else if (delta_pwm < -20)
      delta_pwm = -20;

    // update the pwm
    required_pwm += delta_pwm;
         
  }
  else
  {
    // Force PWM = 0;
    required_pwm -= 10;    
  }

  // Limit the actual pwm to be applied
  if (required_pwm>254)
    required_pwm = 254;
  else if (required_pwm<0)
    required_pwm=0;

  // Update the PWM
  //analogWrite(MOTOR_PWMF,required_pwm); 
  if (is_feeder_active)
    analogWrite(MOTOR_PWMF,200);
  else
    analogWrite(MOTOR_PWMF,0);
     
}
