
// How do I add another C file to an existing project?


/*******************************************************************************
* Calculate Velocity - optimized
*******************************************************************************/ 

float calculate_velocity_opt(float prev_velocity)
{ 
  static unsigned long prev_odo_ticks = 0;
  static unsigned long prev_odo_time_us = 0;
  static int print_counter = 0;
  unsigned long odo_ticks_diff = 0;
  unsigned long odo_time_diff_us = 0;
  double motor_velocity_double_mm;
  double motor_velocity_double_cm;
  int motor_velocity_10;
  float motor_velocity_float;
  
  // copy ISR values to avoid corruption - the following lines should not be seperated!
  unsigned long local_odo_ticks = odo_ticks;
  unsigned long local_odo_time_us = odo_time_us;   
  if (odo_ticks != local_odo_ticks)
  {
    // ISR has corrupted the data -> do not update velocity
    Serial.println("ISR - corrupted data");
    return prev_velocity;
  }  

  // calculate odo ticks
  if (local_odo_ticks>= prev_odo_ticks)
  {
    odo_ticks_diff = local_odo_ticks - prev_odo_ticks;    
  }
  else
  {
    // wrap around - update ticks & time and return
    Serial.println("ticks: wrap around");
    prev_odo_ticks = local_odo_ticks;
    prev_odo_time_us = local_odo_time_us;
    return prev_velocity; // to be handled .... 
  }

  if (local_odo_time_us>= prev_odo_time_us)
    odo_time_diff_us = local_odo_time_us - prev_odo_time_us;
  else
    // wrap around
    return prev_velocity; // to be handled ....   
  
  // Calculate velocity
  if (odo_ticks_diff == 0)
    // no new pulse
    if (odo_time_diff_us > ODO_NO_MOVEMENT)
      motor_velocity_float = 0.0;
    else
      motor_velocity_float = prev_velocity;
  else if (odo_ticks_diff<MIN_ODO_TICKS)
    // wait for more ticks
    motor_velocity_float = prev_velocity;
  else
  {
    // calculate new velocity
    //motor_velocity_double = (odo_ticks_diff/TICKS_FOR_ROTATION*3.1415*DIAMETER)/(odo_time_diff_us/1000000)
    motor_velocity_double_mm = double(1000000.0 * double(odo_ticks_diff)*3.1415*DIAMETER)/double(odo_time_diff_us)/double(TICKS_FOR_ROTATION)/10.0; // check why 10.0 .....
    motor_velocity_double_cm = motor_velocity_double_mm/10.0;
    //motor_velocity_10 = int(motor_velocity_double*10);
    motor_velocity_float = float(motor_velocity_double_cm);
  }

  print_counter++;
  if (print_counter>100)
  {
    print_counter = 0;
    Serial.print("ticks: ");
    Serial.println(odo_ticks_diff);
    Serial.print("time diff us= ");
    Serial.println(odo_time_diff_us);
    Serial.print("motor vel= ");
    Serial.println(motor_velocity_float);
  }
  
  // Update previous values
  prev_odo_ticks = local_odo_ticks;
  prev_odo_time_us = local_odo_time_us;
 
  return motor_velocity_float;

}


float calculateVelocity()
{ 

  float rpm_float;
  float motor_velocity_float;
  int motor_velocity_10;
  
  if (is_feeder_active)
  {
    //rpm_float = ((60000)/time_for_rotation)/GEAR_RATIO; // time_for_rotation in ms
    rpm_float = (float()/time_for_rotation)/float(GEAR_RATIO); // time_for_rotation in us    
    motor_velocity_float = ((rpm_float * 2 * 3.12415 * DIAMETER)/10)/60;      
    Serial.println(time_for_rotation);
    //Serial.print("float RPM= ");
    //Serial.println(rpm_float);
    motor_rpm = int(rpm_float); 
    return motor_velocity_float;
  }
  else
    return 0;
}

/*******************************************************************************
* Odometry ISR - time optimized 
*******************************************************************************/

void odo_ISR_opt()
{

  odo_ticks++;
  odo_time_us = micros();  //try to avoid functions !!
  delay(5); // check if delay needed!!
}



/*******************************************************************************
* Odometry ISR
* 
* 
*  Note - need to reduce the interrupts rate, as it kills the system at highr speeds !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
* 
*******************************************************************************/

void odo_ISR()
{
  static unsigned long prev_time = 0;
  static volatile int rotation; // variale for interrupt function - must be volatile!
  
  odo_intr++;delay(10); // reconsider the delay ..
  dTime = millis();  
  rotation++;
  if(rotation>=TICKS_FOR_ROTATION)
  {
    time_for_rotation = (micros()-prev_time); //time_for_rotation in millisec     
    prev_time = micros();
    rotation=0;
  }
}


/*void odo_ISR()
{
  static unsigned long prev_time = 0;
  static volatile int rotation; // variale for interrupt function - must be volatile!
  
  odo_intr++;delay(10);
  dTime = millis();  
  rotation++;
  if(rotation>=TICKS_FOR_ROTATION)
  {
    time_for_rotation = (micros()-prev_time); //time_for_rotation in millisec     
    prev_time = micros();
    rotation=0;

    //time_for_rotation = (millis()-prev_time); //time_for_rotation in millisec     
    //prev_time = millis();
    //rotation=0;
  }
}*/
