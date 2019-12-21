
// 1:60 motor parameters
#define MOTOR_ODOMETRY        2 // 2 = 1:60 motor
#define MOTOR_PWMF            11
#define MOTOR_PWMR            3

#define GEAR_RATIO            60 //for 1:60 motor//82
#define PPR                   5 //for 1:60 motor
#define DIAMETER              85 //150     //[mm]
#define TICKS_FOR_ROTATION    (GEAR_RATIO*PPR)

// No gear motor
/*
#define MOTOR_ODOMETRY        2
#define MOTOR_PWMF            11
#define MOTOR_PWMR            3

#define GEAR_RATIO            1
#define PPR                   1
#define DIAMETER              12
#define TICKS_FOR_ROTATION    (GEAR_RATIO*PPR)*/

// Odometry constants
#define ODO_NO_MOVEMENT       (500.0*1000.0) // if no new pulse within the last 500ms -> velocity is zero
#define MIN_ODO_TICKS         5
