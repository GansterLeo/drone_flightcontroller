#ifndef DRONE_DATA_V5_1_H
#define DRONE_DATA_V5_1_H
#include <stdint.h>

// #define TUNING
#define DEBUGGING

// GPIO
#define IMU_CS              05

#define INIT_READY_LED_PIN  15  // Low active
#define ONESHOT_FL_PIN      16
#define ONESHOT_FR_PIN      17
#define ONESHOT_BL_PIN      18
#define ONESHOT_BR_PIN      19

#define SPIBUS_MISO         21
#define SPIBUS_SCK          22
#define SPIBUS_MOSI         23

#define BATTERY_VOLTAGE_PIN 33
// end GPIO

// defines
#define SERIAL_BAUDRATE 250000
#define MAX_MSSG_SIZE 100
#ifndef DEG_TO_RAD
#define DEG_TO_RAD (PI / 180)
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180 / PI)
#endif
// global variables
unsigned long prev_micros           = 0;
unsigned long current_time          = 0;
const float   dt                    = 500.0 / 1000000.0;  // default
bool          shutdown              = false;
// end global variables

// Filter parameters
//Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.05;  //Madgwick filter parameter
float B_accel = 0.2;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.17;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
// end Filter parameters

// IMU ERROR
#define AccErrorX  (0.00)
#define AccErrorY  (0.03)
#define AccErrorZ  (0.81)
#define GyroErrorX (-0.02)
#define GyroErrorY (0.02)
#define GyroErrorZ (0.02)

//PID controller 
//Controller parameters (take note of defaults before modifying!):
#define i_limit 25.0    //Integrator saturation level, mostly for safety (default 25.0)
#define MAX_ROLL 30.0   //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
#define MAX_PITCH 30.0  //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
#define MAX_YAW 80.0   //Max yaw rate in deg/sec

enum serialCommunicationERROR{
  SUCCESS = 0,
  VALUE_NOT_VALID = -1,
  ARGV_3RD_NOT_VALID = -2,
  ARGV_2ND_NOT_VALID = -3
};

enum axisName{
  roll,
  pitch,
  yaw,
  nOfAxisNames
};
enum motorName{
  frontLeft,
  frontRight,
  backLeft,
  backRight,
  nOfMotorNames   // has to be the last one, cause it indicates the number of values the enum has
};
float motor[nOfMotorNames];

typedef struct stAttitude{
  float desired, estimate, prev_est;
  float globalOffset;
} stAttitude;
stAttitude attitude[nOfAxisNames];


typedef struct imu_6DOF {
  float gx, gy, gz;
  float ax, ay, az;
} imu_6DOF;
imu_6DOF imu;

float rotationMtrx[nOfAxisNames][nOfAxisNames] = {
  {0.997436643, 0.069494411, 0.017052190},
  {-0.069071874, 0.997317433, -0.024229791},
  {-0.018690281, 0.022989852, 0.999561012}
};

typedef struct stPID {
  float Kp, Ki, Kd;
  float value;
} stPID;
stPID PID[nOfAxisNames];

// esp now
uint8_t controllerAddress[] = { 0xd4, 0x8a, 0xfc, 0x5f, 0xf0, 0x80 };
#ifdef TUNING
  typedef struct pid_tuning {
    float Kp, Ki, Kd;
  } pid_tuning;
#endif
// create struct of the data that is recieved by the drone
typedef struct data_controller_to_drone {
  float throttle;       // 0.0 -> 1.0
  float roll_pos_deg;   // desired roll position of the drone
  float pitch_pos_deg;  // desired pitch position of the drone
  float yaw_deg_per_s;  // desired rotation rate around the yaw axis in deg per second
  uint8_t error;        // 0 if no error accured
  uint8_t status;       // MSB = if TUNING is active
// these flexible variables have to be after the fixed variables, cause of error/status handling
#ifdef TUNING
  struct pid_tuning roll;
  struct pid_tuning pitch;
  struct pid_tuning yaw;
#endif
} data_controller_to_drone;

// create struct of the data that is send from the drone to the controller
typedef struct data_drone_to_controller {
  float roll_pos_deg;   // actual roll position of the drone
  float pitch_pos_deg;  // actual pitch position of the drone
  float yaw_pos_deg;    // actual yaw position of the drone
  uint8_t error;        // 0 if no error accured
  uint8_t status;       // MSB = if TUNING is active
  int16_t out[3];
  uint16_t fl, fr, bl, br;
} data_drone_to_controller;

#endif