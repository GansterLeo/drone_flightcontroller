#ifndef DRONE_DATA_V5_1_H
#define DRONE_DATA_V5_1_H


#define TUNING
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
#define DEG_TO_RAD (PI / 180)

// global variables
unsigned long prev_micros           = 0;
unsigned long current_time          = 0;
const float dt                      = 500.0 / 1000000.0;  // default
bool shutdown                       = false;
// end global variables

// Filter parameters
//Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.05;  //Madgwick filter parameter
float B_accel = 0.2;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.17;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)

// end Filter parameters

// IMU ERROR
#define AccErrorX  (-0.14)
#define AccErrorY   (0.11)
#define AccErrorZ   (9.41)
#define GyroErrorX  (0.03)
#define GyroErrorY  (0.03)
#define GyroErrorZ  (0.03)

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
stAttitude attitudeIMUFrame[nOfAxisNames];

typedef struct imu_6DOF {
  float gx, gy, gz;
  float ax, ay, az;
} imu_6DOF;
imu_6DOF imu;

float rotationMtrxCorrect[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};

typedef struct stPID {
  float Kp, Ki, Kd;
  float value;
} stPID;
stPID PID[nOfAxisNames];


#endif