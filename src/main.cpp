// #define CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_SILENT

#include "header.h"
#include "MPU6500.h"
#include "driver/ledc.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>  // required for the esp_error_check()

// OneShot125 timer
  #define ESC_OFFSET (4096 * 0.14)
  #define ONESHOT_TIMER LEDC_TIMER_0
  #define LEDC_MODE LEDC_LOW_SPEED_MODE
  #define ONESHOT0_CHANNEL LEDC_CHANNEL_0
  #define ONESHOT1_CHANNEL LEDC_CHANNEL_1
  #define ONESHOT2_CHANNEL LEDC_CHANNEL_2
  #define ONESHOT3_CHANNEL LEDC_CHANNEL_3
  #define ONESHOT_DUTY_RES LEDC_TIMER_14_BIT
  #define ONESHOT_DUTY (4096)       // set duty to 25%
  #define ONESHOT_FREQUENCY (2000)  // 2kHz
// end OneShot125 timer

// Serial communciation
  #define MAX_MSSG_SIZE 100
  char message[MAX_MSSG_SIZE] = "";
// end Serial communication

// SPI begin
  SPIClass spiBus(VSPI);
// end SPI

// IMU
  // MPU6050 scaling constants (without external library)
  // Gyro full-scale selection 
  #define GYRO_RANGE_250DPS     MPU6500::GYRO_RANGE_250DPS
  // Accel full-scale selection 
  #define ACCEL_RANGE_2G        MPU6500::ACCEL_RANGE_2G  // Ensure enum scope is resolved
  // Digital low pass filter selection
  #define DLPF_BANDWIDTH_184HZ  MPU6500::DLPF_BANDWIDTH_184HZ
  // IMU Object
  MPU6500 mpu(spiBus, IMU_CS);
// end IMU

// controller
//Controller parameters (take note of defaults before modifying!):
#define i_limit 25.0    //Integrator saturation level, mostly for safety (default 25.0)
#define MAX_ROLL 30.0   //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
#define MAX_PITCH 30.0  //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
#define MAX_YAW 160.0   //Max yaw rate in deg/sec
// end controller

// esp now begin
  #ifdef TUNING
  typedef struct pid_tuning {
    float Kp, Ki, Kd;
  } pid_tuning;
  #endif

  uint8_t controllerAddress[] = { 0xd4, 0x8a, 0xfc, 0x5f, 0xf0, 0x80 };
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
  data_controller_to_drone incomingReadings;  // create the struct for incoming data

  // create struct of the data that is send from the drone to the controller
  typedef struct data_drone_to_controller {
    float roll_pos_deg;   // actual roll position of the drone
    float pitch_pos_deg;  // actual pitch position of the drone
    float yaw_pos_deg;    // actual yaw position of the drone
    uint8_t error;        // 0 if no error accured
    uint8_t status;       // MSB = if TUNING is active
  #ifdef TUNING
    int16_t out[3];
    uint16_t fl, fr, bl, br;
  #endif
  } data_drone_to_controller;
  data_drone_to_controller sendingData;  // create struct for sending data

  esp_now_peer_info_t peerInfo;
// esp now end

// function declaration 
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
void controlANGLE(void);
void oneshot125_init(void);
void oneshot125_write(const float fl_vl, const float fr_vl, const float bl_vl, const float br_vl);
uint8_t getIMUdata(void);
void loopRate(uint16_t freq);
float invSqrt(float x);
void checkBattery(void);
void init_wlcomm(void);
void calibrateAttitude();
void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, const int len);
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
void send_data();
int8_t analyzeMessage(char* pMessage);
void serialCommunication();
void printDelay(uint16_t dt);

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  pinMode(INIT_READY_LED_PIN, OUTPUT);
  digitalWrite(INIT_READY_LED_PIN, LOW);

  Serial.printf("Hello\n");
  PID[roll].Kp = 0.2;
  PID[roll].Ki = 0.3;
  PID[roll].Kd = 0.05;
  
  PID[pitch].Kp = 0.2;
  PID[pitch].Ki = 0.3;
  PID[pitch].Kd = 0.05;


  PID[yaw].Kp = 0.3;
  PID[yaw].Ki = 0.05;
  PID[yaw].Kd = 0.00015;
  // put your setup code here, to run once:
  //checkBattery();
  spiBus.begin(SPIBUS_SCK, SPIBUS_MISO, SPIBUS_MOSI, IMU_CS);                         // SCK, MISO, MOSI, SS
  Serial.printf("Now IMU init\n");
  Serial.printf("01 MPU.begin status: %d\n",                 mpu.begin());
  Serial.printf("02 setAccelRange status: %d\n",             mpu.setAccelRange(ACCEL_RANGE_2G));
  Serial.printf("03 setGyroRange status: %d\n",              mpu.setGyroRange(GYRO_RANGE_250DPS));
  Serial.printf("04 setDlpfBandwidth status: %d\n",          mpu.setDlpfBandwidth(DLPF_BANDWIDTH_184HZ));
  Serial.printf("05 set Data Output Rate status: %d\n",      mpu.setSrd(0)); // use default 1kHz 
  Serial.printf("06 diabled data ready interrupt: %d\n",     mpu.disableDataReadyInterrupt());

  Serial.printf("IMU init done, now oneshot125init\n");
  delay(3);
  oneshot125_init();
  oneshot125_write(0.0f, 0.0f, 0.0f, 0.0f);

  Serial.printf("Oneshot125 init done, now esp now init\n");
  init_wlcomm();

  // !!! comment out after errors are gethered
  // calculate_IMU_error();
  Serial.printf("calibrating attitudeIMUFrame\n");
  calibrateAttitude();

  Serial.printf("Waiting 5s for motors to be initialised\n");
  delay(5000);
  Serial.printf("!!! testing motors !!!\n");
  // test motors
  // motors tart at 0.14
  for (uint8_t k = 0; k < 0.1; k++) {
    for (float i = 0.0f; i < .1f; i += 0.005f) {
      delay(50);
      oneshot125_write(i, i, i, i);
      Serial.println(i);
    }
    for (float i = 0.1f; i > 0.0f; i -= 0.005f) {
      delay(50);
      oneshot125_write(i, i, i, i);
      Serial.println(i);
    }
  }
  oneshot125_write(0.0f, 0.0f, 0.0f, 0.0f);
  // end test motors

  Serial.printf("Initialisation done!\n");

  digitalWrite(INIT_READY_LED_PIN, HIGH);
  prev_micros = micros();
}

void loop() {
  // current_time = micros();
  // dt = (current_time - prev_micros)/1000000.0;
  prev_micros = micros();

  //checkBattery();
  
  if(getIMUdata() == 1){
    //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
    Serial.printf("getIMUdata failed\n");
  }
  else Madgwick6DOF(imu.gx, -imu.gy, -imu.gz, -imu.ax, imu.ay, imu.az, dt);  //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
 
  controlANGLE();

  // control mixer
  motor[frontLeft]   = incomingReadings.throttle - PID[pitch].value + PID[roll].value + PID[yaw].value;  //Front Left
  motor[frontRight]  = incomingReadings.throttle - PID[pitch].value - PID[roll].value - PID[yaw].value;  //Front Right
  motor[backLeft]    = incomingReadings.throttle + PID[pitch].value - PID[roll].value + PID[yaw].value;  //Back Right
  motor[backRight]   = incomingReadings.throttle + PID[pitch].value + PID[roll].value - PID[yaw].value;  //Back Left

  oneshot125_write(motor[frontLeft], motor[frontRight], motor[backLeft], motor[backRight]);

  serialCommunication();

  send_data();

  // Serial.printf("p:%05.2f;r:%05.2f;y:%05.2f\n", attitudeIMUFrame[pitch].estimate, attitudeIMUFrame[roll].estimate, attitudeIMUFrame[yaw].estimate);
  // Serial.println(dt*1000000);
  // Serial.println(micros() - prev_micros);
  printDelay(micros() - prev_micros);
  loopRate(2000);
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */

  // static variables
  static float q0 = 1.0f;
  static float q1, q2, q3 = 0.0f;

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= DEG_TO_RAD;
  gy *= DEG_TO_RAD;
  gz *= DEG_TO_RAD;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  attitudeIMUFrame[roll].estimate   = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29577951;                   //degrees
  attitudeIMUFrame[pitch].estimate  = -asin(constrain(-2.0f * (q1 * q3 - q0 * q2), -0.999999, 0.999999)) * 57.29577951;  //degrees
  attitudeIMUFrame[yaw].estimate    = -atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29577951;                   //degrees

}

void controlANGLE(void) {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */

  // definition static integrals
  static float integral_roll = 0;
  static float integral_pitch = 0;
  static float integral_yaw = 0;

  if (incomingReadings.throttle < 0.06) {  //Don't let integrator build if throttle is too low (<6%)
    integral_roll = 0;
    integral_pitch = 0;
    integral_yaw = 0;
  }

  //Roll
  float error_roll = attitudeIMUFrame[roll].desired - (attitudeIMUFrame[roll].estimate);
  integral_roll += error_roll * dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup

  PID[roll].value = 0.01 * ((PID[roll].Kp * error_roll) + (PID[roll].Ki * integral_roll) - (PID[roll].Kd * imu.gx));  //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  float error_pitch = attitudeIMUFrame[pitch].desired - (attitudeIMUFrame[pitch].estimate);
  integral_pitch += error_pitch * dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup

  PID[pitch].value = .01 * ((PID[pitch].Kp * error_pitch) + (PID[pitch].Ki * integral_pitch) - (PID[pitch].Kd * imu.gy));  //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  static float error_yaw_prev = 0;
  float error_yaw = attitudeIMUFrame[yaw].desired - imu.gz;
  integral_yaw += error_yaw * dt;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev) / dt;

  PID[yaw].value = .01 * ((PID[yaw].Kp * error_yaw) + (PID[yaw].Ki * integral_yaw) + (PID[yaw].Kd * derivative_yaw));  //Scaled by .01 to bring within -1 to 1 range

  // update prev variables
  error_yaw_prev = error_yaw;
}

void oneshot125_init(void) {
  // Prepare and then apply the LEDC PWM timer configuration
  ledc_timer_config_t oneshot_timer = {
    .speed_mode = LEDC_MODE,
    .duty_resolution = ONESHOT_DUTY_RES,
    .timer_num = ONESHOT_TIMER,
    .freq_hz = ONESHOT_FREQUENCY,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&oneshot_timer);

  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t oneshot_channel_0 = {
    .gpio_num = ONESHOT_FL_PIN,
    .speed_mode = LEDC_MODE,
    .channel = ONESHOT0_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = ONESHOT_TIMER,
    .duty = ONESHOT_DUTY,  // Set duty to 25%
    .hpoint = 0
  };
  ledc_channel_config_t oneshot_channel_1 = {
    .gpio_num = ONESHOT_FR_PIN,
    .speed_mode = LEDC_MODE,
    .channel = ONESHOT1_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = ONESHOT_TIMER,
    .duty = ONESHOT_DUTY,  // Set duty to 25%
    .hpoint = 0
  };
  ledc_channel_config_t oneshot_channel_2 = {
    .gpio_num = ONESHOT_BL_PIN,
    .speed_mode = LEDC_MODE,
    .channel = ONESHOT2_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = ONESHOT_TIMER,
    .duty = ONESHOT_DUTY,  // Set duty to 25%
    .hpoint = 0
  };
  ledc_channel_config_t oneshot_channel_3 = {
    .gpio_num = ONESHOT_BR_PIN,
    .speed_mode = LEDC_MODE,
    .channel = ONESHOT3_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = ONESHOT_TIMER,
    .duty = ONESHOT_DUTY,  // Set duty to 25%
    .hpoint = 0
  };

  ledc_channel_config(&oneshot_channel_0);
  ledc_channel_config(&oneshot_channel_1);
  ledc_channel_config(&oneshot_channel_2);
  ledc_channel_config(&oneshot_channel_3);
}

/*
  updates duty cycle of all 4 oneshot outputs
  duty cycle values need to be between 0 and 1
  fl_vl ... FL_PIN
  fr_vl ... FR_PIN
  bl_vl ... BL_PIN
  br_vl ... BR_PIN
  minimum is 25% duty cycle
  maximum is 50% duty cycle
*/
void oneshot125_write(const float fl_vl, const float fr_vl, const float bl_vl, const float br_vl) {
  ledc_set_duty(LEDC_MODE, ONESHOT0_CHANNEL, ((constrain(fl_vl, 0.0, 1.0) * 4096) + 4096 + ESC_OFFSET));
  ledc_set_duty(LEDC_MODE, ONESHOT1_CHANNEL, ((constrain(fr_vl, 0.0, 1.0) * 4096) + 4096 + ESC_OFFSET));
  ledc_set_duty(LEDC_MODE, ONESHOT2_CHANNEL, ((constrain(bl_vl, 0.0, 1.0) * 4096) + 4096 + ESC_OFFSET));
  ledc_set_duty(LEDC_MODE, ONESHOT3_CHANNEL, ((constrain(br_vl, 0.0, 1.0) * 4096) + 4096 + ESC_OFFSET));
  ledc_update_duty(LEDC_MODE, ONESHOT0_CHANNEL); 
  ledc_update_duty(LEDC_MODE, ONESHOT1_CHANNEL);
  ledc_update_duty(LEDC_MODE, ONESHOT2_CHANNEL);
  ledc_update_duty(LEDC_MODE, ONESHOT3_CHANNEL);
}

//DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
  /*
   * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. 
   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
   * the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
   * 
   * return value is 0 if everthing went alright
   *                 1 if the imu data reading failed
   */
uint8_t getIMUdata(void) {
  // define acc and gyro_prev variables here
  static float axPrev, ayPrev = 0;
  static float azPrev = 9.81;
  static float gxPrev, gyPrev, gzPrev = 0;
  uint8_t returnValue = 0;

  mpu.readSensor();
  
  //Accelerometer and correct the outputs with the calculated error values
  imu.ax = (mpu.getAccelX_mss()) - AccErrorX;  //G's
  imu.ay = (mpu.getAccelY_mss()) - AccErrorY;
  imu.az = (mpu.getAccelZ_mss()) - AccErrorZ;

  //LP filter accelerometer data
  imu.ax = ((1.0 - B_accel) * axPrev) + (B_accel * imu.ax);
  imu.ay = ((1.0 - B_accel) * ayPrev) + (B_accel * imu.ay);
  imu.az = ((1.0 - B_accel) * azPrev) + (B_accel * imu.az);
  axPrev = imu.ax;
  ayPrev = imu.ay;
  azPrev = imu.az;

  //Gyro and correct the outputs with the calculated error values
  imu.gx = (mpu.getGyroX_rads() * (180 / PI)) - GyroErrorX;  //deg/sec
  imu.gy = (mpu.getGyroY_rads() * (180 / PI)) - GyroErrorY;
  imu.gz = (mpu.getGyroZ_rads() * (180 / PI)) - GyroErrorZ;

  //LP filter gyro data
  imu.gx = ((1.0 - B_gyro) * gxPrev) + (B_gyro * imu.gx);
  imu.gy = ((1.0 - B_gyro) * gyPrev) + (B_gyro * imu.gy);
  imu.gz = ((1.0 - B_gyro) * gzPrev) + (B_gyro * imu.gz);
  gxPrev = imu.gx;
  gyPrev = imu.gy;
  gzPrev = imu.gz;

  return 0;
}

/*
void calculate_IMU_error(void){
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  
  // Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
  // accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
  // measurement. 

  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY= 0.0;
  GyroErrorZ = 0.0;
  
  //Read IMU values 12000 times
  uint16_t c = 0;
  while(c < 12000){
    mpu.readSensor();    
    //Sum all readings
    AccErrorX  += mpu.getAccelX_mss();
    AccErrorY  += mpu.getAccelY_mss();
    AccErrorZ  += mpu.getAccelZ_mss();
    GyroErrorX += mpu.getGyroX_rads() * (180 / PI);
    GyroErrorY += mpu.getGyroX_rads() * (180 / PI);
    GyroErrorZ += mpu.getGyroX_rads() * (180 / PI);
    
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = (AccErrorZ / c) - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");
  
  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}
*/

void loopRate(uint16_t freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = (1.0 / freq) * 1000000.0;
  unsigned long checker = micros();

  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - prev_micros)) {
    checker = micros();
  }
}

float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter

  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;

  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  // return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}

/*
void checkBattery(void){
  static uint16_t batVoltage = 4095;
  batVoltage = (uint16_t)((.8*batVoltage) + (.2*analogRead(BATTERY_VOLTAGE_PIN)));
  if(batVoltage < 3100){
    shutdown = true;
    sendingData.error = 1;
    sendingData.status |= (STAT_CRITICAL_LOW_VOLT | STAT_HALF_CAPACITY);
  } else if(batVoltage < 3443){
    sendingData.status |= STAT_CRITICAL_LOW_VOLT;
    sendingData.status &= ~STAT_HALF_CAPACITY;
  } else if(batVoltage < 3583){
    sendingData.status &= ~STAT_CRITICAL_LOW_VOLT;
    sendingData.status |= STAT_HALF_CAPACITY;
  } else{
    sendingData.status &= ~(STAT_CRITICAL_LOW_VOLT | STAT_HALF_CAPACITY);
  }
}
*/

/*
  this function initialises esp_now long range
*/
void init_wlcomm(void) {
  // esp_now begin
  esp_netif_init();
  // ESP_ERROR_CHECK(esp_event_loop_create_default());
  // https://www.esp32.com/viewtopic.php?t=24063 explains the problem
  switch (esp_event_loop_create_default()) {
    case ESP_OK:
    case ESP_ERR_INVALID_STATE:
      break;
    default:
      #ifdef DEBUGGING
            Serial.print("Failed to create event loop\n");
      #endif
            while (1);
            break;
        }
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_wifi_init(&cfg);
        esp_wifi_set_storage(WIFI_STORAGE_RAM);
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();
        esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

        if (esp_now_init() != ESP_OK) {
      #ifdef DEBUGGING
          Serial.println("Error initializing ESP-NOW");
      #endif
          while (true) {
          };
        }

  // determine interrupt function, that gets call, if data is recieved
  esp_now_register_recv_cb(onDataRecv);
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, controllerAddress, 6);
  peerInfo.channel = 0;  // Use default channel
  peerInfo.encrypt = false;
  #ifdef DEBUGGING
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
    }
  #else
    esp_now_add_peer(&peerInfo);
  #endif
  // end esp now
}

void calibrateAttitude() {
  //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  //Assuming vehicle is powered up on level surface!
  /*
   * This function is used on startup to warm up the attitudeIMUFrame estimation and is what causes startup to take a few seconds
   * to boot. 
   */
  //Warm up IMU and madgwick filter in simulated main loop
  for (int i = 0; i <= 10000; i++) {
    // unsigned long prev_time = current_time;
    // current_time = micros();
    // dt = (current_time - prev_time)/1000000.0;
    if(getIMUdata() == 0) {
      Madgwick6DOF(imu.gx, -imu.gy, -imu.gz, -imu.ax, imu.ay, imu.az, dt);
    }
    else{
      Serial.println("SPI failed!");
    }
    loopRate(2000);  //do not exceed 2000Hz
  }
}

// gets called, if esp recieves data
// needs between 4 to 10us
void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, const int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  if ((incomingReadings.status & (1 << 7)) != (sendingData.status & (1 << 7))) {
    sendingData.error = 1;  // two different modes
    shutdown = true;
  }
  incomingReadings.throttle = constrain(incomingReadings.throttle, 0.0, 1.0);
  incomingReadings.roll_pos_deg = constrain(incomingReadings.roll_pos_deg, -(MAX_ROLL), MAX_ROLL);
  incomingReadings.pitch_pos_deg = constrain(incomingReadings.pitch_pos_deg, -(MAX_PITCH), MAX_PITCH);
  incomingReadings.yaw_deg_per_s = constrain(incomingReadings.yaw_deg_per_s, -(MAX_YAW), MAX_YAW);
  // !!!!
  // convert to rad
  // !!!!

  // timerWrite(timerEStop, (TIMER_ALARMVAL_us - TIMER_CONNECTION_LOST_us));
  #ifdef TUNING
    PID[roll].Kp = incomingReadings.roll.Kp;
    PID[roll].Ki = incomingReadings.roll.Ki;
    PID[roll].Kd = incomingReadings.roll.Kd;

    // PID[pitch].Kp  = incomingReadings.pitch.Kp;
    // PID[pitch].Ki  = incomingReadings.pitch.Ki;
    // PID[pitch].Kd  = incomingReadings.pitch.Kd;

    PID[pitch].Kp = incomingReadings.roll.Kp;
    PID[pitch].Ki = incomingReadings.roll.Ki;
    PID[pitch].Kd = incomingReadings.roll.Kd;

    PID[yaw].Kp = incomingReadings.yaw.Kp;
    PID[yaw].Ki = incomingReadings.yaw.Ki;
    PID[yaw].Kd = incomingReadings.yaw.Kd;
  #endif
}

// needs about 10 to 15us
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  static uint8_t packet_delivery_failed = 0;
  if (status == ESP_NOW_SEND_SUCCESS) {
    packet_delivery_failed = 0;
  } else {
    packet_delivery_failed++;
  }
  if (packet_delivery_failed > 20) {
    // shutdown = true; // turn off all motors
    sendingData.error = 0b1000000;
  }
}

void send_data() {
  static uint8_t cnt = 0;
  // send data every 200th time = every 100ms
  if (cnt >= 199) {
    sendingData.fl = motor[frontLeft];
    sendingData.fr = motor[frontRight];
    sendingData.bl = motor[backLeft];
    sendingData.br = motor[backRight];
    esp_now_send(controllerAddress, (uint8_t*)&sendingData, sizeof(sendingData));
    cnt = 0;
  } else cnt++;
}


void clrMessage(char* pString){
  *pString = '\0';
}

int8_t charToInt(char c){
  if((c >= '0') && (c <= '9')){
    return (c - '0');
  }
  else return -1;
}

// chatgpts improvement
float ARGV_TO_FLOAT(char* p) {
  if((*p == '\0') || (p == NULL)) return NAN;

  bool neg = (*p=='-');
  if (neg) ++p;
  float val = 0, place = 1;
  // parse integer part
  while (*p && *p != '.' && *p != ' '){
    if (*p < '0' || *p > '9') return NAN;
    val = val*10 + (*p - '0');
    ++p;
  }
  // parse fractional part
  if (*p == '.') {
    ++p;
    while (*p && *p != ' '){
      if (*p < '0' || *p > '9') return NAN;
      place /= 10.0f;
      val += (*p - '0') * place;
      ++p;
    }
  }
  return neg ? -val : val;
}

int8_t analyzeMessage(char* pMessage){
  char *argv[8];
  int argc = 0;

  char *token = strtok(pMessage, " \t\r\n");
  while (token && argc < 8) {
      argv[argc++] = token;
      token = strtok(NULL, " \t\r\n");
  }
  if(strcmp(argv[0], "setPos") == 0){
    for(uint8_t i = roll; (i <= yaw) && (i <= argc); i++){
      attitudeIMUFrame[i].globalOffset = attitudeIMUFrame[i].estimate - ARGV_TO_FLOAT(argv[i + 1]);
      Serial.println(attitudeIMUFrame[i].globalOffset);
    }
  }
  return 1;
}

void serialCommunication(){
  if(Serial.available()){
    char incomingChar = Serial.read();
    static uint8_t currentIdx = 0;
    message[currentIdx] = incomingChar;
    currentIdx++;
    if(incomingChar == '\n'){
      message[currentIdx] = '\0';
      currentIdx = 0;
      analyzeMessage(message);
      clrMessage(message);  
    }
  }
}

void printDelay(uint16_t dt){
  static uint16_t cnt = 0;
  static uint32_t dt_mean = 0;
  dt_mean += dt;
  cnt++;

  if(cnt >= 1000){
    cnt = 0;
    dt_mean /= 1000;
    Serial.println(dt_mean);
  }
  else if(cnt == 500){
    Serial.printf("roll: %5.3f°; pitch: %5.3f°; yaw: %5.3f°\n", (attitudeIMUFrame[roll].estimate), (attitudeIMUFrame[pitch].estimate), (attitudeIMUFrame[yaw].estimate));
  }
}

void deleteMePls(){
  // delete this function
  
}