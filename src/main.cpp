// flighcontroller — Copyright (C) 2025 GANSTER Leo
// This file is part of <project>
//
// <project> is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

#include "Arduino.h"
// #define CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_SILENT

#include "config.h"
#include "MPU6500.h"
#include "battery.h"
#include "driver/ledc.h"
#include "QuickPID.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>  // required for the esp_error_check()

// Gyro full-scale selection 
#define GYRO_RANGE_250DPS     MPU6500::GYRO_RANGE_250DPS
// Accel full-scale selection 
#define ACCEL_RANGE_2G        MPU6500::ACCEL_RANGE_2G  // Ensure enum scope is resolved
// Digital low pass filter selection
#define DLPF_BANDWIDTH_184HZ  MPU6500::DLPF_BANDWIDTH_184HZ
// Oneshot defines
#define ESC_OFFSET        (4096 * 0.13)
#define ONESHOT_TIMER     LEDC_TIMER_0
#define LEDC_MODE         LEDC_LOW_SPEED_MODE
#define ONESHOT0_CHANNEL  LEDC_CHANNEL_0
#define ONESHOT1_CHANNEL  LEDC_CHANNEL_1
#define ONESHOT2_CHANNEL  LEDC_CHANNEL_2
#define ONESHOT3_CHANNEL  LEDC_CHANNEL_3
#define ONESHOT_DUTY_RES  LEDC_TIMER_14_BIT
#define ONESHOT_DUTY      (4096)          // set duty to 25%
#define ONESHOT_FREQUENCY (2000)          // 2kHz

// message buffer
char messageBuf[MAX_MSSG_SIZE] = "";
// SPI object
SPIClass spiBus(VSPI);
// IMU object
MPU6500 mpu(spiBus, IMU_CS);
// esp now 
data_controller_to_drone incomingReadings;  // create the struct for incoming data
data_drone_to_controller sendingData;  // create struct for sending data
esp_now_peer_info_t peerInfo;

// function declaration 
void computeRotationMatrix(float pRCorrectM[][nOfAxisNames], const float pAngles[nOfAxisNames]);
void applyRotationMatrix(const float pDataInitialFrame[], float pDataNewFrame[], const float pRotationMtrx[][nOfAxisNames]);
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq); // from dRhemFlight
void controlAngle(stAttitude pAttitude[]);
void oneshot125_init(void);
void oneshot125_write(const float fl_vl, const float fr_vl, const float bl_vl, const float br_vl);
uint8_t getIMUdata(void);
void calculateIMUError(void);
int8_t loopRate(uint16_t freq);
float invSqrt(float x);
int8_t checkBattery(void);
void init_wlcomm(void);
void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, const int len);
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
void send_data(void);
int8_t charToInt(char c);
float ARGV_TO_FLOAT(char* p);
int8_t analyzeMessage(char* pMessage, char pSerialBuffer[], char* pTempBuffer);
void serialCommunication(char pSerialBuffer[], char* pTempBuffer);
void printDelay(char* pSerialBuffer, char* pTempBuffer, uint16_t dt);
void printPID(char pSerialBuffer[], char* pTempBuffer, stPID pPID[], size_t nOfElements);
void addToBuffer(char destiny[], char source[]);


void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  pinMode(INIT_READY_LED_PIN, OUTPUT);
  digitalWrite(INIT_READY_LED_PIN, LOW);
  Battery battery(3, Battery::BATTERY_LIPO, 0.25, BATTERY_VOLTAGE_PIN, &analogRead);

  Serial.printf("Hello\n");
  PID[roll].Kp = 0.00140000;
  PID[roll].Ki = 0.14000000;
  PID[roll].Kd = 0.00150000;
  
  PID[pitch].Kp = 0.00140000;
  PID[pitch].Ki = 0.14000000;
  PID[pitch].Kd = 0.00150000;

  PID[yaw].Kp = 0.;
  PID[yaw].Ki = 0.;
  PID[yaw].Kd = 0.;
  // put your setup code here, to run once:
  //checkBattery();
  spiBus.begin(SPIBUS_SCK, SPIBUS_MISO, SPIBUS_MOSI, IMU_CS);                         // SCK, MISO, MOSI, SS
  delay(3);
  // Initialize ESP-NOW communication
  init_wlcomm();
  oneshot125_init();
  oneshot125_write(0.0f, 0.0f, 0.0f, 0.0f);

  computationState currentState = ST_initialization;
  computationState prevState    = currentState;
  
  bool enableBatteryVoltControl = false;

  uint32_t wtdIteration = 0;
  
  while(1){
    // get current time
    prev_micros = micros();

    // clear message buffer
    char serialBuffer[SERIALBUFFER_SIZE] = "";
    char tempBuffer[SERIALBUFFER_SIZE*2] = "";

    // state machine
    bool strictCycletime            = false; // just currently set to false, later set back to default true // true if the cycletime musst be obeyed
    bool enableImuDataCommunication = false;
    bool enableMadgewick            = false;
    bool enableControlangle         = false;
    bool enableControlmixer         = false;
    bool enableMotorOutput          = false;
    switch(currentState){
      case ST_initialization:{
        strictCycletime = false;
        switch(prevState){
          case ST_initialization:{
            currentState  = ST_imuInit;
            prevState     = ST_initialization;
            break;
          }
          case ST_imuInit:{
            currentState  = ST_attitudeCalibration;
            prevState     = ST_initialization;
            break;
          }
          case ST_attitudeCalibration:{
            currentState  = ST_motorTesting;
            prevState     = ST_initialization;
            break;
          }
          case ST_motorTesting:{
            sprintf(tempBuffer, "Initialisation done!\n");
            digitalWrite(INIT_READY_LED_PIN, HIGH);
            currentState  = ST_flying;
            prevState     = ST_initialization;
            break;
          }
          default:{
            currentState  = ST_initialization;
            prevState     = ST_initialization;
          }
        }
        break;
      }
      case ST_imuInit:{
        strictCycletime = false;
        sprintf(tempBuffer, "\nNow IMU init\n");
        addToBuffer(serialBuffer, tempBuffer);
        sprintf(tempBuffer, "01 MPU.begin status: %d\n",                 mpu.begin());
        addToBuffer(serialBuffer, tempBuffer);
        sprintf(tempBuffer, "02 setAccelRange status: %d\n",             mpu.setAccelRange(ACCEL_RANGE_2G));
        addToBuffer(serialBuffer, tempBuffer);
        sprintf(tempBuffer, "03 setGyroRange status: %d\n",              mpu.setGyroRange(GYRO_RANGE_250DPS));
        addToBuffer(serialBuffer, tempBuffer);
        sprintf(tempBuffer, "04 setDlpfBandwidth status: %d\n",          mpu.setDlpfBandwidth(DLPF_BANDWIDTH_184HZ));
        addToBuffer(serialBuffer, tempBuffer);
        sprintf(tempBuffer, "05 set Data Output Rate status: %d\n",      mpu.setSrd(0)); // use default 1kHz 
        addToBuffer(serialBuffer, tempBuffer);
        sprintf(tempBuffer, "06 diabled data ready interrupt: %d\n",     mpu.disableDataReadyInterrupt());
        addToBuffer(serialBuffer, tempBuffer);
        sprintf(tempBuffer, "IMU init done, now oneshot125init\n");
        addToBuffer(serialBuffer, tempBuffer);
        
        prevState = currentState;
        currentState = ST_initialization;
        break;
      }
      case ST_attitudeCalibration:{
        enableImuDataCommunication  = true;
        enableMadgewick             = true;

        static uint16_t cntAttitude = 0;
        sprintf(tempBuffer, "\natCal %05d:\n", cntAttitude);
        addToBuffer(serialBuffer, tempBuffer);

        if (cntAttitude >= 20000) {
          cntAttitude = 0;
          prevState = currentState;
          currentState = ST_initialization;
        }
        else cntAttitude++;   
        break;
      }
      case ST_motorTesting:{
        strictCycletime = false;
        enableMotorOutput = true;
        enableImuDataCommunication = true;
        
        static float cntMotorThrottle = 0;
        static bool cntUp = true;
        if(cntMotorThrottle >= MAX_TEST_THROTTLE){
          cntUp = false;
        }
        cntMotorThrottle += (0.04 * dt) * cntUp? +1:-1;
        if((cntMotorThrottle <= 0) && (!cntUp)){
          cntMotorThrottle = 0;
          prevState = currentState;
          currentState = ST_initialization;
        }

        motor[frontLeft] = motor[frontRight] = motor[backLeft] = motor[backLeft] = cntMotorThrottle;
        break;
      }
      case ST_resetPID:{
        enableImuDataCommunication = true;      
        // yet not possible 

        prevState = currentState;
        currentState = ST_initialization;
        break;
      }
      case ST_flying:{
        enableImuDataCommunication = true;
        enableMadgewick            = true;
        enableControlangle         = true;
        enableControlmixer         = true;
        enableMotorOutput          = true;

        if(currentEvent == EV_emptyBattery){
          currentEvent = EV_NONE;
          prevState = currentState;
          currentState = ST_emptyBattery;
        }
        break;
      }
      case ST_emptyBattery:{
        sprintf(tempBuffer, "\nEMPTY BATTERY!!!\nUnable to fly\n");
        addToBuffer(serialBuffer, tempBuffer);
        // do something
        break;
      }
      default:{
        ESP.restart();
        break;
      }
    }
   
    // INPUT
    if(enableImuDataCommunication){
      if(getIMUdata() == 1) {
        enableMadgewick = false;
        // communication to IMU failed
      }
    }

    serialCommunication(serialBuffer, tempBuffer);

    if(enableBatteryVoltControl) {
      int8_t result = checkBattery();
      currentEvent = result ? EV_emptyBattery : currentEvent;
    }

    // COMPUTATION
    if(enableMadgewick)    Madgwick6DOF(imu.gx, -imu.gy, -imu.gz, -imu.ax, imu.ay, imu.az, dt);
    if(enableControlangle) controlAngle(attitude);
    if(enableControlmixer){
      motor[frontLeft]   = incomingReadings.throttle + PID[pitch].value + PID[roll].value + PID[yaw].value;  //Front Left
      motor[frontRight]  = incomingReadings.throttle + PID[pitch].value - PID[roll].value - PID[yaw].value;  //Front Right
      motor[backLeft]    = incomingReadings.throttle - PID[pitch].value + PID[roll].value - PID[yaw].value;  //Back Right
      motor[backRight]   = incomingReadings.throttle - PID[pitch].value - PID[roll].value + PID[yaw].value;  //Back Left
    }
    
    // OUTPUT
    if(enableMotorOutput){
      oneshot125_write(motor[frontLeft], motor[frontRight], motor[backLeft], motor[backRight]);
    }
    else {
      oneshot125_write(0, 0, 0, 0);
    }
    send_data();
    
    printDelay(serialBuffer, tempBuffer, (uint16_t)(micros() - prev_micros));

    
    serialBuffer[SERIALBUFFER_SIZE - 1] = '\0';
    Serial.print(serialBuffer);
    Serial.printf("\ncycletime:%05d\n", micros() - prev_micros);
    // cycletime control 
    int8_t cycleCheckResult = loopRate(2000);
    if((strictCycletime) && (cycleCheckResult == -1)){
      Serial.printf("\nStrict Cycletime: %d\n", strictCycletime);
      Serial.printf("Current State: %d, previous State: %d\n", currentState, prevState);
      Serial.printf("Violated cycletime constrains!\nRestarting...\n");
      ESP.restart();
    }

    // calculate actual cycle time
    dt = (micros() - prev_micros) / 1.0e6;
  }
}

void loop() {
  Serial.print("Jumped out of while(1)!\nReseting...\n");
  ESP.restart();
}

void addToBuffer(char destiny[], char source[]){
  if((SERIALBUFFER_SIZE - strlen(destiny)) > strlen(source)){
    strcpy(destiny, source);
  }
  source[0] = '\0';
}

void computeRotationMatrix(float pRCorrectM[][nOfAxisNames], const float pAngles[nOfAxisNames]){
    // Compute with initial angles (NEGATION applied inside trig)
  float cr = cos(-DEG_TO_RAD * pAngles[roll]);    // =  cos(roll)
  float sr = sin(-DEG_TO_RAD * pAngles[roll]);    // = -sin(roll)
  float cp = cos(-DEG_TO_RAD * pAngles[pitch]);   // =  cos(pitch)
  float sp = sin(-DEG_TO_RAD * pAngles[pitch]);   // = -sin(pitch)
  float cy = cos(-DEG_TO_RAD * pAngles[yaw]);     // =  cos(yaw)
  float sy = sin(-DEG_TO_RAD * pAngles[yaw]);     // = -sin(yaw)

  // Direct matrix (equivalent to Rz * Ry * Rx)
  float temp[3][3] = {
      { cp*cy,              cp*sy,              -sp },
      { sr*sp*cy - cr*sy,   sr*sp*sy + cr*cy,   sr*cp },
      { cr*sp*cy + sr*sy,   cr*sp*sy - sr*cy,   cr*cp }
  };

  Serial.printf("Rotation Matrix\n");
  for(uint8_t i = 0; i < nOfAxisNames; i++){
    for(uint8_t k = 0; k < nOfAxisNames; k++){
      Serial.printf("%10.9f, ", temp[i][k]);
    }
    Serial.printf("\n");
  }

  memcpy(pRCorrectM, temp, sizeof(temp));
}

void applyRotationMatrix(const float pDataInitialFrame[], float pDataNewFrame[], const float pRotationMtrx[][nOfAxisNames]){
  for(uint8_t i = 0; i < nOfAxisNames; i++){
    pDataNewFrame[i] = 0;
    for(uint8_t k = 0; k < nOfAxisNames; k++){
      pDataNewFrame[i] += pDataInitialFrame[k] * pRotationMtrx[i][k];
    }
  }
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
  attitude[roll].estimate   = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29577951;                   //degrees
  attitude[pitch].estimate  = asin(constrain(-2.0f * (q1 * q3 - q0 * q2), -0.999999, 0.999999)) * 57.29577951;  //degrees
  attitude[yaw].estimate    = -atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29577951;                   //degrees

}

void controlAngle(stAttitude pAttitude[]) {
  //DESCRIPTION: Computes control commands based on state error (angle)

  // definition static integrals
  static double integral_roll = 0;
  static double integral_pitch = 0;
  static double integral_yaw = 0;
  // definition of previous errors
  static float prevErrorRoll = 0;
  static float prevErrorPitch = 0;
  static float prevErrorYaw = 0;
  // derivative terms
  static float derivative[3] = {0};


  if (incomingReadings.throttle < 0.06) {  //Don't let integrator build if throttle is too low (<6%)
    integral_roll = 0;
    integral_pitch = 0;
    integral_yaw = 0;
  }

  //Roll
  float error_roll = pAttitude[roll].desired - (pAttitude[roll].estimate);
  integral_roll += error_roll * dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative[roll] = (1-PID_D_LOWPASS)*derivative[roll] + PID_D_LOWPASS*(error_roll - prevErrorRoll)/dt; // calculate derivative and lowpass filter it
  PID[roll].value = ((PID[roll].Kp * error_roll) + (PID[roll].Ki * integral_roll) + (PID[roll].Kd * derivative[roll])); 
  prevErrorRoll = error_roll;

  //Pitch
  float error_pitch = pAttitude[pitch].desired - (pAttitude[pitch].estimate);
  integral_pitch += error_pitch * dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative[pitch] = (1-PID_D_LOWPASS)*derivative[pitch] + PID_D_LOWPASS*(error_pitch - prevErrorPitch) / dt; // calculate derivative and lowpass filter it
  PID[pitch].value = ((PID[pitch].Kp * error_pitch) + (PID[pitch].Ki * integral_pitch) + (PID[pitch].Kd * derivative[pitch]));  
  prevErrorPitch = error_pitch;

  //Yaw, stablize on rate from GyroZ
  float error_yaw = pAttitude[yaw].desired - imu.gz;
  integral_yaw += error_yaw * dt;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative[yaw] = (1-PID_D_LOWPASS)*derivative[yaw] + PID_D_LOWPASS*(error_yaw - prevErrorYaw) / dt;

  PID[yaw].value = ((PID[yaw].Kp * error_yaw) + (PID[yaw].Ki * integral_yaw) + (PID[yaw].Kd * derivative[yaw]));  

  // update prev variables
  prevErrorYaw = error_yaw;
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
  DESCRIPTION: updates duty cycle of all 4 oneshot outputs
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

uint8_t getIMUdata(void) {
  // define acc and gyro_prev variables here
  static float axPrev, ayPrev = 0;
  static float azPrev = 9.81;
  static float gxPrev, gyPrev, gzPrev = 0;
  uint8_t returnValue = 0;

  mpu.readSensor();
  float rawData[nOfAxisNames] = {
    mpu.getAccelX_mss(),
    mpu.getAccelY_mss(),
    mpu.getAccelZ_mss()
  };
  float newFrame[nOfAxisNames];
  applyRotationMatrix(rawData, newFrame, rotationMtrx);
  //Accelerometer and correct the outputs with the calculated error values
  imu.ax = newFrame[0] - AccErrorX;  //G's
  imu.ay = newFrame[1] - AccErrorY;
  imu.az = newFrame[2] - AccErrorZ;

  // LP filter accelerometer data
  // imu.ax = ((1.0 - B_accel) * axPrev) + (B_accel * imu.ax);
  // imu.ay = ((1.0 - B_accel) * ayPrev) + (B_accel * imu.ay);
  // imu.az = ((1.0 - B_accel) * azPrev) + (B_accel * imu.az);
  axPrev = imu.ax;
  ayPrev = imu.ay;
  azPrev = imu.az;

  rawData[0] = mpu.getGyroX_rads();
  rawData[1] = mpu.getGyroY_rads();
  rawData[2] = mpu.getGyroZ_rads();
  applyRotationMatrix(rawData, newFrame, rotationMtrx);
  //Gyro and correct the outputs with the calculated error values
  imu.gx = (rawData[0] * RAD_TO_DEG) - GyroErrorX;  //deg/sec
  imu.gy = (rawData[1] * RAD_TO_DEG) - GyroErrorY;
  imu.gz = (rawData[2] * RAD_TO_DEG) - GyroErrorZ;

  // //LP filter gyro data
  // imu.gx = ((1.0 - B_gyro) * gxPrev) + (B_gyro * imu.gx);
  // imu.gy = ((1.0 - B_gyro) * gyPrev) + (B_gyro * imu.gy);
  // imu.gz = ((1.0 - B_gyro) * gzPrev) + (B_gyro * imu.gz);
  gxPrev = imu.gx;
  gyPrev = imu.gy;
  gzPrev = imu.gz;

  return 0;
}


void calculateIMUError(void){
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface 

  double AcX,AcY,AcZ,GyX,GyY,GyZ;
  AcX = 0.0;
  AcY = 0.0;
  AcZ = 0.0;
  GyX = 0.0;
  GyY= 0.0;
  GyZ = 0.0;
  
  //Read IMU values 12000 times
  uint16_t c = 0;
  while(c < 12000){
    mpu.readSensor();    
    //Sum all readings
    float rawData[nOfAxisNames] = {
      mpu.getAccelX_mss(),
      mpu.getAccelY_mss(),
      mpu.getAccelZ_mss()
    };
    float newFrame[nOfAxisNames];
    applyRotationMatrix(rawData, newFrame, rotationMtrx);
    //Accelerometer and correct the outputs with the calculated error values
    AcX += newFrame[0] - AccErrorX;  //G's
    AcY += newFrame[1] - AccErrorY;
    AcZ += newFrame[2] - AccErrorZ;
    rawData[0] = mpu.getGyroX_rads();
    rawData[1] = mpu.getGyroY_rads();
    rawData[2] = mpu.getGyroZ_rads();
    applyRotationMatrix(rawData, newFrame, rotationMtrx);
    //Gyro and correct the outputs with the calculated error values
    GyX += (rawData[0] * RAD_TO_DEG) - GyroErrorX;  //deg/sec
    GyY += (rawData[1] * RAD_TO_DEG) - GyroErrorY;
    GyZ += (rawData[2] * RAD_TO_DEG) - GyroErrorZ;
    
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AcX  = AcX / c;
  AcY  = AcY / c;
  AcZ  = (AcZ / c) - 9.81;
  GyX = GyX / c;
  GyY = GyY / c;
  GyZ = GyZ / c;

  Serial.print("float AcX = ");
  Serial.print(AcX);
  Serial.println(";");
  Serial.print("float AcY = ");
  Serial.print(AcY);
  Serial.println(";");
  Serial.print("float AcZ = ");
  Serial.print(AcZ);
  Serial.println(";");
  
  Serial.print("float GyX = ");
  Serial.print(GyX);
  Serial.println(";");
  Serial.print("float GyY = ");
  Serial.print(GyY);
  Serial.println(";");
  Serial.print("float GyZ = ");
  Serial.print(GyZ);
  Serial.println(";");
}


int8_t loopRate(uint16_t freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  float invFreq = (1.0 / freq) * 1000000.0;
  unsigned long checker = micros();
  if(invFreq < (checker - prev_micros)){
    return -1;
  }
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - prev_micros)) {
    checker = micros();
  }
  return 1;
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
}

int8_t checkBattery(Battery *pBattery){
  pBattery->update();
  if(pBattery->getPercentage() < 20){
    shutdown = true;
    sendingData.error = 1;
    sendingData.status |= (STAT_CRITICAL_LOW_VOLT | STAT_HALF_CAPACITY);
    return 0;
  } else if(pBattery->getPercentage() < 30){
    sendingData.status |= STAT_CRITICAL_LOW_VOLT;
    sendingData.status &= ~STAT_HALF_CAPACITY;
    return 1;
  } else if(pBattery->getPercentage() < 50){
    sendingData.status &= ~STAT_CRITICAL_LOW_VOLT;
    sendingData.status |= STAT_HALF_CAPACITY;
    return 2;
  } else{
    sendingData.status &= ~(STAT_CRITICAL_LOW_VOLT | STAT_HALF_CAPACITY);
    return 3;
  }
}

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

// gets called, if esp recieves data
// needs between 4 to 10us
void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, const int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  if ((incomingReadings.status & (1 << 7)) != (sendingData.status & (1 << 7))) {
    sendingData.error = 1;  // two different modes
    shutdown = true;
  }
  incomingReadings.throttle = constrain(incomingReadings.throttle, 0.0, 1.0);
  attitude[roll].desired    = constrain(incomingReadings.roll_pos_deg, -(MAX_ROLL), MAX_ROLL);
  attitude[pitch].desired   = constrain(incomingReadings.pitch_pos_deg, -(MAX_PITCH), MAX_PITCH);
  attitude[yaw].desired     = constrain(incomingReadings.yaw_deg_per_s, -(MAX_YAW), MAX_YAW);

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
    cnt = 0;
    sendingData.fl = motor[frontLeft];
    sendingData.fr = motor[frontRight];
    sendingData.bl = motor[backLeft];
    sendingData.br = motor[backRight];
    sendingData.pitch_pos_deg = attitude[pitch].estimate;
    sendingData.roll_pos_deg = attitude[roll].estimate;
    sendingData.yaw_pos_deg = attitude[yaw].estimate;
    esp_now_send(controllerAddress, (uint8_t*)&sendingData, sizeof(sendingData));
  } else cnt++;
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

int8_t analyzeMessage(char* pMessage, char pSerialBuffer[], char* pTempBuffer){
  char *argv[8];
  int argc = 0;

  char *token = strtok(pMessage, " \t\r\n");
  while (token && argc < 8) {
      argv[argc] = token;
      if(argc > 0){
        argv[argc] -= 1;  // replace last character of the pre argument with \0
        *argv[argc] = '\0';
        argv[argc] += 1;
      }
      argc++;
      token = strtok(NULL, " \t\r\n");
  }
  if(strcmp(argv[0], "setPos") == 0){
    float offset[nOfAxisNames]= {0};
    for(uint8_t i = roll; (i <= yaw) && (i <= argc); i++){
      offset[i] = attitude[i].estimate - ARGV_TO_FLOAT(argv[i + 1]);
    }
    computeRotationMatrix(rotationMtrx, offset);
    sprintf(pTempBuffer, "updated rotation Matrix!\n");
    addToBuffer(pSerialBuffer, pTempBuffer);
  }
  else if(strcmp(argv[0], "restart") == 0){
    sprintf(pTempBuffer, "Restarting...\n");
    addToBuffer(pSerialBuffer, pTempBuffer);
    ESP.restart();
  }
  else if(strcmp(argv[0], "throttle") == 0){
    float value = ARGV_TO_FLOAT(argv[1]);
    if(value == NAN) return VALUE_NOT_VALID;
    incomingReadings.throttle = value;
    sprintf(pTempBuffer, "throttle: %05.4f\n", incomingReadings.throttle);
    addToBuffer(pSerialBuffer, pTempBuffer);
  }
  else if(strcmp(argv[0], "print") == 0){
    if(strcmp(argv[1], "pid") == 0){
      printPID(pSerialBuffer, pTempBuffer, PID, nOfAxisNames);
    }
  }
  else if(strcmp(argv[0], "pid") == 0){
    // check if the entered value is even valid othervise continuing is unneccessary
    float value = ARGV_TO_FLOAT(argv[3]);
    if(value == NAN) return VALUE_NOT_VALID;

    axisName selectedAxis; 
    switch(*argv[1]){
      case 'p':
      case 'P':
        selectedAxis = pitch;
        break;
      case 'r':
      case 'R':
        selectedAxis = roll;
        break;
      case 'y':
      case 'Y':
        selectedAxis = yaw;
        break;
      default:
        return ARGV_2ND_NOT_VALID;
    }
    if((strcmp(argv[2], "kp") == 0) || (strcmp(argv[2], "Kp") == 0)){
      PID[selectedAxis].Kp = value;
      sprintf(pTempBuffer, "Kp: %05.3f\n", PID[selectedAxis].Kp);
      addToBuffer(pSerialBuffer, pTempBuffer);
    }
    else if((strcmp(argv[2], "ki") == 0) || (strcmp(argv[2], "Ki") == 0)){
      PID[selectedAxis].Ki = value;
      sprintf(pTempBuffer, "Kp: %05.3f\n", PID[selectedAxis].Ki);
      addToBuffer(pSerialBuffer, pTempBuffer);
    }
    else if((strcmp(argv[2], "kd") == 0) || (strcmp(argv[2], "Kd") == 0)){
      PID[selectedAxis].Kd = value;
      sprintf(pTempBuffer, "Kp: %05.3f\n", PID[selectedAxis].Kd);
      addToBuffer(pSerialBuffer, pTempBuffer);
    }
    else return ARGV_3RD_NOT_VALID;
  }
  
  return SUCCESS;
}

void serialCommunication(char pSerialBuffer[], char* pTempBuffer){
  if(Serial.available()){
    char incomingChar = Serial.read();
    Serial.printf("%c", incomingChar);
    static uint8_t currentIdx = 0;
    messageBuf[currentIdx] = incomingChar;
    currentIdx++;
    if(incomingChar == '\n' or incomingChar == '\0'){
      messageBuf[currentIdx] = '\0';
      currentIdx = 0;
      int8_t error = analyzeMessage(messageBuf, pSerialBuffer, pTempBuffer);
      if(error != SUCCESS){
        sprintf(pTempBuffer, "SERIAL ERROR: %d", error);
        addToBuffer(pSerialBuffer, pTempBuffer);
      };
      messageBuf[0] = '\0'; // clear message
    }
  }
}

void printDelay(char* pSerialBuffer, char* pTempBuffer, uint16_t dt){
  static uint16_t cnt = 0;
  static uint32_t dt_mean = 0;
  dt_mean += dt;
  cnt++;

  if(cnt >= 1000){
    cnt = 0;
    dt_mean /= 1000;
    sprintf(pTempBuffer, "%d", dt_mean);
    addToBuffer(pSerialBuffer, pTempBuffer);
  }
  #ifndef SERIAL_BOOST
  else if(cnt == 500){
    sprintf(pTempBuffer, "roll: %5.3f°; pitch: %5.3f°; yaw: %5.3f°\n", (attitude[roll].estimate), (attitude[pitch].estimate), (attitude[yaw].estimate));
    addToBuffer(pSerialBuffer, pTempBuffer);
    sprintf(pTempBuffer, "throttle: %05.4f\n", incomingReadings.throttle);
    addToBuffer(pSerialBuffer, pTempBuffer);
    sprintf(pTempBuffer, "motor fl: %5.3f; fr: %5.3f; bl: %5.3f; br: %5.3f\n", motor[frontLeft], motor[frontRight], motor[backLeft], motor[backRight]);
    addToBuffer(pSerialBuffer, pTempBuffer);
    sprintf(pTempBuffer, "%05dus\n", (uint)micros()-prev_micros);
    addToBuffer(pSerialBuffer, pTempBuffer);
  }
  #else
    sprintf(pTempBuffer, "%5.3f,%5.3f,%5.3f\n", (attitude[roll].estimate), (attitude[pitch].estimate), (attitude[yaw].estimate));
    addToBuffer(pSerialBuffer, pTempBuffer);
    sprintf(pTempBuffer, "roll:%5.3f,pitch:%5.3f,yaw:%5.3f\n", (attitude[roll].estimate), (attitude[pitch].estimate), (attitude[yaw].estimate));
    addToBuffer(pSerialBuffer, pTempBuffer);
    sprintf(pTempBuffer, "throttle: %05.4f\n", incomingReadings.throttle);
    addToBuffer(pSerialBuffer, pTempBuffer);
    sprintf(pTempBuffer, "mfl:%5.3f,fr:%5.3f,bl:%5.3f,br:%5.3f\n", motor[frontLeft], motor[frontRight], motor[backLeft], motor[backRight]);
    addToBuffer(pSerialBuffer, pTempBuffer);
    sprintf(pTempBuffer, "%05dus\n", (uint)micros()-prev_micros);
    addToBuffer(pSerialBuffer, pTempBuffer);
  
  #endif
}

void printPID(char pSerialBuffer[], char* pTempBuffer, stPID pPID[], size_t nOfElements){
  sprintf(pSerialBuffer + strlen(pSerialBuffer), "PID values\n");
      for(uint8_t i = 0; i < nOfElements; i++){
        sprintf(pTempBuffer, "axis: %d\n", i);
        addToBuffer(pSerialBuffer, pTempBuffer);
        sprintf(pTempBuffer, "\tKp: %10.8f\n", pPID[i].Kp);
        addToBuffer(pSerialBuffer, pTempBuffer);
        sprintf(pTempBuffer, "\tKi: %10.8f\n", pPID[i].Ki);
        addToBuffer(pSerialBuffer, pTempBuffer);
        sprintf(pTempBuffer, "\tKd: %10.8f\n", pPID[i].Kd);
        addToBuffer(pSerialBuffer, pTempBuffer);
      }
      sprintf(pTempBuffer, "\n");
      addToBuffer(pSerialBuffer, pTempBuffer);
}
