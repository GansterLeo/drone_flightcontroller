/*
MPU6500.h
BananaPi Bit Team
juwan@banana-pi.com (Juwan·C)
wanghao@banana-pi.com (Hulk Wang)

We modified it based on https://github.com/bolderflight/MPU9250
The main purpose is to adapt the MPU9250 driver library of BPI-BIT (espressif32). 

Copyright (c) 2017 BananaPi Bit Team

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef MPU6500_h
#define MPU6500_h

#include "mpu6500_registers.h"
#include "mpu6500_register_values.h"

#include "Arduino.h"
#include "I2Cbus.hpp"
// #include "Wire.h"    // I2C library
#include "SPI.h"     // SPI library

class MPU6500{
  public:
    enum GyroRange
    {
      GYRO_RANGE_250DPS,
      GYRO_RANGE_500DPS,
      GYRO_RANGE_1000DPS,
      GYRO_RANGE_2000DPS
    };
    enum AccelRange
    {
      ACCEL_RANGE_2G,
      ACCEL_RANGE_4G,
      ACCEL_RANGE_8G,
      ACCEL_RANGE_16G    
    };
    enum DlpfBandwidth
    {
      DLPF_BANDWIDTH_184HZ,
      DLPF_BANDWIDTH_92HZ,
      DLPF_BANDWIDTH_41HZ,
      DLPF_BANDWIDTH_20HZ,
      DLPF_BANDWIDTH_10HZ,
      DLPF_BANDWIDTH_5HZ
    };
    enum LpAccelOdr
    {
      LP_ACCEL_ODR_0_24HZ = 0,
      LP_ACCEL_ODR_0_49HZ = 1,
      LP_ACCEL_ODR_0_98HZ = 2,
      LP_ACCEL_ODR_1_95HZ = 3,
      LP_ACCEL_ODR_3_91HZ = 4,
      LP_ACCEL_ODR_7_81HZ = 5,
      LP_ACCEL_ODR_15_63HZ = 6,
      LP_ACCEL_ODR_31_25HZ = 7,
      LP_ACCEL_ODR_62_50HZ = 8,
      LP_ACCEL_ODR_125HZ = 9,
      LP_ACCEL_ODR_250HZ = 10,
      LP_ACCEL_ODR_500HZ = 11
    };
    const int16_t tX[3] = {0,  1,  0};
    const int16_t tY[3] = {-1,  0,  0};
    const int16_t tZ[3] = {0,  0,  1};
    MPU6500(I2C_t &bus,uint8_t address);
    MPU6500(SPIClass &bus,uint8_t csPin);
    int8_t begin();
    int8_t setAccelRange(AccelRange range);
    int8_t setGyroRange(GyroRange range);
    int8_t setDlpfBandwidth(DlpfBandwidth bandwidth);
    int8_t setSrd(uint8_t srd);
    int8_t enableDataReadyInterrupt();
    int8_t disableDataReadyInterrupt();
    int8_t enableWakeOnMotion(float womThresh_mg,LpAccelOdr odr);
    int8_t readSensor();
    float getAccelX_mss();
    float getAccelY_mss();
    float getAccelZ_mss();
    float getGyroX_rads();
    float getGyroY_rads();
    float getGyroZ_rads();
    float getTemperature_C();
    
    int8_t calibrateGyro();
    float getGyroBiasX_rads();
    float getGyroBiasY_rads();
    float getGyroBiasZ_rads();
    void setGyroBiasX_rads(float bias);
    void setGyroBiasY_rads(float bias);
    void setGyroBiasZ_rads(float bias);
    int8_t calibrateAccel();
    float getAccelBiasX_mss();
    float getAccelScaleFactorX();
    float getAccelBiasY_mss();
    float getAccelScaleFactorY();
    float getAccelBiasZ_mss();
    float getAccelScaleFactorZ();
    void setAccelCalX(float bias,float scaleFactor);
    void setAccelCalY(float bias,float scaleFactor);
    void setAccelCalZ(float bias,float scaleFactor);
    
  protected:
    // i2c
    uint8_t _address;
    I2C_t *_i2c;
    const uint32_t _i2cRate = 400000; // 400 kHz
    size_t _numBytes; // number of bytes received from I2C
    // spi
    SPIClass *_spi;
    uint8_t _csPin;
    bool _useSPI;
    bool _useSPIHS;
    const uint8_t  SPI_READ = 0x80;
    const uint32_t SPI_LS_CLOCK = 1000000;  // 1 MHz
    const uint32_t SPI_HS_CLOCK = 20000000; // 20 MHz
    // track success of interacting with sensor
    int16_t _status;
    // buffer for reading from sensor
    uint8_t _buffer[21];
    // data counts
    int16_t _axcounts,_aycounts,_azcounts;
    int16_t _gxcounts,_gycounts,_gzcounts;
    int16_t _tcounts;
    // data buffer
    float _ax, _ay, _az;
    float _gx, _gy, _gz;
    float _t;
    // wake on motion
    uint8_t _womThreshold;
    // scale factors
    float _accelScale;
    float _gyroScale;
    const float _tempScale = 333.87f;
    const float _tempOffset = 21.0f;
    // configuration
    AccelRange _accelRange;
    GyroRange _gyroRange;
    DlpfBandwidth _bandwidth;
    uint8_t _srd;
    // gyro bias estimation
    size_t _numSamples = 400;
    double _gxbD, _gybD, _gzbD;
    float _gxb, _gyb, _gzb;
    // accel bias and scale factor estimation
    double _axbD, _aybD, _azbD;
    float _axmax, _aymax, _azmax;
    float _axmin, _aymin, _azmin;
    float _axb, _ayb, _azb;
    float _axs = 1.0f;
    float _ays = 1.0f;
    float _azs = 1.0f;

    // constants
    const float G = 9.807f;
    const float _d2r = 3.14159265359f/180.0f;
    
    // private functions
    int8_t writeRegister(uint8_t subAddress, uint8_t data);
    int8_t readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int16_t whoAmI();
};

class MPU6500FIFO: public MPU6500 {
  public:
    using MPU6500::MPU6500;
    int8_t enableFifo(bool accel, bool gyro, bool temp);
    int8_t readFifo();
    void getFifoAccelX_mss(size_t *size, float* data);
    void getFifoAccelY_mss(size_t *size, float* data);
    void getFifoAccelZ_mss(size_t *size, float* data);
    void getFifoGyroX_rads(size_t *size, float* data);
    void getFifoGyroY_rads(size_t *size, float* data);
    void getFifoGyroZ_rads(size_t *size, float* data);
    void getFifoTemperature_C(size_t *size, float* data);
  protected:
    // fifo
    bool _enFifoAccel,_enFifoGyro,_enFifoTemp;
    size_t _fifoSize,_fifoFrameSize;
    float _axFifo[85], _ayFifo[85], _azFifo[85];
    size_t _aSize;
    float _gxFifo[85], _gyFifo[85], _gzFifo[85];
    size_t _gSize;
    float _tFifo[256];
    size_t _tSize;
};

#endif
