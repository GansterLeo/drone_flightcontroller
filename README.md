# My Flightcontroller for a Quadcopter

## 1. Overview
This README is intended to be a short documentation of the quadcopter I engineerd over the course of 8 months. 

First of all, I want to thank Nicholas Rehm for his DrehmFlight project, who's code I analyzed and helped me troumendously, to create the foundation and basic structure of my flightcontroller. 
My flightcontrollers is based on an ESP32 wroom, it communicates via long range ESP NOW with a second ESP32 wroom, that surfes as the controller to steer the drone. The loop rate is limited to 2kHz, as the motor controllers are not able to update faster and a loop rate of 2kHz proofs to be more than sufficient for this size of quadcopter, eventough the esp32 would be capable of running the code at approxamitly 11kHz. 
The orientation estimation is performed via a 6DOF IMU and a 6DOF Madgewick filter. The IMU is connected via SPI and can be virtually leveled (inside the ESP32 software), which makes the mechanical design much easier. 
The control loop is leveling leveling the drone to a certain angle, provided the controller uC.

## 2. Quick Demo
The following link provides a short demonstration video.

## 3. Features
* Virtual leveling of the IMU
* Up to 220m range over ESP NOW

## 4. Technical Summary
### 4.1 IMU
The IMU used for this project is the MPU6500, which is a 6DOF IMU (3x Acceleration, 3x Gyroscope) and is capable of communicating over SPI.

## 5. Engineering Journey
### 5.1 First prototype
Early on I was determined, to use two ESP32 for my quadcopter project and let them communicate over ESP NOW long range. As I already did so, with an RC car I built in the summer holidays before.
I started this project in the christmas holidays  and the first prototype was ready for flight tests after about 1.5 months. However, it proved that a flying quadcopter, is not that simple to create as some YT guys explain it to be. The first prototype used the most basic BL-DC ESC on the market. These esc's have the same control interface as standard hobbyist servos do, resp. a PWM signal with a frequency of 50Hz. The IMU used in the first prototype, was a BNO055 from Bosch, which communicated over I2C with the ESP32. 

### 5.2 Correctly tuning the PID controllers would fix everything!
At least that was my assumption, and I tuned the PID control loops for hours and hours, with some practical results. In the end I came to the conclution that the drone was indeed reacting to orientation missalignments but way to slow. I concluded that the orientation algorithm was just to bad, as I used a simple complementary filter for the gyro and accelerometer readings.

### 5.3 *A New Hope* - The standard kalman filter
Now that would solve all my problems! After watching some videos about the kalman filters and a person who allegedly used it for a drone at 50Hz loop rate. I was determined to use it for my drone. The initial opitmismn faded quickly after realising that *The Reality Striked Back*, resp. orientation readings were completly of and had a lot of noise. Additionaly adding 2 paramters to the already existing 9 parameters from the PID controllers did not improve my situation either. 
I researched further and got the feeling that my loop rate was simply to slow and decided that I would have to rebuild my drone.

### 5.4 The problems just began - *The I2C Wars*
No, problem I just have to speed up my code. The ESP32 is so fast that this shouldn't be a problem. I thought back to a YT video, I saw a few months before, about the "DrehmFlight", a simple flightcontroller for drones and VTOL aircrafts. There I got the magical loop rate frequency of 2kHz. The first version was too slow, because of the Adafruit IMU library. As a result I communicated with the IMU over the I2C HAL of the ESP32.
Testing the new code, that should finally run at 2kHz, brought a lot of confusion: One time the I2C communication lasts 500us and the other time 2500us, for the same amount of bytes. After hours of googling and some dinner dates with ChatGPT, I found out that the BNO055, uses I2C clock stretching. Because of this "feature" the sensor is practicly useless for my drone project.

### 5.5 New IMU new problems 
The MPU6050 a 6DOF IMU communicating over I2C would be the solution, as this IC does not use clock stretching and is seemingly capable of overclocking up to 2.5 times the maximal frequency resp. 1MHz, according to the DrehmFlight basic code. Finally, the code was capable of running at 2kHz, at least for some time. After a few seconds a communication error caused a complete failure of the ESP32 I2C driver. Moreover, reseting this ESP32 I2C driver and the I2C bus in software did not lead to reestablishing I2C communication with the MPU6050 in a reasonable amount of time. Improving the I2C hardware increased the time until failure significantly. Analyzing the bus via a logical analyzer, showed that the clock frequency of 1MHz was to high for the ESP32 I2C driver. However lowering the clock frequency was no option, as the 2kHz loop rate target was barely rechable with the 1MHz clock frequency.

### 5.6 EKF - *Return of the problems*
While working to increase the loop rate, I simultaniously learnt a lot about control algorithmns specifically the Kalman filters. I switched my orientation estimation from the standard Kalman Filter to the Extended Kalman Filter (EKF). While struggeling the EKF implementation from Phil's Lab, caused by some typos, I decided to ask a teacher of mine for advice. He adviced me to use the Madgwick filter, as the DrehmFlight implementation uses too.
### 5.7 *The Serial Peripheral Interface Awakens*
switching to spi IMUs
### 5.8 *The Last Orientation Algorithmn*
Madgwick algorithmn
Drehmfligh
new motor controllers
### 5.9 *PID: A Tuning Story* 
as already mentioned a lot of tuning
a lot of pid tuning - really getting to now these algorithmns, cleared missconceptions misleaded by AI
building test stand and tuning on it
### 5.10 *The Rise of the Quadcopter*
after a lot of tuning finally beeing able to fly. 
Conclusion: Should have learnt to fly a drone before building one.
