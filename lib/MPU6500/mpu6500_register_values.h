#ifndef MPU6500_REGISTER_VALUES_H
#define MPU6500_REGISTER_VALUES_H
#include <stdint.h>
// ————————————————————————————————————————————————
// ACCEL_CONFIG   (0x1C): full‑scale range selection
// ————————————————————————————————————————————————
const uint8_t ACCEL_FS_SEL_2G   = 0x00;  // ±2 g
const uint8_t ACCEL_FS_SEL_4G   = 0x08;  // ±4 g
const uint8_t ACCEL_FS_SEL_8G   = 0x10;  // ±8 g
const uint8_t ACCEL_FS_SEL_16G  = 0x18;  // ±16 g

// ————————————————————————————————————————————————
// ACCEL_CONFIG2  (0x1D): accel DLPF and FCHOICE_B
// ————————————————————————————————————————————————
const uint8_t ACCEL_DLPF_184    = 0x01;  // BW = 184 Hz (FCHOICE_B=0)
const uint8_t ACCEL_DLPF_92     = 0x02;  // BW = 92 Hz
const uint8_t ACCEL_DLPF_41     = 0x03;  // BW = 41 Hz
const uint8_t ACCEL_DLPF_20     = 0x04;  // BW = 20 Hz
const uint8_t ACCEL_DLPF_10     = 0x05;  // BW = 10 Hz
const uint8_t ACCEL_DLPF_5      = 0x06;  // BW = 5 Hz
// (when FCHOICE_B=1 → ACCEL_DLPF bypass, value = 0x00)

// ————————————————————————————————————————————————
// GYRO_CONFIG    (0x1B): gyro full‑scale range
// ————————————————————————————————————————————————
const uint8_t GYRO_FS_SEL_250DPS  = 0x00;  // ±250 °/s
const uint8_t GYRO_FS_SEL_500DPS  = 0x08;  // ±500 °/s
const uint8_t GYRO_FS_SEL_1000DPS = 0x10;  // ±1000 °/s
const uint8_t GYRO_FS_SEL_2000DPS = 0x18;  // ±2000 °/s

// ————————————————————————————————————————————————
// CONFIG         (0x1A): gyro DLPF and FCHOICE_B
// ————————————————————————————————————————————————
const uint8_t GYRO_DLPF_184      = 0x01;  // BW = 184 Hz
const uint8_t GYRO_DLPF_92       = 0x02;  // BW = 92 Hz
const uint8_t GYRO_DLPF_41       = 0x03;  // BW = 41 Hz
const uint8_t GYRO_DLPF_20       = 0x04;  // BW = 20 Hz
const uint8_t GYRO_DLPF_10       = 0x05;  // BW = 10 Hz
const uint8_t GYRO_DLPF_5        = 0x06;  // BW = 5 Hz
// (when FCHOICE_B=1 → DLPF bypass, value = 0x00)

// ————————————————————————————————————————————————
// SMPDIV        (0x19): sample rate divider
// ————————————————————————————————————————————————
 // no fixed constants—write 0–255 to adjust SAMPLE_RATE = 
 // internal_clock/(1 + SMPDIV)

// ————————————————————————————————————————————————
// INT_ENABLE    (0x38): interrupt enables
// ————————————————————————————————————————————————
const uint8_t INT_RAW_RDY_EN    = 0x01;  // data ready
const uint8_t INT_WOM_EN        = 0x40;  // wake‑on‑motion
// (to fully disable: INT_DISABLE = 0x00)
// pulse duration option (in INT_PIN_CFG): INT_PULSE_50US = 0x00

// ————————————————————————————————————————————————
// PWR_MGMT_1    (0x6B): reset, cycle, clock select
// ————————————————————————————————————————————————
const uint8_t PWR_RESET         = 0x80;  // reset device
const uint8_t PWR_CYCLE         = 0x20;  // cycle between sleep and wake
const uint8_t CLOCK_SEL_PLL     = 0x01;  // PLL with X axis gyroscope ref

// ————————————————————————————————————————————————
// PWR_MGMT_2    (0x6C): disable sensors
// ————————————————————————————————————————————————
const uint8_t SEN_ENABLE        = 0x00;  // all on
const uint8_t DIS_GYRO          = 0x07;  // disable X, Y, Z gyro

// ————————————————————————————————————————————————
// USER_CTRL     (0x6A): master‑I²C, fifo, etc.
// ————————————————————————————————————————————————
const uint8_t I2C_MST_EN        = 0x20;  // enable AUX I²C Master

// ————————————————————————————————————————————————
// I2C_MST_CTRL  (0x24): I²C master clock
// ————————————————————————————————————————————————
const uint8_t I2C_MST_CLK       = 0x0D;  // ~400 kHz bus

// ————————————————————————————————————————————————
// I2C_SLV0_CTRL (0x27): slave‑0 enable + read flag
// ————————————————————————————————————————————————
const uint8_t I2C_SLV0_EN       = 0x80;  // enable Slave 0
const uint8_t I2C_READ_FLAG     = 0x80;  // OR into SLV0_ADDR to read

// ————————————————————————————————————————————————
// MOT_DETECT_CTRL (0x69): wake‑on‑motion
// ————————————————————————————————————————————————
const uint8_t ACCEL_INTEL_EN    = 0x80;  // enable accel intelligence
const uint8_t ACCEL_INTEL_MODE  = 0x40;  // motion triggered interrupts

// ————————————————————————————————————————————————
// LP_ACCEL_ODR  (0x1E): low‑power accel ODR must be set (0–255)
// WOM_THR       (0x1F): wake‑on‑motion threshold (0–255)
// ————————————————————————————————————————————————
 // write 0x00–0xFF to tune ODR / threshold

// ————————————————————————————————————————————————
// FIFO_EN       (0x23): enable data to FIFO
// ————————————————————————————————————————————————
const uint8_t FIFO_TEMP         = 0x80;  // temperature
const uint8_t FIFO_GYRO         = 0x70;  // all 3 axes
const uint8_t FIFO_ACCEL        = 0x08;  // accel

// _________________________________________________
// INTERRUPT REGISTER
// _________________________________________________
const uint8_t INT_DISABLE = 0x00;
const uint8_t INT_PULSE_50US = 0x00;

#endif