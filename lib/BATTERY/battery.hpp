/*
*   ChatGPT implementation - allegedly better
*   (+) uses template based design
*   (-) all the logic is in the header (.hpp)
*/

#pragma once

#include <cstdint>
#include <map>

// =======================
//   Battery Type Enum
// =======================

enum class BatteryType {
    LIION,
    LIPO
};

// =======================
//   Voltage Constants
// =======================

constexpr double LIION_MAX_CELL_VOLTAGE = 4.20;
constexpr double LIPO_MAX_CELL_VOLTAGE  = 4.20;

constexpr double DEFAULT_WEIGHT = 0.1;

// =======================
//   State of Charge Table
// =======================

static const std::map<double, uint8_t> sOCLookUp {
    {3.30, 0},   {3.50, 5},   {3.60, 10},  {3.65, 15},
    {3.68, 20},  {3.70, 25},  {3.72, 30},  {3.74, 35},
    {3.76, 40},  {3.78, 45},  {3.80, 50},  {3.83, 55},
    {3.85, 60},  {3.88, 65},  {3.92, 70},  {3.96, 75},
    {4.00, 80},  {4.06, 85},  {4.10, 90},  {4.15, 95},
    {4.20, 100}
};

// ===================================================
//           Template-Based Battery Class
// ===================================================

template <typename AdcDriver>
class Battery {
public:

    Battery(uint8_t nCells,
            BatteryType type,
            double scaleFactor,
            uint8_t pin,
            double weight = DEFAULT_WEIGHT)
        : _cells(nCells),
          _pin(pin),
          _weight(weight)
    {
        double maxCellV = (type == BatteryType::LIION)
                            ? LIION_MAX_CELL_VOLTAGE
                            : LIPO_MAX_CELL_VOLTAGE;

        // Initial assumption: battery is full
        _batteryVoltage = maxCellV * nCells;

        // Convert ADC raw values to volts
        _bitToVoltage = scaleFactor / static_cast<double>(AdcDriver::MAX_VALUE);
    }

    // -----------------------------------------
    // Update ADC reading and compute SOC
    // -----------------------------------------
    void update()
    {
        uint16_t raw = AdcDriver::read(_pin);

        double newVoltage =
            static_cast<double>(raw) * _bitToVoltage;

        // Low-pass filter
        _batteryVoltage =
            (newVoltage * _weight) +
            (_batteryVoltage * (1.0 - _weight));

        updatePercentage();
    }

    // -----------------------------------------
    // Getters (const, no side effects)
    // -----------------------------------------
    double getVoltage() const    { return _batteryVoltage; }
    double getPercentage() const { return _percentage;     }
    uint8_t getPin() const       { return _pin;            }

private:

    // -----------------------------------------
    // SOC lookup + linear interpolation
    // -----------------------------------------
    void updatePercentage()
    {
        double cellV = _batteryVoltage / _cells;

        auto it = sOCLookUp.lower_bound(cellV);

        if (it == sOCLookUp.begin()) {
            _percentage = it->second;
            return;
        }

        if (it == sOCLookUp.end()) {
            _percentage = std::prev(it)->second;
            return;
        }

        auto itLow  = std::prev(it);
        auto itHigh = it;

        double frac =
            (cellV - itLow->first) /
            (itHigh->first - itLow->first);

        _percentage =
            itLow->second +
            frac * (itHigh->second - itLow->second);
    }

private:
    uint8_t _cells;
    uint8_t _pin;
    double _batteryVoltage = 0.0;
    double _percentage     = 0.0;
    double _bitToVoltage   = 0.0;
    double _weight         = DEFAULT_WEIGHT;
};

// ===================================================
//             Example ADC Drivers
// ===================================================

// Arduino example driver
#ifdef ARDUINO
#include <Arduino.h>

struct ArduinoAdc {
    static constexpr uint16_t MAX_VALUE = 1023; // 10-bit ADC

    static uint16_t read(uint8_t pin) {
        return analogRead(pin);
    }
};
#endif

// ESP32 example driver (Arduino-ESP32)
#ifdef ESP32
#include <Arduino.h>

struct Esp32Adc {
    static constexpr uint16_t MAX_VALUE = 4095; // 12-bit ADC

    static uint16_t read(uint8_t pin) {
        return analogRead(pin);
    }
};
#endif

// Unit testing / simulation
struct FakeAdc {
    static constexpr uint16_t MAX_VALUE = 65535; // arbitrary

    static uint16_t read(uint8_t pin) {
        (void)pin;
        return 32768;
    }
};
