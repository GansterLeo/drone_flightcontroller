#include "battery.h"
#include <map> 

std::map<double, uint8_t> sOCLookUp {
    {3.30, 0},
    {3.50, 5},
    {3.60, 10},
    {3.65, 15},
    {3.68, 20},
    {3.70, 25},
    {3.72, 30},
    {3.74, 35},
    {3.76, 40},
    {3.78, 45},
    {3.80, 50},
    {3.83, 55},
    {3.85, 60},
    {3.88, 65},
    {3.92, 70},
    {3.96, 75},
    {4.00, 80},
    {4.06, 85},
    {4.10, 90},
    {4.15, 95},
    {4.20, 100}
};

Battery::Battery(uint8_t nOfCells, BatteryType type, double scaleFactor, uint8_t pinNumber, uint16_t(*analogReadFunc)(uint8_t pinNumber)) : _analogRead(analogReadFunc) {
    _nOfCells = nOfCells;
    switch(type){
        case BATTERY_LIION: _batteryVoltage=LIION_MAX_CELL_VOLTAGE*nOfCells; break;
        case BATTERY_LIPO:  _batteryVoltage=LIPO_MAX_CELL_VOLTAGE*nOfCells;  break;
        default:            _batteryVoltage=0;                               break;
    }
    setVoltageFactor(scaleFactor);
    _pin = pinNumber;
}

int8_t Battery::setVoltageFactor(double scaleFactor){
    _bitToVoltage = ADC_VOLTAGE*scaleFactor/ADC_FULL_SCALE;
    return 0;
}

double Battery::getPercentageFactor() const {
    return _bitToVoltage;
}

void Battery::update(){
    _batteryVoltage = (_analogRead(_pin) * _bitToVoltage * _weight) + (_batteryVoltage*(1-_weight));

    double meanCellvoltage = _batteryVoltage/_nOfCells;
    
    auto it = sOCLookUp.lower_bound(meanCellvoltage);

    // Case 1: voltage is below the lowest key
    if (it == sOCLookUp.begin())    _percentage = it->second;
    // Case 2: voltage is above the highest key
    else if (it == sOCLookUp.end()) _percentage = std::prev(it)->second;
    // Otherwise interpolate between *it (upper) and *(it-1) (lower)
    else {
        auto itHigh = it;
        auto itLow  = std::prev(it);
        // Linear interpolation
        double dVoltage = (meanCellvoltage - itLow->first) / (itHigh->first - itLow->first);
        _percentage     = itLow->second + dVoltage * (itHigh->second - itLow->second);
    }
}

double Battery::getVoltage() const {
    return _batteryVoltage;
}

double Battery::getPercentage() const {
    return _percentage;
}

void Battery::setADCPin(uint8_t pin){
    _pin = pin;
}

uint8_t Battery::getADCPin() const {
    return _pin;
}

