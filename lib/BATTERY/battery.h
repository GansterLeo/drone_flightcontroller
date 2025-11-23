#pragma once 
#include <stdint.h>

#define LIION_MAX_CELL_VOLTAGE 4.2
#define LIPO_MAX_CELL_VOLTAGE  4.2
#ifndef ADC_FULL_SCALE
#define ADC_FULL_SCALE 4096
#endif
#define ADC_VOLTAGE 3.3
class Battery{
    public:
    enum BatteryType{
        BATTERY_LIION,
        BATTERY_LIPO
    };
    /*
        @param:
            scaleFactor ... factor by which the battery voltage is scaled.
                            e.g.: 2   --> batteryVoltage is doubled
                                  0.5 --> batteryVoltage is halfed
    */
    Battery(uint8_t nOfCells, BatteryType type, double scaleFactor, uint8_t pinNumber, uint16_t(*analogRead)(uint8_t pinNumber));

    /*
        @return:
                    0 ... everything went fine 
                    1 ... batteryType not registered
    */
    int8_t setVoltageFactor(double scaleFactor);
    /*
        returns the scale factor as double
    */
    double getPercentageFactor() const;
    /*
        returns the battery voltage since its last update
    */
    double getVoltage() const;
    /*
        updates the percentage variable with the real battery percentage
    */
    double getPercentage() const;
    
    void    update();
    void    setADCPin(uint8_t pin);
    uint8_t getADCPin() const;

    private:
    uint8_t _pin;
    double _bitToVoltage;
    uint8_t _nOfCells;
    double _batteryVoltage;
    double _percentage;
    float  _weight = 0.2;
    uint16_t (*_analogRead)(uint8_t pinNumber);
};