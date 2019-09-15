/*

HLW8012_ESP82
also works with BL0937 (requires calibration)

Copyright (C) 2019 by Jaromir Kopp <macwyznawca at me dot com>

Based on the library for Arduino created by: Xose Pérez <xose dot perez at gmail dot com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "HLW8012_ESP82.h"
#include "mem.h"
#include "gpio.h"
#include "gpios_id.h"

#include <math.h>

uint8_t _cf_pin;
uint8_t _cf1_pin;
uint8_t _sel_pin;

float _current_resistor = R_CURRENT;
float _voltage_resistor = R_VOLTAGE_HLW;
float _vref = V_REF_HLW;

float _current_multiplier; // Unit: us/A
float _voltage_multiplier; // Unit: us/V
float _power_multiplier;   // Unit: us/W

uint32_t _pulse_timeout = PULSE_TIMEOUT;    //Unit: us
volatile uint32_t _voltage_pulse_width = 0; //Unit: us
volatile uint32_t _current_pulse_width = 0; //Unit: us
volatile uint32_t _power_pulse_width = 0;   //Unit: us
volatile uint32_t _pulse_count = 0;

float _current = 0;
uint16_t _voltage = 0;
uint16_t _power = 0;

uint8_t _current_mode = 1;
uint8_t _model = 0;
volatile uint8_t _mode;

volatile uint32_t _last_cf_interrupt = 0;
volatile uint32_t _last_cf1_interrupt = 0;
volatile uint32_t _first_cf1_interrupt = 0;


LOCAL void HLW8012_intr_handler(void *arg);


void ICACHE_FLASH_ATTR HLW8012_checkCFSignal() {
    if ((system_get_time() - _last_cf_interrupt) > _pulse_timeout) 
        _power_pulse_width = 0;
}

void ICACHE_FLASH_ATTR HLW8012_checkCF1Signal() {
    if ((system_get_time() - _last_cf1_interrupt) > _pulse_timeout) {
        if (_mode == _current_mode) {
            _current_pulse_width = 0;
        } else {
            _voltage_pulse_width = 0;
        }
        HLW8012_toggleMode();
    }
}

// These are the multipliers for current, voltage and power as per datasheet
// These values divided by output period (in useconds) give the actual value
// For power a frequency of 1Hz means around 12W
// For current a frequency of 1Hz means around 15mA
// For voltage a frequency of 1Hz means around 0.5V
LOCAL void ICACHE_FLASH_ATTR _calculateDefaultMultipliers() {
    switch (_model){
    case 1:
        _power_multiplier =   (  50850000.0 * _vref * _vref * _voltage_resistor / _current_resistor / 48.0 / F_OSC_BL0) / 1.1371681416f;  //15102450
        _voltage_multiplier = ( 221380000.0 * _vref * _voltage_resistor /  2.0 / F_OSC_BL0) / 1.0474137931f; //221384120,171674
        _current_multiplier = ( 531500000.0 * _vref / _current_resistor / 24.0 / F_OSC_BL0) / 1.166666f; // 
        break;
    default:
        _power_multiplier =   ( 1000000.0 * 128 * _vref * _vref * _voltage_resistor / _current_resistor / 48.0 / F_OSC_HLW );
        _voltage_multiplier = ( 1000000.0 * 512 * _vref * _voltage_resistor /  2.0 / F_OSC_HLW );
        _current_multiplier = ( 1000000.0 * 512 * _vref / _current_resistor / 24.0 / F_OSC_HLW );
        break;
    }
}

float ICACHE_FLASH_ATTR HLW8012_getCurrentMultiplier() { return _current_multiplier; };
float ICACHE_FLASH_ATTR HLW8012_getVoltageMultiplier() { return _voltage_multiplier; };
float ICACHE_FLASH_ATTR HLW8012_getPowerMultiplier() { return _power_multiplier; };

void ICACHE_FLASH_ATTR HLW8012_setCurrentMultiplier(float current_multiplier) { _current_multiplier = current_multiplier; };
void ICACHE_FLASH_ATTR HLW8012_setVoltageMultiplier(float voltage_multiplier) { _voltage_multiplier = voltage_multiplier; };
void ICACHE_FLASH_ATTR HLW8012_setPowerMultiplier(float power_multiplier) { _power_multiplier = power_multiplier; };

void ICACHE_FLASH_ATTR HLW8012_init(uint8_t cf_pin, uint8_t cf1_pin, uint8_t sel_pin, uint8_t currentWhen, uint8_t model){
    _model = model;

    switch (_model) {
    case 1:
        _voltage_resistor = R_VOLTAGE_BL0;
        _vref = V_REF_BL0;
        break;
    
    default:
        _voltage_resistor = R_VOLTAGE_HLW;
        _vref = V_REF_HLW;
        break;
    }

    _cf_pin = cf_pin;
    _cf1_pin = cf1_pin;
    _sel_pin = sel_pin;
    _current_mode = currentWhen;

    ETS_GPIO_INTR_ATTACH(HLW8012_intr_handler, NULL);

    ETS_GPIO_INTR_DISABLE();

    PIN_FUNC_SELECT(get_pin_mux(_sel_pin), get_pin_func(_sel_pin));
    PIN_FUNC_SELECT(get_pin_mux(_cf_pin), get_pin_func(_cf_pin));
    PIN_FUNC_SELECT(get_pin_mux(_cf1_pin), get_pin_func(_cf1_pin));

    GPIO_DIS_OUTPUT(_cf_pin);
    PIN_PULLUP_EN(get_pin_mux(_cf_pin));
    GPIO_DIS_OUTPUT(_cf1_pin);
    PIN_PULLUP_EN(get_pin_mux(_cf1_pin));

    //gpio_output_set(0, 0, (BIT0 << _sel_pin), 0);

    gpio_register_set(GPIO_PIN_ADDR(_cf_pin),
                        GPIO_PIN_INT_TYPE_SET(GPIO_PIN_INTR_DISABLE)
                      | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_DISABLE)
                      | GPIO_PIN_SOURCE_SET(GPIO_AS_PIN_SOURCE));

    gpio_register_set(GPIO_PIN_ADDR(_cf1_pin),
                        GPIO_PIN_INT_TYPE_SET(GPIO_PIN_INTR_DISABLE)
                      | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_DISABLE)
                      | GPIO_PIN_SOURCE_SET(GPIO_AS_PIN_SOURCE));

    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(_cf_pin));
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(_cf1_pin));
    
    gpio_pin_intr_state_set(GPIO_ID_PIN(_cf_pin),GPIO_PIN_INTR_NEGEDGE);
    gpio_pin_intr_state_set(GPIO_ID_PIN(_cf1_pin),GPIO_PIN_INTR_NEGEDGE);
    
    _calculateDefaultMultipliers();

    _mode = _current_mode;
    GPIO_OUTPUT_SET(_sel_pin, _mode);

    ETS_GPIO_INTR_ENABLE();
}

void ICACHE_FLASH_ATTR HLW8012_setMode(hlw8012_mode_t mode) {
    _mode = (mode == MODE_CURRENT) ? _current_mode : 1 - _current_mode;
    GPIO_OUTPUT_SET((_sel_pin), _mode);
    
    _last_cf1_interrupt = _first_cf1_interrupt = system_get_time();
   
}

hlw8012_mode_t ICACHE_FLASH_ATTR HLW8012_getMode() {
    return (_mode == _current_mode) ? MODE_CURRENT : MODE_VOLTAGE;
}

hlw8012_mode_t ICACHE_FLASH_ATTR HLW8012_toggleMode() {
    hlw8012_mode_t new_mode = HLW8012_getMode() == MODE_CURRENT ? MODE_VOLTAGE : MODE_CURRENT;
    HLW8012_setMode(new_mode);
    return new_mode;
}

uint16_t ICACHE_FLASH_ATTR HLW8012_getCurrent() {

    // Power measurements are more sensitive to switch offs,
    // so we first check if power is 0 to set _current to 0 too

    HLW8012_getActivePower();

    if (_power == 0) {
         _current_pulse_width = 0;

    } else {
         HLW8012_checkCF1Signal();
    }
    _current = (_current_pulse_width > 0) ? _current_multiplier / _current_pulse_width  : 0;
    return (uint16_t)(_current * 100);

}

uint16_t ICACHE_FLASH_ATTR HLW8012_getVoltage() {
    HLW8012_checkCF1Signal();
    
    _voltage = (_voltage_pulse_width > 0) ? _voltage_multiplier / _voltage_pulse_width : 0;
    return _voltage;
}

uint32_t ICACHE_FLASH_ATTR HLW8012_getEnergy() {

    /*
    Pulse count is directly proportional to energy:
    P = m*f (m=power multiplier, f = Frequency)
    f = N/t (N=pulse count, t = time)
    E = P*t = m*N  (E=energy)
    */
    return _pulse_count * _power_multiplier / 1000000l;
}

uint16_t ICACHE_FLASH_ATTR HLW8012_getActivePower() {

    HLW8012_checkCFSignal();

    _power = (_power_pulse_width > 0) ? _power_multiplier / _power_pulse_width : 0;
    return _power;
}

uint16_t ICACHE_FLASH_ATTR HLW8012_getApparentPower() {
    float current = HLW8012_getCurrent();
    uint16_t voltage = HLW8012_getVoltage();
    return voltage * current;
}


float ICACHE_FLASH_ATTR HLW8012_getPowerFactor() {
    uint16_t active = HLW8012_getActivePower();
    uint16_t apparent = HLW8012_getApparentPower();
    if (active > apparent) return 1;
    if (apparent == 0) return 0;
    return (float) active / apparent;
}

void ICACHE_FLASH_ATTR HLW8012_resetEnergy() {
    _pulse_count = 0;
}

void ICACHE_FLASH_ATTR HLW8012_expectedCurrent(float value) {
    if (_current == 0) HLW8012_getCurrent();
    if (_current > 0) _current_multiplier *= (value / _current);
}

void ICACHE_FLASH_ATTR HLW8012_expectedVoltage(uint16_t value) {
    if (_voltage == 0) HLW8012_getVoltage();
    if (_voltage > 0) _voltage_multiplier *= ((float) value / _voltage);
}

void ICACHE_FLASH_ATTR HLW8012_expectedActivePower(uint16_t value) {
    if (_power == 0) HLW8012_getActivePower();
    if (_power > 0) _power_multiplier *= ((float) value / _power);
}

void ICACHE_FLASH_ATTR HLW8012_resetMultipliers() {
    _calculateDefaultMultipliers();
}

void ICACHE_FLASH_ATTR HLW8012_setResistors(float current, float voltage_upstream, float voltage_downstream) {
    if (voltage_downstream > 0) {
        _current_resistor = current;
        _voltage_resistor = (voltage_upstream + voltage_downstream) / voltage_downstream;
        _calculateDefaultMultipliers();
    }
}

LOCAL void  HLW8012_cf_interrupt(void) {
    uint32_t now = system_get_time();
    _power_pulse_width = now - _last_cf_interrupt;
    _last_cf_interrupt = now;
    _pulse_count++;
}

LOCAL void  HLW8012_cf1_interrupt(void) {

    uint32_t now = system_get_time();

    if ((now - _first_cf1_interrupt) > _pulse_timeout) {

        uint32_t pulse_width;
        
        if (_last_cf1_interrupt == _first_cf1_interrupt) {
            pulse_width = 0;
        } else {
            pulse_width = now - _last_cf1_interrupt;
        }

        if (_mode == _current_mode) {
            _current_pulse_width = pulse_width;
        } else {
            _voltage_pulse_width = pulse_width;
        }

        _mode = 1 - _mode;
        
        GPIO_OUTPUT_SET((_sel_pin), _mode);
        _first_cf1_interrupt = now;
    }

    _last_cf1_interrupt = now;
}

LOCAL void HLW8012_intr_handler(void *arg){
    uint32 gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);

    if (gpio_status & BIT(_cf_pin)) {
        HLW8012_cf_interrupt();
        //disable interrupt
        gpio_pin_intr_state_set(GPIO_ID_PIN(_cf_pin), GPIO_PIN_INTR_DISABLE);
        //clear interrupt status
        GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT(_cf_pin));
        gpio_pin_intr_state_set(GPIO_ID_PIN(_cf_pin), GPIO_PIN_INTR_NEGEDGE);
    }
    if (gpio_status & BIT(_cf1_pin)) {
        HLW8012_cf1_interrupt();
        //disable interrupt
        gpio_pin_intr_state_set(GPIO_ID_PIN(_cf1_pin), GPIO_PIN_INTR_DISABLE);
        //clear interrupt status
        GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT(_cf1_pin));
        gpio_pin_intr_state_set(GPIO_ID_PIN(_cf1_pin), GPIO_PIN_INTR_NEGEDGE);
    }
}



