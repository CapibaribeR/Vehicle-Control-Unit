/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain
 
 * This file is part of the car's VCU (VEHICLE CONTROL UNIT)
 * it contains the APPS plausibilty check, BSE plausibility check
 * as well the methods to read and map all the angle related sensors connected to the adc
 ***/

#ifndef _ANALOG_SENSOR_H_
#define _ANALOG_SENSOR_H_

#include "mbed.h"
#include <cstdint>
#include <time.h>

/*================================== SENSORS PARAMETERS ==================================*/
//General ADC Parameters 
#define VREF_ADC                3.3         // ADC Reference (in volts), it scales the 16bit in that range
#define VOLT_MIN                0.3         // Minimum voltage Input the sensor should read
#define VOLT_MAX                3.2         // Maximum voltage Input the sensor should read

#define INPUT_MIN_              0.3         // Minimum 16bit Input the sensor should read
#define INPUT_MAX_              3.2         // Maximum 16bit Input the sensor should read

#define MAX_NOISE               0.09        // Expected Noise [Voltage variation] in ADC read
#define SATURATION_VOLTAGE      0.1         // Saturation 

//Steering Wheel Parameters
#define STEERING_MIN            -0.7        // Minimum value for the Steering Wheel angle (rad)
#define STEERING_MAX             0.7        // Maximum value for the Steering Wheel angle (rad)

// Pedal Parameters
#define PEDAL_MIN               0           // Minimum value for the Accelerator Pedal angle (Degrees)
#define PEDAL_MAX               100         // Maximum value for the Accelerator Pedal angle (Degrees)

//Brake System Parameters
#define MIN_BRAKE_VOLT          1.0         // Minimum Voltage to Turn on Brake light                
#define BRAKE_LIGHT_PIN         PF_13       // Brake Light Output Pin


//APPS (Accelerator Pedal Position Sensor) Parameters


//Ultility Functions
unsigned long current_ms();
float map(float Variable, float in_min, float in_max, float out_min, float out_max);
uint16_t map_u16 (float Variable, float in_min, float in_max, uint16_t out_min, uint16_t out_max);

// Safety Checks
bool APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2, uint8_t* Error_Count);  // Acc. pedal Plausibility
bool BSE_Error_check(uint16_t Apps_val, uint16_t Brake_val, uint8_t* Error_BPPC);   // Brake pedal Plausibility

/*================================== Angle Sensors ==================================*/
//class for Acelleration Pedal, brake Pedal, and Steering wheel
class AnalogSensor{
    //Atributes
    protected:
    AnalogIn ADC_Pin;           // Input Pin in the MicroController

    float Angle{0};             // Angle's value in Degree
    float Current_ADC{0};       // Last ADC voltage read [V]
    bool Circuit_ERROR{0};      // Flag for short or open circuit
    
    //Methods
    public:
    float read_steering();                             // returns scaled angle
    bool Circuit_Error_Check(float voltage_in);     // tests if ADC voltage is within bounds of sensor
    bool get_circuit_error();                       // Returns Circuit Error
    void Voltage_print();                           // prints pin's voltage


    //Constructors:
    AnalogSensor(PinName adc_Pin);
};

/*====================================== Pedal Sensors ======================================*/
// Pedal Sensor Class (for the Accel. and brake Pedals)
class PedalSensor: public AnalogSensor{
    // Attributes:
    private:
    uint16_t Pedal_pos;                         // Pedal Travel [0% = 0 | 100% = 16b]
    DigitalOut BRAKE_LIGT{BRAKE_LIGHT_PIN};     //

    // Methods:
    public:
    uint16_t read_pedal();      // Reads current Pedal Position
    float read_brake();         // Reads Brake 
    void Voltage_print();       // Print ADC voltage read and Pedal Travel
    
    // Constructors:
    PedalSensor(PinName adc_Pin);
};

#endif