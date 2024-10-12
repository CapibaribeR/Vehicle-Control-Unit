#include "AnalogSensor.h"
#include <cstdint>

/*================================== ACCELERATION PEDAL PLAUSIBILITY CHECK ==================================*/
// Every 10ms, checks if there's a discrepancy bigger than 10% lastting more then 100 ms
bool APPS_Error_check(uint16_t Apps_1, uint16_t Apps_2, uint8_t* Error_Count){
    uint8_t Error_Counter = *Error_Count;

    // Checks 10% discrepancy
    if ( abs(Apps_1 - Apps_2) > (0.1 * max(Apps_1, Apps_2)) ){
        // if there's discrepancy, adds in counter
        Error_Counter++;
    }
    else { 
        Error_Counter= 0;      //if error ceased resets counter
    }
    
    // If Implausibility lasts 5 iterations (100ms - 120 ms response time), Sets Error
    if(Error_Counter >=5){
        printf("APPS ERROR DETECTED");
        Error_Counter =4;
        return 1;
    }
    else{
        return 0;
    }

}

/*====================================== BRAKE PEDAL PLAUSIBILITY CHECK ======================================*/
// Checks if the Accel. and Brake Were both pressed at the same time
bool BSE_Error_check(uint16_t Apps_val, uint16_t Brake_val, uint8_t *flag_BPPC){    
    bool Error_BPPC = *flag_BPPC;

    //If APPS >= 25% of pedal travel and Brake is pressed, stops the car 
    if ( (Apps_val >= 0.25*UINT16_MAX) && (Brake_val >= 0.03*UINT16_MAX) ){
        Error_BPPC = 1;
    }

    // Error only Stops if APPS goes below 5% of pedal travel
    if(Apps_val < 0.05*UINT16_MAX){
        Error_BPPC = 0;
    }

    if(Error_BPPC){
        printf("\nBSE: ERROR IN PROGRESS");
    }
    
    *flag_BPPC = Error_BPPC;
    return Error_BPPC;
}


/*================================== Angle Sensors ==================================*/
// Constructors
AnalogSensor::AnalogSensor(PinName adc_Pin)
    :ADC_Pin{adc_Pin,VREF_ADC}
    { ADC_Pin.set_reference_voltage(3.3); };


// Methods
/* Reads the ADC pin and returns the angle value in degrees */ 
float AnalogSensor:: read_steering(){
    float New_ADC = ADC_Pin.read_voltage();
    
    // Tests if ADC voltage read is within the sensor's bounds [short or open circuit]
    if(New_ADC<VOLT_MIN || New_ADC>VOLT_MAX){
        printf("\nCIRCUIT: ERROR DETECTED\n");
        Circuit_ERROR=1;
    }
    else{
        Circuit_ERROR=0;
    }

    // If variation is bigger than the expected noise, updates measurement 
    if(abs(New_ADC - Current_ADC) > MAX_NOISE){
        Current_ADC= New_ADC;

        // Left Turn [0.3V - 1.6V]
        if(New_ADC >=0.3 || New_ADC <= 1.6){
            Angle = (New_ADC-VOLT_MIN)*STEERING_MIN/1.3;
        }

        // For low angles, it's zero [1.6V - 2.0v]
        if(New_ADC > 1.6 || New_ADC <= 2){
            Angle = 0;
        }   

        // Right Turn [2.0V - 3.3V]
        if(New_ADC>2){
            Angle = (New_ADC-2)*STEERING_MAX/1.3;
        }  
    
    } 

    return Angle;
}


/*Reads Brake Pedal*/
float PedalSensor:: read_brake(){
    float New_ADC = ADC_Pin.read_voltage();
    
    // Tests if ADC voltage read is within the sensor's bounds [short or open circuit]
    if(New_ADC<VOLT_MIN || New_ADC>VOLT_MAX){
        printf("\nCIRCUIT: ERROR DETECTED\n");
        Circuit_ERROR=1;
    }
    else{
        Circuit_ERROR=0;
    }

    // If variation is bigger than the expected noise, updates measurement 
    if(abs(New_ADC - Current_ADC) > MAX_NOISE){
        Current_ADC= New_ADC;
        Angle= map(Current_ADC, VOLT_MIN, VOLT_MAX, 0, 100);
    }

    // Turn Brake Light on or off
    if(Current_ADC > MIN_BRAKE_VOLT){
           BRAKE_LIGT.write(1);
    }
    else{
           BRAKE_LIGT.write(0);
    }
    return Angle;
}

/*Returns if the circuit is short circuited */
bool AnalogSensor::get_circuit_error(){      
    return Circuit_ERROR;
}

/* Prints voltage read and 16b */
void AnalogSensor:: Voltage_print(){
    printf("\n[VCU] ADC: Voltage_Read[16bit]: %d , Voltage[V]: %.2f V ",ADC_Pin.read_u16(), ADC_Pin.read_voltage());    
    printf("Angle: %.2f\n", Angle);
}


/*====================================== Pedal Sensors ======================================*/
// Constructors
PedalSensor::PedalSensor(PinName adc_Pin)
    :AnalogSensor{adc_Pin}{}

// Methods
/* Reads the Pedal travel [0% = 0 | 100% = 16b] */
uint16_t PedalSensor:: read_pedal(){
    float New_ADC = ADC_Pin.read_voltage();

    if(New_ADC<VOLT_MIN || New_ADC>VOLT_MAX){
        
    }
    // printf("\n NEW: %.2f , Current: %.2f, dif: %.2f\n",New_ADC, Current_ADC,abs(New_ADC - Current_ADC));
    // If variation is bigger than the expected noise, updates measurement 
    if(abs(New_ADC - Current_ADC) > MAX_NOISE){
        Current_ADC= New_ADC;
        Pedal_pos= map_u16(Current_ADC, VOLT_MIN, VOLT_MAX, PEDAL_MIN, UINT16_MAX);
    }

    if(Current_ADC< 0.8){
        Pedal_pos =0;
    }

    if(Current_ADC > 3.1){
        Pedal_pos = UINT16_MAX;
    }

    return Pedal_pos;
}

/* Print ADC voltage and Pedal Travel */
void PedalSensor:: Voltage_print(){
    uint16_t Voltage_16bit=ADC_Pin.read_u16();
    float Percentage=(float(Voltage_16bit)/65535)*100;

    printf("\n[VCU] ADC: Voltage_Read[16bit]: %d , Voltage[V]: %.2f V ",Voltage_16bit, ADC_Pin.read_voltage() );    
    printf("\nPedal Pressed: [%%]: %.2f %%\n",Percentage);
}





/*======================================== Auxiliar functions ========================================*/
//time passed (in ms) since the program first started
inline unsigned long current_ms(){
    using namespace std::chrono;
    auto now_us = time_point_cast<microseconds>(Kernel::Clock::now());      //time of referece= program's begin
    unsigned long micros = now_us.time_since_epoch().count();               //time (in us) since the reference 
    return micros / 1000.0;                                                 //turns time passed from us to ms
}

//Maps the ADC float voltage read into the angle's range 
inline float map (float Variable, float in_min, float in_max, float out_min, float out_max) {
    float Mapped_Variable = (Variable - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    
    // Saturation + Out of bounds variable
    if (Variable >= in_max ) {
        Mapped_Variable= out_max;    
    }
    if (Variable <= (in_min + SATURATION_VOLTAGE) ) {
        Mapped_Variable= out_min;    
    }
    return Mapped_Variable;
}

//Maps the ADC float voltage read into 16bit 
inline uint16_t map_u16 (float Variable, float in_min, float in_max, uint16_t out_min, uint16_t out_max) {
    uint16_t Mapped_Variable = (Variable - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    
    // Saturation + Out of bounds variable
    if (Variable >= (in_max - SATURATION_VOLTAGE) ) {
        Mapped_Variable= out_max;    
    }
    
    // If it's close to min voltage, its the min voltage
    if (Variable <= (in_min + SATURATION_VOLTAGE) ) {
        Mapped_Variable= out_min;    
    }
    
    return Mapped_Variable;
}

