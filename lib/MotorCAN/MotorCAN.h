// /***
//  * Capibarib E-racing
//  * Federal University of Pernambuco (UFPE)
//  * Group Area: Powertrain

//  * This file contains methods used for the car's embedded system's communication
//  * The datafield follows the standard for CAN 2.0 communication as described in the
//  * Plettenberg documentation (for MST motor controllers)
//  ***/
#ifndef _MOTOR_CAN_H_
#define _MOTOR_CAN_H_

#include "mbed.h"
#include "CAN.h"
#include <cstdint>

// /*================================== COMMUNICATION PARAMETERS ==================================*/
#define CONTROLLER_TX_ID      0x100
#define CONTROLLER_RX_ID      0x101

#define CONTROLLER_TX_ID_2    0x200
#define CONTROLLER_RX_ID_2    0x201

#define MAX_CAN_DATA_SIZE   8
#define MOTOR_POLE_PAIRS    15          //Number of Pole Pairs in the Motor


#define MAXRPM 9000
#define MAXPWM 65535


/*==================================== SAFETY PARAMETERS ====================================*/
#define MAX_CURRENT_LIMIT       38          // Max Phase Current in the motor [A]
#define MAX_RPM_LIMIT           9000        // Max Velocity of the motor [RPM] 

/*==================================== SAFETY PARAMETERS ====================================*/
#define MAX_TEMP_CONTROLLER      80                          // Motor Controller Max temperature [°C]
#define MAX_TEMP_MOTOR           80                          // Motor Max temperature [°C]

/*===================================== COMMUNICATION PORTS (STM32 F746ZG) ==================*/
// CAN 1: General communication in the VCU
#define CAN1_RX                 PD_0
#define CAN1_TX                 PD_1

// CAN 2: Communication with the motor controller
#define CAN2_RX                 PB_5
#define CAN2_TX                 PB_6

// CAN frequency in Hz (bit/s)
#define CAN1_FREQUENCY           1e6
#define CAN2_FREQUENCY           1e6        // for CAN 2.0, its 1MHz [1Mbit/s] [Mandatory]






/*================================== Receive Struct ==================================*/
struct RxStruct{
    //Receiver Datafield in CAN 2.0 standard (In order)
    uint8_t     Msg_Counter{0};             //
    float       Supply_Voltage{0};          // [V]
    int16_t     Temp_Controller{0};         // [°C]
    int16_t     Temp_motor{0};              // [°C]
    uint16_t    RPM{0};                     // [RPM]
    float       PWM_read{0};                // 0 - 100 [%]
    uint8_t     Current{0};                 // [A]
};

/*================================== Send Struct ==================================*/
struct TxStruct{
    //Transceiver Datafield in CAN 2.0 standard (In order)
    uint16_t    RPM_Limit{0};
    // motor pole pair is a constant
    uint16_t    Tx_PWM{0};
    uint16_t    Current_Limit{0};
    bool        isBreak{0};         //1= break, 0= Throttle
    bool        isReverse{0};       //1= Reverse, 0= Forward
};

// Aux 
bool Temperature_Shutdown(RxStruct Controller_1, RxStruct Controller_2);
void Print_Datafield(int Num,RxStruct Motor_Data);

/*================================== CLASS ==================================*/
class MotorCAN:public CAN{

    private:
    RxStruct Datafield_Controller_1; //saves the datafield received from Inverter 1
    RxStruct Datafield_Controller_2; //saves the datafield received from Inverter 2

    //Methods
    public:
    void set_CAN();
    void reset_can();
    bool baud_test();
    bool is_can_active();
    void receive();
    
    //Send data to both motor Controllers
    void send_to_controller(unsigned int Motor_Id, uint16_t DC_pwm_1);
    void send_to_controller_1(uint16_t DC_pwm_1);
    void send_to_controller_2(uint16_t DC_pwm_2);
    void attach2(RxStruct Controller_Data_1, RxStruct Controller_Data_2);
    
    // Receive data from both motor controllers
    void receive_from_controller();
    // RxStruct receive_from_inverter(unsigned int Inverter_Id);
    RxStruct receive_from_controller_1();
    RxStruct receive_from_controller_2();
    


    //print received data
    void Print_Datafields();
    void Print_Datafield(int Num, RxStruct Inv);
    
    //Constructors
    public:
    MotorCAN(PinName _can_pin_rx, PinName _can_pin_tx, uint32_t _can_frequency);
};

    RxStruct receive_from_controller(unsigned int Inverter_Id);
    RxStruct receive_from_controller_1();
    RxStruct receive_from_controller_2();
    



#endif