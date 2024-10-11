/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Data Acquisition

 * This file contains methods used for the car's VCU communication with the TCU, BMS and all
 * other low voltage CAN connected devices
 ***/
#ifndef _GENERAL_CAN_H_
#define _GENERAL_CAN_H_

// CAN DEBUG MODE:
#define DEBUG_CAN2

#ifdef DEBUG_CAN2
    #define DEBUG_PRINT_CAN2(...) printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT_CAN2(...)
#endif

#include "mbed.h"
#include "CAN.h"
#include "MotorCAN.h"
#include <cstdint>

/*Struct that contains BMS Datafield*/
struct BMSData{

};

/*Struct that contains BMS Datafield*/
struct TCUDataRx{

};
/*Struct that contains BMS Datafield*/
struct TCUDataTx{

};

/*================================== CLASS ==================================*/
class GeneralCAN:public CAN{

    //Methods
    public:
    void set_CAN();
    void reset_can();
    
    //Send data to the TCU
    void send_to_TCU(RxStruct Motor_1, RxStruct Motor_2);
    
    // Receive data from both motor controllers

    //print received data
    
    //Constructors
    public:
    GeneralCAN(PinName _can_pin_rx, PinName _can_pin_tx, uint32_t _can_frequency);
};

#endif