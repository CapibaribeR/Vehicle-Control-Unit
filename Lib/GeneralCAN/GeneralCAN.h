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




/*Struct that contains BMS Datafield*/
struct BMSData{



};


/*Struct that contains BMS Datafield*/
struct TCUDataRx{



};
/*Struct that contains BMS Datafield*/
struct TCUDataTx{



};






#endif