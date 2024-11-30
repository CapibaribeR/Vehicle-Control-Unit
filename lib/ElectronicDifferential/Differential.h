/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain

 * File for the Electronic Differential of the BLDC Nova 15 Motor 
 * using the MST 400-200 Motor controller

 ***/
#ifndef _ELECTRONIC_DIFFERENTIAL_
#define _ELECTRONIC_DIFFERENTIAL_

#include "mbed.h"
#include <cstdint>
#include <stdint.h>

/*==================================== CONTROL PARAMETERS ====================================*/
// #define PI  3.14159265358979323846                  // PI constant 
// const uint16_t MAX_RPM  =   8000;                   // PI constant

/*==================================== MECHANIC PARAMETERS ====================================*/
const float TRACK_WIDTH =  1.25;                             // [m] "Bitola" distance between the wheels on the same axle
const float WHEELBASE   =  1.6;                             // [m] Distance between the front and rear axles           
const float K_DIF       =  TRACK_WIDTH / ( 2*WHEELBASE );   // [adm] constant for Differential
const uint16_t DEL_W_MAX = 10000;


/*======================================== Aux Functions ========================================*/
void OpenLoopDifferential(float Steering_rad, uint16_t Apps, uint16_t Wm_ref[]);    // Diff in Open Loop
void Differential(float Steering_rad, uint16_t Apps, uint16_t RPM_ref[]);           // Diff in Open Loop
void PrintDifferential(uint16_t Apps_1, float Steering_dg, uint16_t Wm_ref[]);      //

#endif