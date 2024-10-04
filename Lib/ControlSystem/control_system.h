/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain

 * File for the Digital (and Differential) Control of the BLDC Nova 15 Motor 
 * using the MST 400-200 Motor controller

 ***/

#ifndef _CONTROL_SYSTEM_H_
#define _CONTROL_SYSTEM_H_

#include "mbed.h"
#include <cstdint>


/*==================================== CONTROL PARAMETERS ====================================*/
#define PI  3.14159265358979323846                  // PI constant 
const uint16_t MAX_RPM  =   8000;                   // PI constant

/*==================================== MECHANIC PARAMETERS ====================================*/
const float TRACK_WIDTH =  1.4;                             // [m] "Bitola" distance between the wheels on the same axle
const float WHEELBASE   =  1.8;                             // [m] Distance between the front and rear axles           
const float K_DIF       =  TRACK_WIDTH / ( 2*WHEELBASE );   // [adm] constant for Differential
const uint16_t DEL_W_MAX = 10000;


/*======================================== Aux Functions ========================================*/
void OpenLoopDifferential(float Steering_dg, uint16_t Apps, uint16_t Dc_Motor[]);   // Diff in Open Loop
void Get_Differential(float Steering_dg, uint16_t Apps, uint16_t Wm_ref[]);

#endif