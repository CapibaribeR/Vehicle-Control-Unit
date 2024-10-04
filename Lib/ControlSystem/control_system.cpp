#include "control_system.h"
#include <cstdint>
#include <stdint.h>

/*========================================= ELECTRONIC DIFFERENTIAL =========================================*/
// Calculates the Velocity [in Rad/s] of each wheel during a turn, W1 = [LEFT WHEEL] || W2 = [RIGHT WHEEL]
void OpenLoopDifferential(float Steering_dg, uint16_t Apps, uint16_t Wm_ref[]){
    int32_t W1_ref,W2_ref, Wv_ref;      // 32bit Aux Variables, to prevent overflow
    uint16_t Steering{0};
    float STREERING_TO_ACK{0};
    float Ack_rad;
    float del_W;

    // Angular Velocity based on the Acc. Pedal
    Wv_ref= Apps;
    
    // For very low angles in the Steering wheel, there's no change
    if( abs(Steering_dg)< 5){
        Steering_dg=0;
    }

    // Get Ackerman Angle [in rad] using the steering Wheel rotation [in Degrees] 
    // Ack_rad= Steering*STREERING_TO_ACK;
    Ack_rad= (0.14 * Steering_dg)* PI/180;

    // Velocity Variation during turn
    del_W= Wv_ref * K_DIF * tan(Ack_rad);

    if(del_W>DEL_W_MAX){
        del_W = DEL_W_MAX;
    }

    // Calculate differential [in Rad/s] and turns into RPM (Note: there's a Implicit Type Conversion there)
    W1_ref = Wv_ref + del_W;
    W2_ref = Wv_ref - del_W;

    //if overflow, sets max
    if(W1_ref>UINT16_MAX){
        W1_ref=UINT16_MAX;
    }
    if(W2_ref>UINT16_MAX){
        W2_ref=UINT16_MAX;
    }

    // With Overflow prevented, saves the values
    Wm_ref[0]= uint16_t(W1_ref);
    Wm_ref[1]= uint16_t(W2_ref);
}


void PrintDifferential(uint16_t Apps_1, float Steering_dg, uint16_t Wm_ref[]){
    double Pedal=(float(Apps_1)/65.535);
    double Pm1=(float(Wm_ref[0])/65.535);
    double Pm2=(float(Wm_ref[1])/65.535);
    
    printf("\n[Electronic Differential]  ==================================================#");
    printf("\n Pedal_Travel: %.2f%%,  Steering Angle : %.2f°\n", Pedal,Steering_dg);
    printf("\n Dc_1: %.2f %%,  Dc_2: %.2f %%\n", Pm1,Pm2);
    printf("#============================================================================#\n");
}

