#include "GeneralCAN.h"

/*Sends Message to the TCU*/
void GeneralCAN::send_to_TCU(RxStruct Motor_1, RxStruct Motor_2){
    uint8_t Input_Voltage = uint8_t(Motor_1.Supply_Voltage*10);

    /* Message 1: Motor and Controller Status*/
    CANMessage Msg;
    Msg.id=1;
    Msg.len=8;
    
    Msg.data[0] = Input_Voltage & 0xFF;     // LSB Input Voltage (V)
    Msg.data[1] = Input_Voltage >> 8;       // MSB Input Voltage (V)
    
    Msg.data[2] = Motor_1.Temp_motor;       // Motor 1 Temperature (°C)
    Msg.data[3] = Motor_2.Temp_motor;       // Motor 2 Temperature (°C)
    
    Msg.data[4] = Motor_1.Current;          // Motor 1 Phase Current
    Msg.data[5] = Motor_2.Current ;         // Motor 2 Phase Current
    
    Msg.data[6] = Motor_1.RPM & 0xFF;       // LSB RPM Motor 1
    Msg.data[7] = Motor_1.RPM >> 8;         // MSB RPM Motor 2

    //if message was not sent as it should, resets CAN
    if(!write(Msg) ) {
        DEBUG_PRINT_CAN2("\n[CAN TCU]: Not sent");
        reset_can();
    }else{
        DEBUG_PRINT_CAN2("\n[CAN TCU]: SENT");
    }

    /* Message 2: Gyro and Acelerometer*/

}