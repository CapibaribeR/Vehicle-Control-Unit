#include "mbed.h"

void getAccel();
void getGyro();

void getAccel(float Accel_Array[3]){
    float ax,ay,az;


    // Update Accel. Array
    Accel_Array[0] = ax;
    Accel_Array[1] = ay;
    Accel_Array[2] = az;
}

void getGyro(float Gyro_Array[3]){
    float gx,gy,gz;

    // Update Accel. Array
    Gyro_Array[0] = gx;
    Gyro_Array[1] = gy;
    Gyro_Array[2] = gz;
}

// Gets Gyro + Acc data and returns them as uint8 array
void getMsg(uint8_t Msg[6]){
    uint8_t Gx,Gy,Gz;
    uint8_t Ax,Ay,Az;

    // Get Gyro 

    // Get Accel.    

    // Saves Gyro and Accel as uint8 array
    Msg[0] = Gx;
    Msg[1] = Gy;
    Msg[2] = Gz;
    Msg[3] = Ax;
    Msg[4] = Ay;
    Msg[5] = Az;
}