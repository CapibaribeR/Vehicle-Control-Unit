/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain
 
 * This file contains the Integrated VCU (VEHICLE CONTROL UNIT) that unites the
 * CAN communication with Inverters, the analog communication with all the sensors and 
 * the Electronic Differential Closed Loop control algorithm
 ***/

#include "mbed.h"
#include "rtos.h"

#include <AnalogSensor.h>        // Read APPS, BSE and Steering Wheel sensors
#include <Differential.h>       // Electronic Differential
#include <MotorCAN.h>           // CAN 2.0 Communication with the Motor
//
// #include <MotorCAN.h>           // Read MPU6050 (Gyro + Acelerometer + Magnetometer)

// main() runs in its own thread in the OS

// DEBUG MODE
#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(...)
#endif


// Digital Pins
#define START_BUTTON_PIN        PE_15       // Start Button Input Pin
#define RTDS_PIN                PF_12       // Buzzer Output Pin

/* ============ Objetcs ============ */
// Communication
CAN CAN_VCU(CAN1_RX, CAN1_TX, CAN2_FREQUENCY);                  // VCU general CAN
MotorCAN CAN_Motor(CAN2_RX, CAN2_TX, CAN2_FREQUENCY);           // Motor/Inverter CAN

// Pedal and Steering wheel Sensors
PedalSensor BSE(BSE_PIN);                                       // Brake Pedal sensor
PedalSensor APPS_1(APPS1_PIN);                                  // Accel. Pedal sensor 1
PedalSensor APPS_2(APPS2_PIN);                                  // Accel. Pedal sensor 2
SteeringSensor Steering_sensor (Steering_WHEEL_PIN);            // Steering Wheel sensor

// Digital Pins
DigitalIn StartButton(START_BUTTON_PIN,PullDown);               // Start Button input 
DigitalOut RTDS_Buzzer(RTDS_PIN,0);                             // Ready-To-Drive Sound (Buzzer)

// Threads
Thread ControlThread(osPriorityNormal, 4096);                   //

/* ============ Aux Functions ============ */
// Control Systems
void OpenLoop();                // Open Loop Control

// Safety
void SafetyCheck();             // APPS + Brake plausibility and Temperature shutdown
bool ReadyToDrive();            // 1: Ready + Plays Buzzer , 0: Not Ready


/* ============ Global  Variables ============ */
// Structs
RxStruct Controller1_data, Controller2_data;     // Structs for data received from each controller

// Flags
bool Error_State{0};                            // Checks APPS, BSE
bool Overheating_Flag{0};                       // flag for Overheating
uint8_t BSE_Flag{0};                            // Flag counter for BSE error
uint8_t Apps_Flag{0};                           // Flag counter for APPS error


/*====================================== INITIALIZATION ======================================*/
int main(){
    // IDLE until Ready to Drive [ Start Button + Brake]
    while(!ReadyToDrive()){
        ThisThread::sleep_for(100ms);
    };

    // Run RTDS (Buzzer)
    RTDS_Buzzer.write(1);
    ThisThread::sleep_for(1ms);
    RTDS_Buzzer.write(0);

    // Comm. system Initializaion   
    CAN_Motor.set_CAN();

    // Powertrain & motor control
    ControlThread.start(OpenLoop);

/*=========================================== MAIN LOOP ===========================================*/
    while (true) {
        // SafetyCheck();
        ThisThread::sleep_for(20ms);
    }
}


/*====================================== Functions ========================================*/
// Waits for the Start Button + Brake
bool ReadyToDrive(){
    if(BSE.read_angle()>=20 && StartButton.read()==1){
        return 1;
    }
    else{
        DEBUG_PRINT("\nBrake: %f",BSE.read_angle());
        DEBUG_PRINT("\nStart Button: %d",StartButton.read());
        return 0;
    }
}

void Send_to_TCU(){
    float Gyro_Arr[3]{0};
    float Accel_Arr[3]{0};
    
    // Receive Data from Inverters
    Controller1_data = CAN_Motor.receive_from_controller_1();
    Controller2_data = CAN_Motor.receive_from_controller_2();
    
    // Check Temperature (1: Error, 0: Safe)
    Overheating_Flag = Temperature_Shutdown(Controller1_data, Controller2_data);

    // Print Message
    Print_Datafield(1, Controller1_data);
    Print_Datafield(2, Controller2_data);

    // Read MPU
    // MPU_data = mpu.GetMsg();

    // Send data to TCU
    //send(Controller1_data, Controller2_data, MPU_data)
}


/* Safety Check: Every 20 ms*/
void SafetyCheck(){
    /* Read all sensors*/
    float Apps_1 = APPS_1.read_angle();
    float Apps_2 = APPS_2.read_angle();
    float Brake_val = BSE.read_angle();

    // Check for errors
    bool APPS_Error = APPS_Error_check(Apps_1, Brake_val, &BSE_Flag);
    bool BSE_Error = BSE_Error_check(Apps_1, Apps_2, &Apps_Flag);
    bool Circuit_Error = APPS_1.Error_Flag | APPS_2.Error_Flag | BSE.Error_Flag;
    
    // Sets flag if there's an error
    Error_State = APPS_Error || BSE_Error || Overheating_Flag || Circuit_Error;
}


/* Open Loop Control:Every 100 ms*/
void OpenLoop(){
    uint16_t Dc_Motor[2]{0};    // Control Signal
    uint16_t apps;              // Acc. Pedal
    float brake;             // Brake Pedal
    float Steering_dg;          // Steering Wheel

    while(true){
        // Read Sensor's Data
        apps = APPS_1.read_pedal();
        brake = BSE.read_angle();
        Steering_dg = Steering_sensor.read_angle();
        
        // Open Loop without Differential        
        Dc_Motor[0]= apps;
        Dc_Motor[1]= apps;

        // Open Loop with Differential
        OpenLoopDifferential(Steering_dg, apps, Dc_Motor);

        // Pedal Travel Percentage [0 - 100]
        DEBUG_PRINT("\nAPPS:  %.2f, [%d]", (float(apps)/UINT16_MAX)*100, apps);
        DEBUG_PRINT("\nBrake: %.2f", brake);

        // Check for Errors
        Error_State =0;
        brake=0;
        if (Error_State or brake>3){
            Dc_Motor[0] = 0;
            Dc_Motor[1] = 0;
        }
        
        // Print Control Signal Percentage [0 - 100]
        DEBUG_PRINT("\nMotor 1: %.2f%%  || Motor 2: %.2f%%",
        (float(Dc_Motor[0])/UINT16_MAX)*100, (float(Dc_Motor[1])/UINT16_MAX)*100);
        
        // Send data to Inverters 
        CAN_Motor.send_to_controller_1( Dc_Motor[0] );     // Send control Signal to Controller 1
        CAN_Motor.send_to_controller_2( Dc_Motor[1] );     // Send control Signal to Controller 2

        ThisThread::sleep_for(100ms);
    }
}