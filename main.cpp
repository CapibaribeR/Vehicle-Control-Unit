/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Powertrain
 
 * This file contains the Integrated VCU (VEHICLE CONTROL UNIT) that unites the
 * CAN communication with Inverters, the analog communication with all the sensors and 
 * the Electronic Differential Closed Loop control algorithm
 ***/
// main() runs in its own thread in the OS
#include "mbed.h"
#include "rtos.h"
#include <cstdint>
#include <cstdio>


#include <AnalogSensor.h>           // Sensors (APPS, Brake, Steering Wheel)
#include <Differential.h>           // Electronic Differential
#include <MotorCAN.h>               // CAN 2.0 Communication with the motor controller

// DEBUG MODE
#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(...)
#endif


/*===================================== ADC PORTS (STM32 F746ZG) =====================================*/
#define Steering_WHEEL_PIN      PF_10        // Steering Wheel Sensor
#define BSE_PIN                 PF_3        // Brake Pedal Sensor
#define APPS1_PIN               PF_4        // Acc. Pedal Sensor 1
#define APPS2_PIN               PF_4        // Acc. Pedal Sensor 2

// Digital Pins
#define START_BUTTON_PIN        PE_12       // Start Button Input Pin
#define RTDS_PIN                PE_15       // Buzzer Output Pin

/*===================================== COMMUNICATION PORTS (STM32 F746ZG) =====================================*/
// CAN 1: General communication in the VCU
// #define CAN1_RX                 PB_8
// #define CAN1_TX                 PB_9

// CAN 2: Communication with the motor controller
#define CAN2_RX                 PB_8
#define CAN2_TX                 PB_9

// CAN frequency in Hz (bit/s)
// #define CAN1_FREQUENCY           1e6
#define CAN2_FREQUENCY           1e6        // for CAN 2.0, its 1MHz [1Mbit/s] [Mandatory]


//*===================================== Aux Functions =====================================*//
// Control Systems
void OpenLoop();                // Open Loop Control
void RunControl();              // Closed Loop Control

// Safety
void PlausibilityCheck();       //
void SafetyCheck();             // APPS + Brake plausibility and Temperature shutdown
bool ReadyToDrive();            // 1: Ready + Plays Buzzer , 0: Not Ready

// Communication
void Controller_CAN_ISR();
void VCU_CAN_ISR();
void ReadCAN();                 //
void send_to_controller(unsigned int Motor_Id, uint16_t DC_pwm);



/*=================================================== Objetcs =====================================================*/
// Communication
// CAN CAN_VCU(CAN1_RX, CAN1_TX, CAN2_FREQUENCY);                                          // VCU general CAN
MotorCAN CAN_Motor(CAN2_RX, CAN2_TX, CAN2_FREQUENCY);                                   // Motor/Inverter CAN

// Pedal and Steering wheel Sensors
PedalSensor BSE(BSE_PIN, BSE_VMIN, BSE_VMAX);                                           // Brake Pedal sensor
PedalSensor APPS_1(APPS1_PIN ,APPS1_VMIN, APPS1_VMAX);                                  // Accel. Pedal sensor 1
PedalSensor APPS_2(APPS2_PIN, APPS2_VMIN, APPS2_VMAX);                                  // Accel. Pedal sensor 2
SteeringSensor Steering_sensor (Steering_WHEEL_PIN, STEERING_VMIN, STEERING_VMAX);      // Steering Wheel sensor

// Digital Pins
DigitalIn StartButton(START_BUTTON_PIN,PullDown);                                       // Start Button input 
DigitalOut RTDS_Buzzer(RTDS_PIN,0);                                                     // Ready-To-Drive Sound(Buzzer)

// Threads
Thread ControlThread(osPriorityNormal, 6096);


/*===================================== Global  Variables =====================================*/
// Structs
RxStruct Controller1_data, Controller2_data;     // Structs for data received from each controller

// Flags
bool Error_State{0};                            // Checks APPS, BSE
uint8_t BSE_Flag{0};                            // Flag counter for BSE
uint8_t Apps_Flag{0};                           // Flag counter for Apps


int main(){
/*====================================== INITIALIZATION ======================================*/
    // IDLE until Ready to Drive [ Start Button + Brake]
    // while(!ReadyToDrive()){
    //     ThisThread::sleep_for(100ms);
    // };

    // Run RTDS (Buzzer)
    RTDS_Buzzer.write(1);
    ThisThread::sleep_for(1ms);
    RTDS_Buzzer.write(0);

    // Comm. system Initializaion   
    CAN_Motor.set_CAN();
    // Motor.mode(CAN::Normal);
    // Motor.filter(0, 0, CANStandard);
    // CAN_Motor.attach(Controller_CAN_ISR, CAN::RxIrq);

    // Powertrain & motor control
    ControlThread.start(OpenLoop);

/*=========================================== MAIN LOOP ===========================================*/
    while (true) {
        // SafetyCheck();
        // Controller_CAN_ISR();
        // send_to_controller(unsigned int Motor_Id, uint16_t DC_pwm)
        // Controller_CAN_ISR();
        // send_to_controller(CONTROLLER_TX_ID_2, 65535);
        ThisThread::sleep_for(100ms);
        // CAN_Motor.send_to_controller_1(10000);
        // CAN_Motor.send_to_controller_2(10000);
    }
}



/*====================================== Functions ========================================*/
// Waits for the Start Button + Brake
bool ReadyToDrive(){
    if(BSE.read_brake()>=20 && StartButton.read()==1){
        return 1;
    }
    else{
        DEBUG_PRINT("\nBrake: %f",BSE.read_angle());
        DEBUG_PRINT("\nStart Button: %d",StartButton.read());
        return 0;
    }
}

/* Safety Check: Every 20 ms*/
void SafetyCheck(){
    /* Read all sensors*/
    float Apps_1 = APPS_1.read_angle();
    float Apps_2 = APPS_2.read_angle();
    float Brake_val = BSE.read_angle();

    // Check for errors
    // Error_State = BSE_Error_check(Apps_1, Brake_val, &BSE_Flag) || APPS_Error_check(Apps_1, Apps_2, &Apps_Flag) ;
    // Error_State = Error_State || Temperature_Shutdown(Controller1_data,Controller2_data);
}


/* Open Loop Control:Every 1 ms*/
void OpenLoop(){
    uint16_t Dc_Motor[2]{0};    // Control Signal
    uint16_t apps;              // Acc. Pedal
    uint16_t brake;             // Brake Pedal
    float Steering;          // Steering Wheel
    float m1, m2;            

    while(true){
        // Read Data from Controller
        Controller1_data = CAN_Motor.receive_from_inverter_1();
        Controller2_data = CAN_Motor.receive_from_inverter_2();

        // Controller_CAN_ISR();
        // Controller_CAN_ISR();
        // Print_Datafield(2, Controller2_data);

        // Controller_CAN_ISR();
        // Read Sensor Data
        apps = APPS_1.read_pedal();
        brake = BSE.read_brake();
        Steering = Steering_sensor.read_angle();
        // APPS_1.Voltage_print();
        // BSE.Voltage_print();
        // Steering_sensor.Voltage_print();

        // Open Loop without Differential        
        Dc_Motor[0]= apps;
        Dc_Motor[1]= apps;

        // Open Loop with Differential
        OpenLoopDifferential(Steering, apps, Dc_Motor);

        // DEBUG_PRINT("\nBrake: %f", float(brake)*100/UINT16_MAX);
        // DEBUG_PRINT("\nAPPS: %f", float(apps)*100/UINT16_MAX);
        DEBUG_PRINT("\nSteering:  %.3f", Steering);

        // Check for Errors
        // Error_State =0;
        // brake=0;
        // if (Error_State or brake>3){
        //     Dc_Motor[0] = 0;
        //     Dc_Motor[1] = 0;
        // }
            
        DEBUG_PRINT("\nMotor 1: %.2f%%  || Motor 2: %.2f%%",
        float(Dc_Motor[0])*100/UINT16_MAX,float(Dc_Motor[1])*100/UINT16_MAX);
        
        // Send data to Controller
        CAN_Motor.send_to_controller_1( Dc_Motor[0] );     // Send control Signal to Controller 1
        CAN_Motor.send_to_controller_2( Dc_Motor[1 ] );     // Send control Signal to Controller 2
        ThisThread::sleep_for(100ms);
    }

}


void send_to_controller(unsigned int Motor_Id, uint16_t DC_pwm){    
    // uint16_t

    CANMessage inverter_tx_msg;     // Creates Can message
    inverter_tx_msg.id= CONTROLLER_TX_ID_2;   // Id
    inverter_tx_msg.len = 8;        // Datafield size [Bytes], the max size is 8
 
    inverter_tx_msg.data[0] = 65535 & 0xFF;                 // LSB RPM
    inverter_tx_msg.data[1] = 65535 >> 8;                   // MSB RPM
    inverter_tx_msg.data[2] = 15;                     // Constant (15 pairs)
    inverter_tx_msg.data[3] = DC_pwm & 0xFF;                        // LSB PWM (DutyCycle)
    inverter_tx_msg.data[4] = DC_pwm >> 8;                          // MSB PWM (DutyCycle)
    inverter_tx_msg.data[5] = 100 & 0xFF;             // Current's  LSB;
    inverter_tx_msg.data[6] = 100 >> 8;               // Current's  MSB;
    inverter_tx_msg.data[7] = 0b00000000;
    //inverter_tx_msg.data[7] = (IsBreak<<7);                       // b7: 0 = Throtle || 1 = Brake 0
    
    //Sends CAN message, resets can if there is an error

    //if message was not sent as it should, sends it again 
    if(!CAN_Motor.write(inverter_tx_msg) ) {
        printf("\nnot sent");
        CAN_Motor.reset_can();
        CAN_Motor.write(inverter_tx_msg);
    }else{
        printf("\nSENT: %d", DC_pwm);
        printf("\nPWM LSB: %u, PWM MSB: %u", inverter_tx_msg.data[3], inverter_tx_msg.data[4]);
    }

}





// /* ISR for evey time the Motor CAN receives a message*/
void Controller_CAN_ISR(){
    RxStruct Datafield;
    CANMessage ControllerMsg;
    int Voltage_Hb;         //voltage High byte
    int Voltage_int;        // Voltage value

    if( CAN_Motor.read(ControllerMsg) ){
        // Msg Counter
        Datafield.Msg_Counter = ControllerMsg.data[0] & 0xF;
        
        // Voltage 
        Voltage_Hb = ControllerMsg.data[0] >> 4;
        Voltage_int = (Voltage_Hb<<8) | ControllerMsg.data[1];
        Datafield.Supply_Voltage = float(Voltage_int)/10;
        
        // Temperature, Range[0-255],Temp Range [-100°C to 155°C]
        Datafield.Temp_Controller = ControllerMsg.data[2]-100; 
        Datafield.Temp_motor = ControllerMsg.data[3]-100;
        
        // Velocity (RPM)
        Datafield.RPM= (ControllerMsg.data[5]<< 8) | ControllerMsg.data[4] ;
        
        // Duty Cycle and Current  
        Datafield.PWM_read=(float(ControllerMsg.data[6])/255)*100;
        Datafield.Current = ControllerMsg.data[7]; 

        switch (ControllerMsg.id) {
            case CONTROLLER_RX_ID:
                Controller1_data = Datafield;
                Print_Datafield(1,Controller1_data);
                break;

            case CONTROLLER_RX_ID_2:
                Controller2_data = Datafield;
                Print_Datafield(2,Controller2_data);
                break;
            default:
                Controller2_data = Datafield;
                Print_Datafield(ControllerMsg.id, Controller2_data);
        }
    }
    else{
        DEBUG_PRINT("\n[CAN MOTOR]: ERROR WHILE READING MESSAGE");
    }

}







// /* Sends Data to Motor Controller */
// void send_to_controller(unsigned int Motor_Id, uint16_t DC_pwm){    
    
//     CANMessage inverter_tx_msg;     // Creates Can message
//     inverter_tx_msg.id= Motor_Id;   // Id
//     inverter_tx_msg.len = 8;        // Datafield size [Bytes], the max size is 8
 
//     inverter_tx_msg.data[0] = MAX_RPM_LIMIT & 0xFF;                 // LSB RPM
//     inverter_tx_msg.data[1] = MAX_RPM_LIMIT >> 8;                   // MSB RPM
//     inverter_tx_msg.data[2] = MOTOR_POLE_PAIRS;                     // Constant (15 pairs)
//     inverter_tx_msg.data[3] = DC_pwm & 0xFF;                        // LSB PWM (DutyCycle)
//     inverter_tx_msg.data[4] = DC_pwm >> 8;                          // MSB PWM (DutyCycle)
//     inverter_tx_msg.data[5] = MAX_CURRENT_LIMIT & 0xFF;             // Current's  LSB;
//     inverter_tx_msg.data[6] = MAX_CURRENT_LIMIT >> 8;               // Current's  MSB;
//     inverter_tx_msg.data[7] = 0b00000000;
//     //inverter_tx_msg.data[7] = (IsBreak<<7);                       // b7: 0 = Throtle || 1 = Brake 0
    
//     //Sends CAN message, resets can if there is an error

//     //if message was not sent as it should, sends it again 
//     if( !Motor.write(inverter_tx_msg) ) {
//         Motor.write(inverter_tx_msg);
//         Motor.reset();
//         printf("\nnot sent");
        
//     }else{
//         printf("\nSENT: %d", DC_pwm);
//     }

// }





