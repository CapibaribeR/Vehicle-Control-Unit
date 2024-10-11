#include "MotorCAN.h"
#include <cstdio>

/*================================== Constructors ==================================*/
MotorCAN::MotorCAN(PinName _can_pin_rx, PinName _can_pin_tx, uint32_t _can_frequency)
:CAN(_can_pin_rx,
     _can_pin_tx,
    _can_frequency) {}

/*================================== METHODS ==================================*/
/* */
void MotorCAN:: set_CAN(){
    mode(CAN::Normal);
    filter(0, 0, CANStandard);
}

/* Resets CAN chanel */
void MotorCAN:: reset_can() {  
    reset();
    mode(CAN::Normal);
    filter(0, 0, CANStandard); // reconfigurar o filtro para receber todas as mensagens
}

/* Tests if CAN can send messages (Creates empy CAN message and tests if it was sent) */
bool MotorCAN:: baud_test(){
    CANMessage msg;
    return write(msg);    
}

/* Tests if CAN can read messages */
bool MotorCAN::is_can_active(){
    CANMessage msg;
    return read(msg); // tente ler uma mensagem
}

/*==========================================  SEND DATA ==========================================*/
/* Sends Data to Motor Controller 1 */
void MotorCAN:: send_to_controller_1(uint16_t DC_pwm_1){
    send_to_controller(CONTROLLER_TX_ID, DC_pwm_1);
}

/* Sends Data to Motor Controller 2 */
void MotorCAN:: send_to_controller_2(uint16_t DC_pwm_2){
    send_to_controller(CONTROLLER_TX_ID_2, DC_pwm_2);
}

/* Sends Data to Motor Controller */
void MotorCAN:: send_to_controller(unsigned int Motor_Id, uint16_t DC_pwm){    
    
    CANMessage inverter_tx_msg;     // Creates Can message
    inverter_tx_msg.id= Motor_Id;   // Id
    inverter_tx_msg.len = 8;        // Datafield size [Bytes], the max size is 8
 
    inverter_tx_msg.data[0] = MAX_RPM_LIMIT & 0xFF;                 // LSB RPM
    inverter_tx_msg.data[1] = MAX_RPM_LIMIT >> 8;                   // MSB RPM
    inverter_tx_msg.data[2] = MOTOR_POLE_PAIRS;                     // Constant (15 pairs)
    inverter_tx_msg.data[3] = DC_pwm & 0xFF;                        // LSB PWM (DutyCycle)
    inverter_tx_msg.data[4] = DC_pwm >> 8;                          // MSB PWM (DutyCycle)
    inverter_tx_msg.data[5] = MAX_CURRENT_LIMIT & 0xFF;             // Current's  LSB;
    inverter_tx_msg.data[6] = MAX_CURRENT_LIMIT >> 8;               // Current's  MSB;
    inverter_tx_msg.data[7] = 0b00000000;
    //inverter_tx_msg.data[7] = (IsBreak<<7);                       // b7: 0 = Throtle || 1 = Brake 0
    
    //Sends CAN message, resets can if there is an error

    //if message was not sent as it should, sends it again 
    if(!write(inverter_tx_msg) ) {
        DEBUG_PRINT_CAN("\n[MOTOR CAN]: Not sent");
        reset_can();
        // write(inverter_tx_msg);
    }else{
        DEBUG_PRINT_CAN("\n[MOTOR CAN]: SENT %d", DC_pwm);

    }

}


/*==========================================  RECEIVE DATA ==========================================*/
/* Receives Data from Motor Controller 1 */
RxStruct MotorCAN:: receive_from_inverter_1(){
    RxStruct Datafield_1 = receive_from_inverter(CONTROLLER_RX_ID);
    return Datafield_1;
}

/* Receives Data from Motor Controller 2 */
RxStruct MotorCAN:: receive_from_inverter_2(){
    RxStruct Datafield_2 = receive_from_inverter(CONTROLLER_RX_ID_2);
    return Datafield_2;
}

/* Receives Data from Motor Controller */
RxStruct MotorCAN:: receive_from_inverter(unsigned int Inverter_Id){
    RxStruct Datafield;

    //Aux variables
    int Voltage_Hb; //voltage High byte
    int Voltage_int;

    CANMessage inverter_rx_msg;
    inverter_rx_msg.len = 8;
    if(is_can_active() ) {
        if(read(inverter_rx_msg)){
        // Aguardar a recepção da mensagem do inversor
            if(inverter_rx_msg.id == Inverter_Id) {
                // Msg Counter
                Datafield.Msg_Counter = inverter_rx_msg.data[0] & 0xF;
                
                // Voltage 
                Voltage_Hb = inverter_rx_msg.data[0] >> 4;
                Voltage_int = (Voltage_Hb<<8) | inverter_rx_msg.data[1];
                Datafield.Supply_Voltage =float(Voltage_int)/10;

                // Temperature
                Datafield.Temp_Controller = inverter_rx_msg.data[2]-100; //Range[0-255],Temp Range [-100°C to 155°C]
                Datafield.Temp_motor = inverter_rx_msg.data[3]-100;     //Range[0-255],Temp Range [-100°C to 155°C]

                // Velocity (RPM)
                Datafield.RPM= (inverter_rx_msg.data[5]<< 8) | inverter_rx_msg.data[4] ;

                // Duty Cycle and Current  
                Datafield.PWM_read=(inverter_rx_msg.data[6]/255.0f)*100;
                Datafield.Current = inverter_rx_msg.data[7];            
                Print_Datafield(Inverter_Id, Datafield);
            
            }
        }
    }
    else{
        printf("Fail to stablish CAN connection. Reseting...\n");
        reset_can(); 
    }
    
    // Datafield_inv1 =Datafield;
    return Datafield;
}


// RxStruct receive_from_controller(unsigned int Inverter_Id){
//     RxStruct Datafield;
//     CANMessage Controller_Rx_msg;
//     int Voltage_Hb;         //voltage High byte
//     int Voltage_int;        // Voltage value

//     if(read(Controller_Rx_msg)){
//         // Msg Counter
//         Datafield.Msg_Counter = Controller_Rx_msg.data[0] & 0xF;
        
//         // Voltage 
//         Voltage_Hb = Controller_Rx_msg.data[0] >> 4;
//         Voltage_int = (Voltage_Hb<<8) | Controller_Rx_msg.data[1];
//         Datafield.Supply_Voltage = float(Voltage_int)/10;
        
//         // Temperature
//         Datafield.Temp_Controller = Controller_Rx_msg.data[2]-100; //Range[0-255],Temp Range [-100°C to 155°C]
//         Datafield.Temp_motor = Controller_Rx_msg.data[3]-100;     //Range[0-255],Temp Range [-100°C to 155°C]
        
//         // Velocity (RPM)
//         Datafield.RPM= (Controller_Rx_msg.data[5]<< 8) | Controller_Rx_msg.data[4] ;
        
//         // Duty Cycle and Current  
//         Datafield.PWM_read=(Controller_Rx_msg.data[6]/255.0f)*100;
//         Datafield.Current = Controller_Rx_msg.data[7]; 
//     }
//     else{
//         printf("\n[CAN MOTOR]: ERROR WHILE READING MESSAGE");
//     }

//     return Datafield;
// }
    



/*====================================== MOTOR/MOTOR CONTROLLER ERROR CHECK ======================================*/
// Checks if the motor sent 
bool Temperature_Check(RxStruct Controller_1, RxStruct Controller_2){
    bool Temperature_Shutdown{0};

    int16_t Tm_1 = Controller_1.Temp_motor; 
    int16_t Tm_2 = Controller_1.Temp_motor; 
    int16_t Tc_1 = Controller_2.Temp_Controller;
    int16_t Tc_2 = Controller_2.Temp_Controller;
    
    // if Temperature is above limit, shuts the car down
    Temperature_Shutdown = (Tm_1 > MAX_TEMP_MOTOR) || (Tm_2 > MAX_TEMP_MOTOR);
    Temperature_Shutdown = Temperature_Shutdown || (Tc_1 > MAX_TEMP_CONTROLLER) || (Tc_2 > MAX_TEMP_CONTROLLER);
    
    if(Temperature_Shutdown){
        DEBUG_PRINT_CAN("\n[Powertrain]: TEMPERATURE SHUTDOWN ACTIVATED");
    }
    return Temperature_Shutdown;
}










/*==========================================  PRINT DATA ==========================================*/
void MotorCAN::Print_Datafields(){
    // Print_Datafield(1, Datafield_inv1);
    // Print_Datafield(2, Datafield_inv2);
    
}

void MotorCAN::Print_Datafield(int Num, RxStruct Inv){
    DEBUG_PRINT_CAN("\r\n\t[CAN] Controller [%d]: MSG[%d], Volt=%.1f V, T_Ctrl= %d°C ,T_Motor = %d°C", 
    Num,Inv.Msg_Counter, Inv.Supply_Voltage ,Inv.Temp_Controller ,Inv.Temp_motor);
    
    DEBUG_PRINT_CAN(" , RPM = %d, PWM = %.2f (%.2f %%), Ic= %d A",
    Inv.RPM , Inv.PWM_read, (Inv.PWM_read/255.0f)*100,Inv.Current );
}

void Print_Datafield(int Num,RxStruct Motor_Data){
    DEBUG_PRINT_CAN("\r\n\t[CAN] Controller [%d]: MSG[%d]: Volt=%.1f V , RPM = %d , Ic= %d A", 
    Num, Motor_Data.Msg_Counter , Motor_Data.Supply_Voltage , Motor_Data.RPM , Motor_Data.Current );
    
    // Dc Pwm
    DEBUG_PRINT_CAN(" PWM = %.2f , (%.2f %%)", Motor_Data.PWM_read, (float(Motor_Data.PWM_read)/65535)*100);

    // Temperature
    DEBUG_PRINT_CAN(" Tc= %d°C , Tm = %d°C", Motor_Data.Temp_Controller , Motor_Data.Temp_motor);
}
