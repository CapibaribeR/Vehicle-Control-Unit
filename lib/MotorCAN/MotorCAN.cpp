#include "MotorCAN.h"

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
    if( !write(inverter_tx_msg) ) {
        write(inverter_tx_msg);
    }

}

/*==========================================  RECEIVE DATA ==========================================*/
/* Sends Data to Motor Controller 1 */
RxStruct MotorCAN:: receive_from_controller_1(){
    return Datafield_Controller_1;
}

/* Sends Data to Motor Controller 2 */
RxStruct MotorCAN:: receive_from_controller_2(){
    return Datafield_Controller_2;
}

/* Receives Data from Motor Controller */
void MotorCAN::receive_from_controller(){
    RxStruct Datafield;

    //Aux variables
    int Voltage_Hb; //voltage High byte
    int Voltage_int;

    CANMessage Controller_rx_msg;
    Controller_rx_msg.len = 8;
    if( read(Controller_rx_msg) ){
        Datafield.Msg_Counter = Controller_rx_msg.data[0] & 0xF;
        
        Voltage_Hb = Controller_rx_msg.data[0] >> 4;
        Voltage_int = (Voltage_Hb<<8) | Controller_rx_msg.data[1];
        Datafield.Supply_Voltage = Voltage_int/10.0f;
        Datafield.Temp_Controller = Controller_rx_msg.data[2]-100;      //Range[0-255],Temp Range [-100°C to 155°C]
        Datafield.Temp_motor = Controller_rx_msg.data[3]-100;           //Range[0-255],Temp Range [-100°C to 155°C]
        
        Datafield.RPM= (Controller_rx_msg.data[5]<< 8) | Controller_rx_msg.data[4] ;
        
        Datafield.PWM_read=(Controller_rx_msg.data[6]/255.0f)*100;
        Datafield.Current = Controller_rx_msg.data[7];            
            }
    else{
        printf("\n[CAN MOTOR]: ERROR WHILE READING MESSAGE, Reseting!");
        reset_can(); 
    }

    if(Controller_rx_msg.id == CONTROLLER_RX_ID) {
        Datafield_Controller_1 = Datafield;
    }
    if(Controller_rx_msg.id == CONTROLLER_RX_ID_2) {
        Datafield_Controller_2 = Datafield;
    }
}

/*====================================== MOTOR/MOTOR CONTROLLER ERROR CHECK ======================================*/
// Checks if the motor sent 
bool Temperature_Shutdown(RxStruct Controller_1, RxStruct Controller_2){
    bool Temperature_Shutdown{0};

    int16_t Tm_1 = Controller_1.Temp_motor; 
    int16_t Tm_2 = Controller_1.Temp_motor; 
    int16_t Tc_1 = Controller_2.Temp_Controller;
    int16_t Tc_2 = Controller_2.Temp_Controller;
    
    // if Temperature is above limit, shuts the car down
    Temperature_Shutdown = (Tm_1> MAX_TEMP_MOTOR) || (Tm_2> MAX_TEMP_MOTOR);
    Temperature_Shutdown = Temperature_Shutdown || (Tc_1> MAX_TEMP_CONTROLLER) || (Tc_2> MAX_TEMP_CONTROLLER);
    
    if(Temperature_Shutdown){
        printf("[Powertrain]: TEMPERATURE SHUTDOWN ACTIVATED");
    }
    return Temperature_Shutdown;
}



/*==========================================  PRINT DATA ==========================================*/
void MotorCAN::Print_Datafields(){
    Print_Datafield(1, Datafield_Controller_1);
    Print_Datafield(2, Datafield_Controller_2);
    
}

void MotorCAN::Print_Datafield(int Num, RxStruct Inv){
    printf("\r\n\t[CAN] Controller %d: Volt=%.1f V, T_Ctrl= %d°C ,T_Motor = %d°C", 
    Num, Inv.Supply_Voltage ,Inv.Temp_Controller ,Inv.Temp_motor);
    
    printf(" , RPM = %d, PWM = %.2f (%.2f %%), Ic= %d A",
    Inv.RPM , Inv.PWM_read, (Inv.PWM_read/255.0f)*100,Inv.Current );
}

void Print_Datafield(int Num,RxStruct Motor_Data){
    printf("\r\n\t[CAN] Controller %d [msg: %d]: Volt=%.1f V , RPM = %d , Ic= %d A", 
    Num,Motor_Data.Msg_Counter , Motor_Data.Supply_Voltage , Motor_Data.RPM , Motor_Data.Current );
    
    // Dc Pwm
    printf(" PWM = %.2f , (%.2f %%)", Motor_Data.PWM_read, (Motor_Data.PWM_read/65535.0)*100.0);

    // Temperature
    printf(" Tc= %d°C , Tm = %d°C", Motor_Data.Temp_Controller , Motor_Data.Temp_motor);
}
