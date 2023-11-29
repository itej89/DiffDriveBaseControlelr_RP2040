/**
 * RP2040 FreeRTOS Template
 * 
 * @copyright 2022, Tony Smith (@smittytone)
 * @version   1.4.1
 * @licence   MIT
 *
 */

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>


#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <atomic>
#include <algorithm>

#include "hardware/sync.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"

#include "pico_pwm.h"
#include "DiffDriveState.h"



#define DEBUG_SERIAL 1


/**
 * @brief CONSTANTS ----------------------------------------------------------
 * ---------------------------------------------------------------------------
 */
// RTOS TASK IDs for reading commands and speed control loops
#define TIMER_ID_COMPUTE_STATE  2
#define TIMER_ID_READ_CMD     3

// GPIO Connection PIN numbers
#define LEFT_PWM_PIN  1
#define RIGHT_PWM_PIN  0

#define LEFT_DIR_PIN  2
#define RIGHT_DIR_PIN  3

#define RIGHT_ENCODER_PIN 7
#define LEFT_ENCODER_PIN 6

//Execution frequency of reading commands and speed control loops
#define COMPUTE_STATE_RATE 20 //hz
#define READ_CMD_RATE    20   //hz

#define COMPUTE_STATE_PERIOD_MS 1000/COMPUTE_STATE_RATE //Milliseconds
#define READ_CMD_PERIOD_MS    1000/READ_CMD_RATE        //Milliseconds

//Float to Integer scale factor for Serial State Commands
#define WRITE_STATE_SCALE_FACTOR 10000
//----------------------------------------------------------



/**
 * @brief SPEED CONTROL CONTEXT ----------------------------------------------------------
 * ---------------------------------------------------------------------------------------
 */
uint8_t LEFT_WHEEL_DIR = 0;
uint8_t RIGHT_WHEEL_DIR = 0;
uint8_t LEFT_PWM = 0;
uint8_t RIGHT_PWM = 0;

uint32_t LEFT_WHEEL_VEL = 0;
uint32_t RIGHT_WHEEL_VEL = 0;

uint32_t LEFT_WHEEL_RPM = 0;
uint32_t RIGHT_WHEEL_RPM = 0;
//----------------------------------------------------------


//time before we decide our control program has failed and we kill the motors
#define timeoutMs 500

//set to 1 to enable a subscriber for P, I and D values.
//note that D values are scaled by dividing by 1000, since rosserial can only do 32 bit floats
#define pidTuningMode 0



double rITerm = 0.0;
double lITerm = 0.0;
int rLastSpeed = 0;
int lLastSpeed = 0;

double _kP = .045;
double _kI = 0.000;
double _kD = 0.000000000625;

// double _kP = .000045;
// double _kI = 0.0;
// double _kD = 0.0;

long nextUpdateR = 0;
long nextUpdateL = 0;
long nextUpdatePID = 0;
long nextUpdatePower = 0;



//Initialize all required tasks--------------------------------------
//-------------------------------------------------------------------
DiffDriveState* DiffDriveState::instancePtr = NULL;
TaskHandle_t hw_interface_timer_schedule_handle = NULL;
volatile TimerHandle_t compute_state_timer;
volatile TimerHandle_t read_command_timer;
//-------------------------------------------------------------------


//Initialize Frame intformation--------------------------------------
//-------------------------------------------------------------------
#define BYTES_SF  0x1FA8 //Start of frame
#define BYTES_EF  0x1FA9 //End of frame

//Stores all incoming data
std::vector<uint8_t> RX_BUFFER;

//Stores Extracted frames from the RX_BUFFER
std::vector<std::vector<uint8_t>> RX_FRAMES;
//-------------------------------------------------------------------




//Initialize GPIO pins-----------------------------------------------
//-------------------------------------------------------------------
void init_gpio() {
    gpio_init(LEFT_DIR_PIN);
    gpio_set_dir(LEFT_DIR_PIN, GPIO_OUT);
    gpio_put(LEFT_DIR_PIN, 1);

    gpio_init(RIGHT_DIR_PIN);
    gpio_set_dir(RIGHT_DIR_PIN, GPIO_OUT);
    gpio_put(RIGHT_DIR_PIN, 1);


    analogInit(LEFT_PWM_PIN, 200, 0);
    analogInit(RIGHT_PWM_PIN, 200, 0);
    analogWrite(LEFT_PWM_PIN, 200, 0);
    analogWrite(RIGHT_PWM_PIN, 200, 0);
}
//-------------------------------------------------------------------


//Interrupt handlers for encoders------------------------------------
//-------------------------------------------------------------------
void IRQ_ENCODER(uint gpio, uint32_t events) {
    switch(gpio){

        case LEFT_ENCODER_PIN:
        DiffDriveState::getInstance()->UpdateLeftEncoderCount();
        break;

        case RIGHT_ENCODER_PIN:
        DiffDriveState::getInstance()->UpdateRightEncoderCount();
        break;

    };
}
//-------------------------------------------------------------------




//Write back the wheek state such as position and velocity
void write_state() {
    // Radians units scaled by WRITE_STATE_SCALE_FACTOR
    uint32_t LeftPosScaled  = DiffDriveState::getInstance()->GetLeftWheelPosition()  * WRITE_STATE_SCALE_FACTOR;
    uint32_t RightPosScaled = DiffDriveState::getInstance()->GetRightWheelPosition() * WRITE_STATE_SCALE_FACTOR;
    
    // m/sec units scaled by WRITE_STATE_SCALE_FACTOR
    uint32_t LeftVelScaled  = DiffDriveState::getInstance()->GetLeftWheelVelocity()  * WRITE_STATE_SCALE_FACTOR;
    uint32_t RightVelScaled = DiffDriveState::getInstance()->GetRightWheelVelocity() * WRITE_STATE_SCALE_FACTOR;

    //FRAME FORMAT [SOF, LEN, DATA, CHK, EOF]
    uint8_t stateFrame[] = {
        0x1F, 0xE8,
        0x10,
        (uint8_t)(LeftPosScaled  >> 24  & 0xFF), (uint8_t)(LeftPosScaled  >> 16 & 0xFF), (uint8_t)(LeftPosScaled  >> 8 & 0xFF), (uint8_t)(LeftPosScaled  & 0xFF), 
        (uint8_t)(RightPosScaled >> 24  & 0xFF), (uint8_t)(RightPosScaled >> 16 & 0xFF), (uint8_t)(RightPosScaled >> 8 & 0xFF), (uint8_t)(RightPosScaled & 0xFF), 
        LEFT_WHEEL_DIR,
        (uint8_t)(LeftVelScaled  >> 24  & 0xFF), (uint8_t)(LeftVelScaled  >> 16 & 0xFF), (uint8_t)(LeftVelScaled  >> 8 & 0xFF), (uint8_t)(LeftVelScaled  & 0xFF), 
        RIGHT_WHEEL_DIR,
        (uint8_t)(RightVelScaled >> 24  & 0xFF), (uint8_t)(RightVelScaled >> 16 & 0xFF), (uint8_t)(RightVelScaled >> 8 & 0xFF), (uint8_t)(RightVelScaled & 0xFF), 
        0x02, 0x02,
        0x1F, 0xE9
    };

    for (int i=0; i< 23; i++)
        putchar(stateFrame[i]);
}


std::atomic<bool>  IsMotorCmdReceived(false);
void handle_command() {
    //   printf("\nReceived Requst!!" );
    
    if(RX_FRAMES.size() > 0){
        uint8_t COMMAND_TYPE = RX_FRAMES.at(0).at(0);
        switch(COMMAND_TYPE){
            case 0x10:
            {
                uint32_t tLEFT_WHEEL_VEL = RX_FRAMES.at(0).at(2);
                tLEFT_WHEEL_VEL = tLEFT_WHEEL_VEL << 24 | RX_FRAMES.at(0).at(3);
                tLEFT_WHEEL_VEL = tLEFT_WHEEL_VEL << 16 | RX_FRAMES.at(0).at(4);
                tLEFT_WHEEL_VEL = tLEFT_WHEEL_VEL << 8  | RX_FRAMES.at(0).at(5);

                uint32_t tRIGHT_WHEEL_VEL = RX_FRAMES.at(0).at(7);
                tRIGHT_WHEEL_VEL = tRIGHT_WHEEL_VEL << 24 | RX_FRAMES.at(0).at(8);
                tRIGHT_WHEEL_VEL = tRIGHT_WHEEL_VEL << 16 | RX_FRAMES.at(0).at(9);
                tRIGHT_WHEEL_VEL = tRIGHT_WHEEL_VEL << 8 | RX_FRAMES.at(0).at(10);

                LEFT_WHEEL_DIR  =  RX_FRAMES.at(0).at(1);
                RIGHT_WHEEL_DIR =  RX_FRAMES.at(0).at(6);

                //Recieved Wheel velocities in  mm per second
                LEFT_WHEEL_VEL  = tLEFT_WHEEL_VEL;
                RIGHT_WHEEL_VEL = tRIGHT_WHEEL_VEL;

                //Converted Wheel RPM
                double lomega = ((double)LEFT_WHEEL_VEL*1.0) / ((double)WHEEL_RADIUS*1.0);
                double lrps = lomega / (2 * 3.14);
                double lrpm = lrps * 60;
                LEFT_WHEEL_RPM  = (uint32_t)lrpm;

                double romega = ((double)RIGHT_WHEEL_VEL*1.0) / ((double)WHEEL_RADIUS*1.0);
                double rrps = romega / (2 * 3.14);
                double rrpm = rrps * 60;
                RIGHT_WHEEL_RPM  = (uint32_t)rrpm;

                IsMotorCmdReceived = true;
            }
            break;

            case 0x20:{
                write_state();
            }
            break;
        }
    }
}

int limitPWM(int pwm) {
//   if (pwm < 20)
//      pwm = 20;

  if (pwm > 100)
     pwm = 100;

  return pwm;   
}

int one_sec_counter = 0;
void compute_state_callback(TimerHandle_t timer) {
        DiffDriveState::getInstance()->updateWheelPosition();
        DiffDriveState::getInstance()->updateWheelState();


        unsigned int l_rpm = DiffDriveState::getInstance()->GetLeftWheelRPM();
        unsigned int r_rpm = DiffDriveState::getInstance()->GetRightWheelRPM();


        // Maps (a value of 0 to 1) and (a value of 1 to -1)
        DiffDriveState::getInstance()->SetLeftWheelDirection( (LEFT_WHEEL_DIR * -2)+1 );
        gpio_put(LEFT_DIR_PIN, LEFT_WHEEL_DIR);

        int error = LEFT_WHEEL_RPM - l_rpm;  // calculate error
        lITerm += (_kI * (double)error); // calculate integral term
        double dInput = l_rpm - lLastSpeed; // calculate derivative
        int adjustment = (_kP * (double)error) + lITerm - (_kD * dInput);
        LEFT_PWM += adjustment;
        //limit speed to range of pwm 0-255
        LEFT_PWM = limitPWM(LEFT_PWM);

        if (LEFT_WHEEL_VEL == 0)
        {
            LEFT_PWM = 0;
            DiffDriveState::getInstance()->SetLeftWheelStopped();
        }

        lLastSpeed = l_rpm;


        // Maps (a value of 0 to 1) and (a value of 1 to -1)
        DiffDriveState::getInstance()->SetRightWheelDirection( (RIGHT_WHEEL_DIR * -2)+1 );
        gpio_put(RIGHT_DIR_PIN, RIGHT_WHEEL_DIR);
        error = RIGHT_WHEEL_RPM - r_rpm;  // calculate error
        rITerm += (_kI * (double)error); // calculate integral term
        dInput = r_rpm - rLastSpeed; // calculate derivative
        adjustment = (_kP * (double)error) + rITerm - (_kD * dInput);
        RIGHT_PWM += adjustment;
        //limit speed to range of pwm 0-255
        RIGHT_PWM = limitPWM(RIGHT_PWM);

        if (RIGHT_WHEEL_VEL == 0)
        {
            RIGHT_PWM = 0;
            DiffDriveState::getInstance()->SetRightWheelStopped();
        }
     
        rLastSpeed = r_rpm;



        if(one_sec_counter == 20) {
            one_sec_counter = 0;
            IsMotorCmdReceived = false;
        }


        if (IsMotorCmdReceived) {
            if(LEFT_PWM != 100)
                analogWrite(LEFT_PWM_PIN, 200, LEFT_PWM);
            if(RIGHT_PWM != 100)
                analogWrite(RIGHT_PWM_PIN, 200, RIGHT_PWM);
        }
        else {
            analogWrite(LEFT_PWM_PIN, 200, 0);
            analogWrite(RIGHT_PWM_PIN, 200, 0);
        }


        one_sec_counter++;


    // printf("\n%d %d %d %d %f %f",LEFT_PWM,  RIGHT_PWM, LEFT_WHEEL_VEL, LEFT_WHEEL_RPM, DiffDriveState::getInstance()->GetLeftWheelVelocity(), DiffDriveState::getInstance()->GetLeftWheelRPM() );
    
}



void read_command_callback(TimerHandle_t timer) {
    //Read a byte from the input
    int ch;
    ch = getchar_timeout_us(5000);
    uint8_t byte = ch;
    while(ch != PICO_ERROR_TIMEOUT){
        RX_BUFFER.push_back(byte&0xFF);
        ch = getchar_timeout_us(5000);
        byte = ch;
    }

    //Parse all complete frames recieved
    //FRAME FORMAT [SOF, LEN, DATA, CHK, EOF]
    while (RX_BUFFER.size() > 3) {

        //Validate start of frame bytes
        if(RX_BUFFER.at(0) == (BYTES_SF>>8 & 0xFF) && 
        RX_BUFFER.at(1) == (BYTES_SF    & 0xFF)){
                
                //Get frame legth
                uint8_t length = RX_BUFFER.at(2);

                //Wait and read complete frame length
                // SOF+CMD_TYPE+DATA+CHKSUM+EOF
                while(RX_BUFFER.size() < 2+1+length+2+2){
                    ch = getchar_timeout_us(5000);
                    if(ch == PICO_ERROR_TIMEOUT)
                        break;
                    RX_BUFFER.push_back(ch&0xFF);
                }

                //Addd frame to the que
                RX_FRAMES.push_back(std::vector<uint8_t>(RX_BUFFER.begin()+3, RX_BUFFER.begin()+3+length));

                //handle command
                handle_command();
                
                //erase frame from the que
                RX_BUFFER.erase(RX_BUFFER.begin(),RX_BUFFER.begin() + 2+1+RX_FRAMES.at(0).size()+2+2 );
                RX_FRAMES.erase(RX_FRAMES.begin(),RX_FRAMES.begin()+1);
        }
        else
            //keep clearing data until valid start of frame is found
            RX_BUFFER.erase(RX_BUFFER.begin(), RX_BUFFER.begin()+1);
    }
}



void hw_interface_timer_scheduler(void* arg) {

    // Create Two timers for reading commands ans writing state
    compute_state_timer = xTimerCreate("COMPUTE_STATE_TIMER",
                                pdMS_TO_TICKS(COMPUTE_STATE_PERIOD_MS),
                                pdTRUE,
                                (void*)TIMER_ID_COMPUTE_STATE,
                                compute_state_callback);

    read_command_timer = xTimerCreate("READ_CMD_TIMER",
                                pdMS_TO_TICKS(READ_CMD_PERIOD_MS),
                                pdTRUE,
                                (void*)TIMER_ID_READ_CMD,
                                read_command_callback);
    
    if (compute_state_timer  != NULL)  xTimerStart(compute_state_timer,  0);
    if (read_command_timer != NULL)  xTimerStart(read_command_timer, 0);

    while(true){
        /*pass*/
    }
}



/*
 * RUNTIME START
 */
int main() {

    // Initialize chosen serial port
    stdio_init_all();
    init_gpio();

    printf ("Hello GPIO IRQ\n");
    gpio_set_irq_enabled_with_callback (RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_FALL, true, &IRQ_ENCODER);
    gpio_set_irq_enabled_with_callback (LEFT_ENCODER_PIN,  GPIO_IRQ_EDGE_FALL, true, &IRQ_ENCODER);

    //Main task that handles hw interface's read and write timers
    BaseType_t hw_interface_status = xTaskCreate(hw_interface_timer_scheduler, 
                                         "TIMER_INITIALIZER", 
                                         128, 
                                         NULL, 
                                         1, 
                                         &hw_interface_timer_schedule_handle); 
    
    // Start the FreeRTOS scheduler
    // FROM 1.0.1: Only proceed with valid tasks
    if (hw_interface_status == pdPASS) {
        vTaskStartScheduler();
    }
    
    while (true) {    }
    
}