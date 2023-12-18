/**
 * RP2040 FreeRTOS Template
 * 
 * @copyright 2022, Tony Smith (@smittytone)
 * @version   1.4.1
 * @licence   MIT
 *
 */

//FreeRTOS Headers-----------------
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>

//C++ Headers----------------------
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

//PICO SDK Headers---------------
#include "hardware/sync.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"

//Application Headers------------
#include "Constants.h"
#include "PIDParams.h"
#include "pico_pwm.h"
#include "DiffDriveState.h"




//Initialize all required tasks--------------------------------------
//-------------------------------------------------------------------
DiffDriveState diffDriveState;
#include "CommandParser.h"
TaskHandle_t hw_interface_timer_schedule_handle = NULL;
volatile TimerHandle_t compute_state_timer;
volatile TimerHandle_t read_command_timer;
//-------------------------------------------------------------------


//Initialize Frame intformation--------------------------------------
//-------------------------------------------------------------------
#define BYTES_SF  0x1FA8 // Start of frame
#define BYTES_EF  0x1FA9 // End of frame

// Stores all incoming data
std::vector<uint8_t> RX_BUFFER;

// Stores Extracted frames from the RX_BUFFER
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




std::atomic<bool>  IsMotorCmdReceived(false);
//Parses the command received over serial
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
                handle_command(RX_FRAMES.at(0));
                
                //erase frame from the que
                RX_BUFFER.erase(RX_BUFFER.begin(),RX_BUFFER.begin() + 2+1+RX_FRAMES.at(0).size()+2+2 );
                RX_FRAMES.erase(RX_FRAMES.begin());
                IsMotorCmdReceived = true;
        }
        else
            //keep clearing data until valid start of frame is found
            RX_BUFFER.erase(RX_BUFFER.begin(), RX_BUFFER.begin()+1);
    }
}



int limitPWM(int pwm) {
//   if (pwm < 20)
//      pwm = 20;

  if (pwm > 100)
     pwm = 100;

//    double _pwm =  (((double)pwm*1.0)/100.0)*60.0;
//    pwm = _pwm;
  return (int)pwm;   
}

int one_sec_counter = 0;
void compute_state_callback(TimerHandle_t timer) {
        diffDriveState.updateWheelPosition();
        diffDriveState.updateWheelState();

        unsigned int l_rpm = diffDriveState.GetLeftWheelRPM();
        unsigned int r_rpm = diffDriveState.GetRightWheelRPM();

        // Maps (a value of 0 to 1) and (a value of 1 to -1)
        diffDriveState.SetLeftWheelDirection( (LEFT_WHEEL_DIR * -2)+1 );
        gpio_put(LEFT_DIR_PIN, LEFT_WHEEL_DIR);

        int error = LEFT_WHEEL_RPM - l_rpm;  // calculate error
        lITerm += (_kI * (double)error); // calculate integral term
        double dInput = l_rpm - lLastSpeed; // calculate derivative
        int adjustment = (_kP * (double)error) + lITerm - (_kD * dInput);
        LEFT_PWM += adjustment;

        //limit speed to range of pwm 0-255
        LEFT_PWM = limitPWM(LEFT_PWM);

        if (LEFT_WHEEL_RPM == 0) {
            LEFT_PWM = 0;
            diffDriveState.SetLeftWheelStopped();
        }

        lLastSpeed = l_rpm;

        // Maps (a value of 0 to 1) and (a value of 1 to -1)
        diffDriveState.SetRightWheelDirection( (RIGHT_WHEEL_DIR * -2)+1 );
        gpio_put(RIGHT_DIR_PIN, RIGHT_WHEEL_DIR);
        
        
        error = RIGHT_WHEEL_RPM - r_rpm;  // calculate error
        rITerm += (_kI * (double)error); // calculate integral term
        dInput = r_rpm - rLastSpeed; // calculate derivative
        adjustment = (_kP * (double)error) + rITerm - (_kD * dInput);
        RIGHT_PWM += adjustment;

        // if ( r_rpm > RIGHT_WHEEL_RPM )
        //     RIGHT_PWM--;

        // if ( r_rpm < RIGHT_WHEEL_RPM )
        //     RIGHT_PWM++;

        //limit speed to range of pwm 0-255
        RIGHT_PWM = limitPWM(RIGHT_PWM);

        if (RIGHT_WHEEL_RPM == 0) {
            RIGHT_PWM = 0;
            diffDriveState.SetRightWheelStopped();
        }
     
        rLastSpeed = r_rpm;



        if(one_sec_counter == 20) {
            one_sec_counter = 0;
            IsMotorCmdReceived = false;
        }


        // if (IsMotorCmdReceived) {
            if(LEFT_PWM <= 60)
                analogWrite(LEFT_PWM_PIN, 200, LEFT_PWM);
            if(RIGHT_PWM <= 60)
                analogWrite(RIGHT_PWM_PIN, 200, RIGHT_PWM);
        // }
        // else {
        //     analogWrite(LEFT_PWM_PIN, 200, 0);
        //     analogWrite(RIGHT_PWM_PIN, 200, 0);
        //     DiffDriveState::getInstance()->SetLeftWheelStopped();
        //     DiffDriveState::getInstance()->SetRightWheelStopped();
        // }

        one_sec_counter++;

    // printf("\n%d %d %d %d %f %f",LEFT_PWM,  RIGHT_PWM, LEFT_WHEEL_VEL, LEFT_WHEEL_RPM, DiffDriveState::getInstance()->GetLeftWheelVelocity(), DiffDriveState::getInstance()->GetLeftWheelRPM() );
    
  printf("\n%ld %ld",diffDriveState.LeftEncoderTickCounter, diffDriveState.RightEncoderTickCounter );
        
   
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
    gpio_set_irq_enabled_with_callback (RIGHT_ENCODER_PIN_1, GPIO_IRQ_EDGE_FALL, true, [](uint gpio, uint32_t events){
                diffDriveState.IRQ_ENCODER(gpio, events);
            });

    gpio_set_irq_enabled_with_callback (LEFT_ENCODER_PIN_1,  GPIO_IRQ_EDGE_FALL, true, [](uint gpio, uint32_t events){
                diffDriveState.IRQ_ENCODER(gpio, events);
            });

    gpio_set_irq_enabled_with_callback (RIGHT_ENCODER_PIN_2, GPIO_IRQ_EDGE_FALL, true, [](uint gpio, uint32_t events){
                diffDriveState.IRQ_ENCODER(gpio, events);
            });

    gpio_set_irq_enabled_with_callback (LEFT_ENCODER_PIN_2,  GPIO_IRQ_EDGE_FALL, true, [](uint gpio, uint32_t events){
                diffDriveState.IRQ_ENCODER(gpio, events);
            });

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