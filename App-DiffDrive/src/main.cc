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
#include <algorithm>

#include "hardware/sync.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"

#include "constants.h"
#include "pico_pwm.h"
#include "DiffDriveState.h"

#include "PID_v1.h"

#define DEBUG_SERIAL 1


#define COMPUTE_STATE_RATE 20
#define READ_CMD_RATE    20

#define COMPUTE_STATE_PERIOD_MS 1000/COMPUTE_STATE_RATE
#define READ_CMD_PERIOD_MS    1000/READ_CMD_RATE

#define TIMER_ID_COMPUTE_STATE  2
#define TIMER_ID_READ_CMD     3

#define WRITE_STATE_SCALE_FACTOR 10000

uint8_t LEFT_PWM = 0;
uint8_t RIGHT_PWM = 0;
uint32_t LEFT_WHEEL_VEL = 0;
uint32_t RIGHT_WHEEL_VEL = 0;
uint8_t LEFT_WHEEL_DIR = 0;
uint8_t RIGHT_WHEEL_DIR = 0;


DiffDriveState* DiffDriveState::instancePtr = NULL;

// FROM 1.0.0 Record references to the tasks
TaskHandle_t hw_interface_timer_schedule_handle = NULL;

volatile TimerHandle_t compute_state_timer;
volatile TimerHandle_t read_command_timer;


double Setpoint, Input, Output;
double Kp=10, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);



//Write back the wheek state such as position and velocity
void write_state() {
    
    // printf("\n Left Position %f", DiffDriveState::getInstance()->GetLeftWheelPosition()*180.0/3.14);
    // printf("\n Right Position %f", DiffDriveState::getInstance()->GetRightWheelPosition()*180.0/3.14);
    // printf("\n Left Velocity %f", DiffDriveState::getInstance()->GetLeftWheelVelocity());
    // printf("\n Right Velocity %f", DiffDriveState::getInstance()->GetRightWheelVelocity());

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


#define BYTES_SF  0x1FA8
#define BYTES_EF  0x1FA9
enum RX_STATES {SF, LEN, DATA, CHK, EF};
std::vector<uint8_t> RX_BUFFER;
std::vector<std::vector<uint8_t>> RX_FRAMES;

//perform operation as per the command
void handle_command(){
    if(RX_FRAMES.size() > 0){

        uint8_t COMMAND_TYPE = RX_FRAMES.at(0).at(0);

        switch(COMMAND_TYPE){
            case 0x10:
            {

                uint32_t tLEFT_WHEEL_VEL = RX_FRAMES.at(0).at(2);
                tLEFT_WHEEL_VEL = tLEFT_WHEEL_VEL << 24 | RX_FRAMES.at(0).at(3);
                tLEFT_WHEEL_VEL = tLEFT_WHEEL_VEL << 16 | RX_FRAMES.at(0).at(4);
                tLEFT_WHEEL_VEL = tLEFT_WHEEL_VEL << 8 | RX_FRAMES.at(0).at(5);

                uint32_t tRIGHT_WHEEL_VEL = RX_FRAMES.at(0).at(7);
                tRIGHT_WHEEL_VEL = tRIGHT_WHEEL_VEL << 24 | RX_FRAMES.at(0).at(8);
                tRIGHT_WHEEL_VEL = tRIGHT_WHEEL_VEL << 16 | RX_FRAMES.at(0).at(9);
                tRIGHT_WHEEL_VEL = tRIGHT_WHEEL_VEL << 8 | RX_FRAMES.at(0).at(10);

                LEFT_WHEEL_DIR =  RX_FRAMES.at(0).at(1);
                RIGHT_WHEEL_DIR =  RX_FRAMES.at(0).at(6);

                LEFT_WHEEL_VEL = tLEFT_WHEEL_VEL;
                RIGHT_WHEEL_VEL = tRIGHT_WHEEL_VEL;

            }
            break;

            case 0x20:{
                write_state();
            }
            break;
        }
        
    }
}

void compute_state_callback(TimerHandle_t timer) {
    DiffDriveState::getInstance()->updateWheelPosition();
    DiffDriveState::getInstance()->updateWheelState();


    // Input = (DiffDriveState::getInstance()->GetLeftWheelVelocity()/1000)*100/1.8;
    // myPID.Compute();
    // printf("\n%f, %f, %f", Setpoint, Input, Output);

    // if(Setpoint != 0)
    // {
    //     analogWrite(LEFT_PWM_PIN, 200, Output);
    //     analogWrite(RIGHT_PWM_PIN, 200, 0);
    // }

    if(LEFT_WHEEL_VEL != 0){
        // Maps (a value of 0 to 1) and (a value of 1 to -1)
        DiffDriveState::getInstance()->SetLeftWheelDirection( (LEFT_WHEEL_DIR * -2)+1 );
        gpio_put(LEFT_DIR_PIN, LEFT_WHEEL_DIR);

        if(LEFT_WHEEL_VEL < DiffDriveState::getInstance()->GetLeftWheelVelocity()){
            LEFT_PWM--;
            if(LEFT_PWM < 55) LEFT_PWM = 55;
            analogWrite(LEFT_PWM_PIN, 200, LEFT_PWM);
        }
        else
        if(LEFT_WHEEL_VEL > DiffDriveState::getInstance()->GetLeftWheelVelocity()){
            LEFT_PWM++;
            if(LEFT_PWM > 80) LEFT_PWM = 80;
            analogWrite(LEFT_PWM_PIN, 200, LEFT_PWM);
        }
    }

    if(RIGHT_WHEEL_VEL != 0){
        // Maps (a value of 0 to 1) and (a value of 1 to -1)
        DiffDriveState::getInstance()->SetRightWheelDirection( (RIGHT_WHEEL_DIR * -2)+1 );
        gpio_put(RIGHT_DIR_PIN, RIGHT_WHEEL_DIR);

        if(RIGHT_WHEEL_VEL < DiffDriveState::getInstance()->GetRightWheelVelocity()){
            RIGHT_PWM--;
            if(RIGHT_PWM < 55) RIGHT_PWM = 55;
            analogWrite(RIGHT_PWM_PIN, 200, RIGHT_PWM);
        }
        else 
        if(RIGHT_WHEEL_VEL > DiffDriveState::getInstance()->GetRightWheelVelocity()){
            RIGHT_PWM++;
            if(RIGHT_PWM > 80) RIGHT_PWM = 80;
            analogWrite(RIGHT_PWM_PIN, 200, RIGHT_PWM);
        }
    }


    if(LEFT_WHEEL_VEL == 0){
        LEFT_PWM = 0;
        analogWrite(LEFT_PWM_PIN, 200, LEFT_PWM);
    }
    if(RIGHT_WHEEL_VEL == 0){
        RIGHT_PWM = 0;
        analogWrite(RIGHT_PWM_PIN, 200, RIGHT_PWM);
    }

    // printf("\n%d %d %d %d %f %f",LEFT_PWM,  RIGHT_PWM, LEFT_WHEEL_VEL, RIGHT_WHEEL_VEL, DiffDriveState::getInstance()->GetRightWheelVelocity(), DiffDriveState::getInstance()->GetRightWheelVelocity() );
    
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
    while(RX_BUFFER.size() > 3){

        //Validate start of frame bytes
        if(RX_BUFFER.at(0) == (BYTES_SF>>8 & 0xFF) && 
        RX_BUFFER.at(1) == (BYTES_SF    & 0xFF)){
                
                //Get frame legth
                uint8_t length = RX_BUFFER.at(2);

                //Wait and read complete frame length
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



void hw_interface_timer_scheduler(void* arg){

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

//Initialize GPIO pins
void init_gpio(){
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


static char event_str[128];

void gpio_event_string(char *buf, uint32_t events);


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


/*
 * RUNTIME START
 */
int main() {

    // Initialize chosen serial port
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0,100);
    stdio_init_all();
    init_gpio();

    printf("Hello GPIO IRQ\n");
    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_FALL, true, &IRQ_ENCODER);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN,  GPIO_IRQ_EDGE_FALL, true, &IRQ_ENCODER);

    //Main task that handles hw interface's read and write timers
    BaseType_t hw_interface_status = xTaskCreate(hw_interface_timer_scheduler, 
                                         "TIMER_INITIALIZER", 
                                         128, 
                                         NULL, 
                                         1, 
                                         &hw_interface_timer_schedule_handle); 
    
    // Start the FreeRTOS scheduler
    // FROM 1.0.1: Only proceed with valid tasks
    if (hw_interface_status == pdPASS) 
    {
        vTaskStartScheduler();
    }
    
    while(true)
    {

    }
}


static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}