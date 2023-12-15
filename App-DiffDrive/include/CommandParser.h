#pragma once

#include <vector>


#include "pico/stdio.h"
#include "DiffDriveState.h"

/**
 * @brief COMMAND INFORMATION ----------------------------------------------------------
 * -------------------------------------------------------------------------------------
 */
// Requested  Direction
uint8_t LEFT_WHEEL_DIR = 0;
uint8_t RIGHT_WHEEL_DIR = 0;

// Requested  Velocity
uint32_t LEFT_WHEEL_VEL = 0;
uint32_t RIGHT_WHEEL_VEL = 0;

// Requested  RPM
uint32_t LEFT_WHEEL_RPM = 0;
uint32_t RIGHT_WHEEL_RPM = 0;
//------------------------------------------------------------------





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



//Read velocity and rpm from the latest command
void read_state(std::vector<uint8_t>& frame) {

    uint32_t tLEFT_WHEEL_VEL = frame.at(2);
    tLEFT_WHEEL_VEL = tLEFT_WHEEL_VEL << 24 | frame.at(3);
    tLEFT_WHEEL_VEL = tLEFT_WHEEL_VEL << 16 | frame.at(4);
    tLEFT_WHEEL_VEL = tLEFT_WHEEL_VEL << 8  | frame.at(5);

    uint32_t tRIGHT_WHEEL_VEL = frame.at(7);
    tRIGHT_WHEEL_VEL = tRIGHT_WHEEL_VEL << 24 | frame.at(8);
    tRIGHT_WHEEL_VEL = tRIGHT_WHEEL_VEL << 16 | frame.at(9);
    tRIGHT_WHEEL_VEL = tRIGHT_WHEEL_VEL << 8 | frame.at(10);

    LEFT_WHEEL_DIR  =  frame.at(1);
    RIGHT_WHEEL_DIR =  frame.at(6);

    //Recieved Wheel velocities in  mm per second
    LEFT_WHEEL_VEL  = tLEFT_WHEEL_VEL;
    RIGHT_WHEEL_VEL = tRIGHT_WHEEL_VEL;

    //Converted Wheel RPM
    double lomega = (LEFT_WHEEL_VEL*1.0) / (WHEEL_RADIUS*1.0);
    double lrps = lomega / (2 * 3.14);
    double lrpm = lrps * 60;
    LEFT_WHEEL_RPM  = (uint32_t)lrpm;

    double romega = (RIGHT_WHEEL_VEL*1.0) / (WHEEL_RADIUS*1.0);
    double rrps = romega / (2 * 3.14);
    double rrpm = rrps * 60;
    RIGHT_WHEEL_RPM  = (uint32_t)rrpm;

}



void handle_command(std::vector<uint8_t>& frame) {
        uint8_t COMMAND_TYPE = frame.at(0);
        switch(COMMAND_TYPE){
            case 0x10:
            {
                read_state(frame);
            }
            break;

            case 0x20:{
                write_state();
            }
            break;
        }
}

