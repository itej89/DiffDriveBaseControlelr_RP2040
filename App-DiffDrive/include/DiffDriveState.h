#pragma once

#include <cmath>
#include <stdint.h>
#include <time.h>

#include "Constants.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"

#define WHEEL_RADIUS   95
#define WHEEL_PERIMETER  2 * 3.14 * WHEEL_RADIUS

#define ENCODER_COUNT_PER_REV 1200

#define RADIANS_PER_COUNT   2*3.14/ ENCODER_COUNT_PER_REV
#define DISTANCE_PER_COUNT  WHEEL_PERIMETER/ENCODER_COUNT_PER_REV

//MAX RPM ; RPM = ( (V/r) / (2*pi) ) * 60
#define MAX_RPM_CLAMP 700

//MM_PER_SEC
#define MAX_VEL_CLAMP 7500

//Radians
#define MAX_POS_CLAMP 3.14



class DiffDriveState{
    public:
        //Count with direction for postion computation
        long LeftEncoderDirectionalTickCounter = 0;
        long RightEncoderDirectionalTickCounter = 0;

        //Always counts up for velcoity computation
        unsigned long LeftEncoderTickCounter = 0;
        unsigned long RightEncoderTickCounter = 0;

        //Current state-------------------------
        int8_t LeftWheelDirection = 1;
        int8_t RightWheelDirection = 1;

        double LeftWheelPosition = 0;
        double RightWheelPosition = 0;

        double LeftWheelVelocity = 0;
        double RightWheelVelocity = 0;

        unsigned int LeftWheelRPM = 0;
        unsigned int RightWheelRPM = 0;
        //---------------------------------------


        //Used to check if the wheel has turned------
        int64_t LastLeftEncoderTickCounter = 0;
        int64_t LastRightEncoderTickCounter = 0;
        clock_t LastStateUpdateTime;
        //-------------------------------------------

        


        clock_t clock(){
            return (clock_t) time_us_64() / 10000;}

        
        


    public:
        DiffDriveState(){
                    LastStateUpdateTime = clock();
                }
       
       
        //Getters and setters------------------------------------------------------
        int64_t GetLeftEncoderCount() { return LeftEncoderTickCounter;  }
        int64_t GetRightEncoderCount(){ return RightEncoderTickCounter; }

        void SetLeftWheelDirection(int8_t dir ) { LeftWheelDirection = dir; }
        void SetRightWheelDirection(int8_t dir ) { RightWheelDirection = dir; }
 
        float GetLeftWheelPosition() { return LeftWheelPosition;  }
        float GetRightWheelPosition(){ return RightWheelPosition; }
 
        float GetLeftWheelVelocity() { return LeftWheelVelocity;  }
        float GetRightWheelVelocity(){ return RightWheelVelocity; }
 
        float GetLeftWheelRPM() { return LeftWheelRPM;  }
        float GetRightWheelRPM(){ return RightWheelRPM; }
 
        void SetLeftWheelStopped() { LeftWheelVelocity=0;  LeftWheelRPM = 0;}
        void SetRightWheelStopped(){ RightWheelVelocity=0; RightWheelRPM = 0;}
        //---------------------------------------------------------------------------


        /**
         * @brief Encoder Interrupt handle
         * 
         */
        void IRQ_ENCODER(uint gpio, uint32_t events) {  
            if (gpio == 8 || gpio == 9){
                UpdateLeftEncoderCount();
            }

            if (gpio == 10 || gpio == 11) {
                UpdateRightEncoderCount();
            }
        }

        void UpdateLeftEncoderCount() {
            LeftEncoderDirectionalTickCounter += LeftWheelDirection*1;
            LeftEncoderTickCounter = LeftEncoderTickCounter+1;
        }


        void UpdateRightEncoderCount(){
            RightEncoderDirectionalTickCounter += RightWheelDirection*1; 
            RightEncoderTickCounter = RightEncoderTickCounter+1;
        }
        //---------------------------------------------------------------------------



        /**
         * @brief Computes Wheel position
         * 
         */
        void updateWheelPosition(){

            //Update Left wheel position and velocity
            LeftWheelPosition = (LeftEncoderDirectionalTickCounter % ENCODER_COUNT_PER_REV) * RADIANS_PER_COUNT;
            LeftWheelPosition = std::ceil(LeftWheelPosition * 100.0) / 100.0;
            if(LeftWheelPosition < 0) LeftWheelPosition = 6.28 + LeftWheelPosition;


            //Update Right wheel position and velocity
            RightWheelPosition = (RightEncoderDirectionalTickCounter % ENCODER_COUNT_PER_REV) * RADIANS_PER_COUNT;
            RightWheelPosition = std::ceil(RightWheelPosition * 100.0) / 100.0;
            if(RightWheelPosition < 0) RightWheelPosition = 6.28  + RightWheelPosition;

        }
        //---------------------------------------------------------------------------


        /**
         * @brief Computes Wheel Velocity and RPM
         * 
         */
        void updateWheelState(){
            
            clock_t nowTime = clock();
            double executionTime = ((double)(nowTime - LastStateUpdateTime) / CLOCKS_PER_SEC)+0.00001;
            
            if(LastLeftEncoderTickCounter != LeftEncoderTickCounter){
                double left_ticks_per_second = std::abs(LastLeftEncoderTickCounter - LeftEncoderTickCounter)/executionTime;
                double left_rev_per_sec  = left_ticks_per_second/ENCODER_COUNT_PER_REV;
                double temp_left_rpm  = left_rev_per_sec * 60;
                LeftWheelRPM = (unsigned int)(temp_left_rpm > MAX_RPM_CLAMP ? MAX_RPM_CLAMP : temp_left_rpm);

                double tempLeftWheelVelocity = left_ticks_per_second*DISTANCE_PER_COUNT;
                LeftWheelVelocity = tempLeftWheelVelocity > MAX_VEL_CLAMP ? MAX_VEL_CLAMP : tempLeftWheelVelocity;
            }

            if(LastRightEncoderTickCounter != RightEncoderTickCounter){
                double right_ticks_per_second = std::abs(LastRightEncoderTickCounter - RightEncoderTickCounter)/executionTime;
                double right_rev_per_sec  = right_ticks_per_second/ENCODER_COUNT_PER_REV;
                double temp_right_rpm  = right_rev_per_sec * 60;
                RightWheelRPM = (unsigned int)(temp_right_rpm > MAX_RPM_CLAMP ? MAX_RPM_CLAMP : temp_right_rpm);

                double tempRightWheelVelocity = right_ticks_per_second*DISTANCE_PER_COUNT;
                RightWheelVelocity = tempRightWheelVelocity > MAX_VEL_CLAMP ? MAX_VEL_CLAMP : tempRightWheelVelocity;
            }
        
            LastStateUpdateTime = nowTime;

            LastLeftEncoderTickCounter = LeftEncoderTickCounter;
            LastRightEncoderTickCounter = RightEncoderTickCounter;

        }
        //---------------------------------------------------------------------------
};
