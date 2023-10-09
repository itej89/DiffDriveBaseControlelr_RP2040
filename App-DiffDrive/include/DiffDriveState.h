#pragma once

#include <cmath>
#include <stdint.h>
#include <time.h>

#define WHEEL_RADIUS   95
#define WHEEL_PERIMETER  2 * 3.14 * WHEEL_RADIUS

#define ENCODER_COUNT_PER_REV 80

#define RADIANS_PER_COUNT   2*3.14/ ENCODER_COUNT_PER_REV
#define DISTANCE_PER_COUNT  WHEEL_PERIMETER/ENCODER_COUNT_PER_REV

//MM_PER_SEC
#define MAX_VEL_CLAMP 7500

//Radians
#define MAX_POS_CLAMP 3.14



class DiffDriveState{
    private:
        int64_t LeftEncoderDirectionalTickCounter = 0;
        int64_t RightEncoderDirectionalTickCounter = 0;

        int64_t LeftEncoderTickCounter = 0;
        int64_t RightEncoderTickCounter = 0;

        int8_t LeftWheelDirection = 1;
        int8_t RightWheelDirection = 1;

        double LeftWheelPosition = 0;
        double RightWheelPosition = 0;

        double LeftWheelVelocity = 0;
        double RightWheelVelocity = 0;

        static DiffDriveState* instancePtr;

        int64_t LastLeftEncoderTickCounter = 0;
        int64_t LastRightEncoderTickCounter = 0;
        clock_t LastStateUpdateTime;
        DiffDriveState(){
            LastStateUpdateTime = clock();
        }

        clock_t clock(){
            return (clock_t) time_us_64() / 10000;}
        


    public:

        DiffDriveState(const DiffDriveState& obj)= delete;

        static DiffDriveState* getInstance()
        {
            // If there is no instance of class
            // then we can create an instance.
            if (instancePtr == NULL)
            {
            // We can access private members
            // within the class.
            instancePtr = new DiffDriveState();
            
            // returning the instance pointer
            return instancePtr;
            }
            else
            {
            // if instancePtr != NULL that means
            // the class already have an instance.
            // So, we are returning that instance
            // and not creating new one.
            return instancePtr;
            }
        }
        void SetLeftWheelDirection(int8_t dir ) { LeftWheelDirection = dir; }
        void SetRightWheelDirection(int8_t dir ) { RightWheelDirection = dir; }
 
        float GetLeftWheelPosition() { return LeftWheelPosition;  }
        float GetRightWheelPosition(){ return RightWheelPosition; }
 
        float GetLeftWheelVelocity() { return LeftWheelVelocity;  }
        float GetRightWheelVelocity(){ return RightWheelVelocity; }
 
        void SetLeftWheelStopped() { LeftWheelVelocity=0;  }
        void SetRightWheelStopped(){ RightWheelVelocity=0; }


        void UpdateLeftEncoderCount() { 
            LeftEncoderDirectionalTickCounter += LeftWheelDirection*1;
            LeftEncoderTickCounter += 1;
        }


        void UpdateRightEncoderCount(){ 
            RightEncoderDirectionalTickCounter += RightWheelDirection*1; 
            RightEncoderTickCounter += 1;
        }

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

        void updateWheelState(){
            
            clock_t nowTime = clock();
            double executionTime = ((double)(nowTime - LastStateUpdateTime) / CLOCKS_PER_SEC)+0.00001;
            
            if(LastLeftEncoderTickCounter != LeftEncoderTickCounter){
                double tempLeftWheelVelocity = std::abs(LastLeftEncoderTickCounter - LeftEncoderTickCounter)*DISTANCE_PER_COUNT/executionTime;
                LeftWheelVelocity = tempLeftWheelVelocity > MAX_VEL_CLAMP ? MAX_VEL_CLAMP : tempLeftWheelVelocity;
            }

            if(LastRightEncoderTickCounter != RightEncoderTickCounter){
                double tempRightWheelVelocity = std::abs( LastRightEncoderTickCounter - RightEncoderTickCounter)*DISTANCE_PER_COUNT/executionTime;
                RightWheelVelocity = tempRightWheelVelocity > MAX_VEL_CLAMP ? MAX_VEL_CLAMP : tempRightWheelVelocity;
            }
        

            LastStateUpdateTime = nowTime;

            LastLeftEncoderTickCounter = LeftEncoderTickCounter;
            LastRightEncoderTickCounter = RightEncoderTickCounter;

        }
};