#pragma once

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
#define COMPUTE_STATE_RATE 5 //hz
#define READ_CMD_RATE    20   //hz

#define COMPUTE_STATE_PERIOD_MS 1000/COMPUTE_STATE_RATE //Milliseconds
#define READ_CMD_PERIOD_MS    1000/READ_CMD_RATE        //Milliseconds

//Float to Integer scale factor for Serial State Commands
#define WRITE_STATE_SCALE_FACTOR 10000
//----------------------------------------------------------
