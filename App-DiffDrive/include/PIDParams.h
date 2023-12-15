/**
 * @brief PID CONTEXT ------------------------------------------------------------------
 */
// PID Coefficients
double _kP = 0.5;
double _kI = 0.006;
double _kD = 0.000000000625;

// PID Data
double rITerm = 0.0;
double lITerm = 0.0;

// Current Speed
int rLastSpeed = 0;
int lLastSpeed = 0;


// Current PWM
uint8_t LEFT_PWM = 0;
uint8_t RIGHT_PWM = 0;
//--------------------------------------------------------------------------------------
