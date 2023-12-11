/******************************************************************************
 *                                                                            *                                                    *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Stefano Mulargia
 */

#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

class PIController {
private:
    double Kp;              // Proportional gain constant
    double Ki;              // Integral gain constant
    double T;               // Time step
    double integral;        // Integral term
    double integralMax;     // Maximum integral value to prevent wind-up

public:
    // Constructor
    PIController(double Kp_, double Ki_, double T_);

    // Compute the control command
    double compute(double measurement, double setpoint);
};



class PIDController {
public:
    PIDController(double Kp_, double Ki_, double Kd_, double T_);

    double compute(double measurement, double setpoint);

private:
    double Kp;
    double Ki;
    double Kd;
    double T;
    double integral;
    double integralMax;
    double previousError;
};



#endif

