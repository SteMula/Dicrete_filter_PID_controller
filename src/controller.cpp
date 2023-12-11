/******************************************************************************
 *                                                                            *                                                    *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Stefano Mulargia
 */

#include "controller.h"

PIController::PIController(double Kp_, double Ki_, double T_) {
    Kp = Kp_;
    Ki = Ki_;
    T = T_;
    integral = 0.0;
    integralMax = 1000.0; // Set the maximum value as needed
}

double PIController::compute(double measurement, double setpoint) {
    double error = setpoint - measurement;
    integral += error * T;

    // Perform integral wind-up handling
    if (integral > integralMax) {
        integral = integralMax;
    } else if (integral < -integralMax) {
        integral = -integralMax;
    }

    double command = Kp * error + Ki * integral;
    return command;
}

PIDController::PIDController(double Kp_, double Ki_, double Kd_, double T_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    T = T_;
    integral = 0.0;
    integralMax = 1000.0; // Set the maximum value as needed
    previousError = 0.0;
}

double PIDController::compute(double measurement, double setpoint) {
    double error = setpoint - measurement;
    integral += error * T;

    // Perform integral wind-up handling
    if (integral > integralMax) {
        integral = integralMax;
    } else if (integral < -integralMax) {
        integral = -integralMax;
    }

    double derivative = (error - previousError) / T;
    previousError = error;

    double command = Kp * error + Ki * integral + Kd * derivative;
    return command;
}
