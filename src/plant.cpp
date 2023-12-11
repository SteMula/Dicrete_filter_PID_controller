/******************************************************************************
 *                                                                            *                                                    *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Stefano Mulargia
 */

#include "plant.h"

/*Implementation of a transfer function for a linear, time-invariant. 
Digital filter can be expressed as a transfer function in the Z-domain;
If it is causal it is in the form 
        b0 + b_1z^{-1} + ... + b_N z^{-N}
H(z) = -----------------------------------
        a0 + a_1z^{-1} + ... + a_N z^{-N})
        
https://en.wikipedia.org/wiki/Digital_filter
*/

ZTransform::ZTransform(std::vector<double> numCoeffs, std::vector<double> denCoeffs) {
    numeratorCoefficients = numCoeffs;
    denominatorCoefficients = denCoeffs;
    inputHistory.resize(numeratorCoefficients.size(), 0.0);
    outputHistory.resize(denominatorCoefficients.size() - 1, 0.0);
}

/* In order to compute the response y(n) it translates to the difference equation:
 a0 * y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] + b3*x[n-3] + ... 
               - a1*y[n-1] - a2*y[n-2] - a3*y[n-3] - ...
*/
double ZTransform::processSample(double input) {
    inputHistory.insert(inputHistory.begin(), input);
    inputHistory.pop_back();

    double output = 0.0;

    // Calculate the output using the z-transform equation
    for (int i = 0; i < numeratorCoefficients.size(); i++) {
        output += numeratorCoefficients[i] * inputHistory[i];
    }

        
    /*for loop start from 1 because we have -a1*y[n-1] and a minus in front of the coefficient */
    for (int i = 1; i < denominatorCoefficients.size(); i++) {
        output -= denominatorCoefficients[i] * outputHistory[i - 1];
    }
    
    /*we divide by a0 to get y(n)*/
    output /= denominatorCoefficients[0]; // Divide by a0

    outputHistory.insert(outputHistory.begin(), output);
    outputHistory.pop_back();

    return output;
}



DigitalFilter::DigitalFilter(double numCoeffs[], int numSize, double denCoeffs[], int denSize) {
    numeratorSize = numSize;
    denominatorSize = denSize;

    numeratorCoefficients = new double[numSize];
    denominatorCoefficients = new double[denSize];
    inputHistory = new double[numSize];
    outputHistory = new double[denSize - 1];

    // Copy coefficients
    for (int i = 0; i < numSize; i++) {
        numeratorCoefficients[i] = numCoeffs[i];
        inputHistory[i] = 0.0;
    }

    for (int i = 0; i < denSize; i++) {
        denominatorCoefficients[i] = denCoeffs[i];
        if (i < denSize - 1) {
            outputHistory[i] = 0.0;
        }
    }
}

DigitalFilter::~DigitalFilter() {
    delete[] numeratorCoefficients;
    delete[] denominatorCoefficients;
    delete[] inputHistory;
    delete[] outputHistory;
}

double DigitalFilter::processSample(double input) {
    // Shift input history
    for (int i = numeratorSize - 1; i > 0; i--) {
        inputHistory[i] = inputHistory[i - 1];
    }
    inputHistory[0] = input;

    double output = 0.0;

    // Calculate the output using the difference equation
    for (int i = 0; i < numeratorSize; i++) {
        output += numeratorCoefficients[i] * inputHistory[i];
    }

    // Calculate the feedback using the difference equation
    for (int i = 1; i < denominatorSize; i++) {
        output -= denominatorCoefficients[i] * outputHistory[i - 1];
    }

    // Divide by a0 to get y(n)
    output /= denominatorCoefficients[0];

    // Shift output history
    for (int i = denominatorSize - 2; i > 0; i--) {
        outputHistory[i] = outputHistory[i - 1];
    }
    outputHistory[0] = output;

    return output;
}
