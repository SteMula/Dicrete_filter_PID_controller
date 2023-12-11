/******************************************************************************
 *                                                                            *                                                    *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Stefano Mulargia
 */

 #ifndef Z_TRANSFORM_H
#define Z_TRANSFORM_H

#include <vector>

class ZTransform {
private:
    std::vector<double> numeratorCoefficients;
    std::vector<double> denominatorCoefficients;
    std::vector<double> inputHistory;
    std::vector<double> outputHistory;

public:
    ZTransform(std::vector<double> numCoeffs, std::vector<double> denCoeffs);

    double processSample(double input);
};


class DigitalFilter {
public:
    DigitalFilter(double numCoeffs[], int numSize, double denCoeffs[], int denSize);
    ~DigitalFilter();
    double processSample(double input);

private:
    double* numeratorCoefficients;
    double* denominatorCoefficients;
    double* inputHistory;
    double* outputHistory;
    int numeratorSize;
    int denominatorSize;
};

#endif
