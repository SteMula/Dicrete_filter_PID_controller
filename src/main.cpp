/******************************************************************************
 *                                                                            *
/******************************************************************************
 *                                                                            *                                                    *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Stefano Mulargia
 */


#include <iostream>
#include <fstream>
#include <sstream>
#include <vector> 
#include <array>
#include "controller.h"
#include "plant.h"

int main(int argc, char *argv[]) {
    constexpr double K = 1.0;
    constexpr double Ts = 0.01;
    constexpr double gain = (K * Ts) / 2.0;
    double plant_reference = 0.0;
    double plant_output = 0.0;

    /*Code modified according to Nicolò advice*/
    std::array<double,5> kp = {10.0, 15.0, 15.0, 10.0, 15.0}; // default 10.0
    std::array<double,5> ki = {1.0 , 1.0 , 0.1 , 0.1 , 0.1 };   // default 1.0
    std::array<double,5> kd = {0.0 , 0.0 , 0.0 , 1.0 , 0.1};   // default 0.0 

    double e_comp = 0.0;
    double gain4 = 0.9*Ts*Ts/2;
    
    //Discrete controller coefficient
    std::vector<double> numeratorCoeffs1 = {0.00045, 0.0009, 0.00045};
    std::vector<double> denominatorCoeffs1 = {1.0, -1.921, 0.9231};

    //Reference plant coefficient
    std::vector<double> numeratorCoeffs2 = {1.0, 1.0};
    std::vector<double> denominatorCoeffs2 = {1.0, -1.0};

    //Coefficient that came out from Tustin bilinear trasform of
    // G(s) = 0.9/(0.5s^2 +s)
    std::vector<double> numeratorCoeffs4 = { 1.0, 2.0, 1.0};
    std::vector<double> denominatorCoeffs4 = {Ts+1.0, -2.0 ,-Ts+1};
    
    
    //Discrete Controller 1
    ZTransform filter1(numeratorCoeffs1, denominatorCoeffs1);

    //Reference Plant
    ZTransform filter2(numeratorCoeffs2, denominatorCoeffs2);

    //Discrete Controller 2
    ZTransform filter3(numeratorCoeffs1, denominatorCoeffs1);

    //Plant discretized according to Tustin method
    //s <- \frac{2(z-1)}{Ts(z+1)} 
    ZTransform filter4(numeratorCoeffs4, denominatorCoeffs4);
    int i = 0;
    for (auto& kp_value : kp) {
        // PID controller definition
        PIDController pid(kp_value, ki[kp_value], kd[kp_value], Ts);

        /*Open scope_data.tsv to get reference input signal*/
        std::ifstream file("data/scope_data.tsv");
        if (!file.is_open()) {
            std::cout << "Failed to open the file." << std::endl;
            return 1;
        }

        /*Create a tsv file called scope_data_stefano to store results*/
        std::ostringstream outputFileStream;
        outputFileStream << "data/scope_data_stefano_" << i << ".tsv";
        std::cout << "created output file: " << outputFileStream.str() << std::endl;
        std::ofstream outputFile(outputFileStream.str());
        if (!outputFile.is_open()) {
            std::cout << "Failed to create the output file." << std::endl;
            return 1;
        }

        /*Simulation*/
        std::string line;
        while (std::getline(file, line)) { /*until there is a line in the scope_data.tsv file*/
            /*From scope_data file extract 
            column1 : time
            column2 : reference input*/

            /*iss will allow you to extract values from the line string using stream extraction (>>) operations. 
            It provides functionality to parse and extract values from the string as if it were an input stream.*/
            std::istringstream iss(line);
            std::string column1, column2;
            if (!(iss >> column1 >> column2)) {
                std::cout << "Error reading line: " << line << std::endl;
                continue;
            }

            /*definition of a variable inputSample to store the current value of the refence signal */
            double inputSample;
            try {
                inputSample = std::stod(column2); //convert string into double.
            } catch (const std::exception& e) {
                std::cout << "Error converting input sample: " << column2 << std::endl;
                continue;
            }
            //compensation error computation.
            //it is important to do it before updating plant_output and plant_reference 
            //since we are only able to use the value a instant "t-1".
            e_comp = pid.compute(plant_output, plant_reference);

            //error between reference(t) and plant reference(t-1) 
            double reference_plantreference = inputSample - plant_reference;

            //response of Discrete Controller 1 with "reference_plantreference" as input
            double controller_reference = filter1.processSample(reference_plantreference);

            //response of Referense Plant with controller_reference as input
            plant_reference = gain * filter2.processSample(controller_reference);

            //error between reference(t) and plant output(t-1) 
            double reference_plantoutput = inputSample - plant_output;
                
            //response of Discrete Controller 1 with "reference_plantoutput" as input

            double controller_output = filter3.processSample(reference_plantoutput) + e_comp;

            /*response of plant discretized according to Tustin bilinear transformation 1 with "controller_output" as input*/
            plant_output = gain4*filter4.processSample(controller_output) ;

            /*save data in outputFile namely scope_data.tsv*/
            outputFile << column1 << '\t' << inputSample   << '\t' << plant_reference  <<'\t' << plant_output  << std::endl;

        }
        i++;
        file.close();
        outputFile.close();
    }
    return EXIT_SUCCESS;
}
