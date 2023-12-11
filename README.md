# Compile
To run the code just open a new terminal and run "compile.sh" and everything will be executed and plotted.

# Report

The following report provides an overview and explanation of the code provided.
## Revision note
- added a chapter at the end explaining the modification that has been implemented in the code.

## Introduction
The Reference Model Control (RMC) scheme offers a reliable and well-established framework for achieving desired control performance in various applications. By employing a PI compensator, the control system aims to minimize the discrepancy between the reference plant output and the actual plant output. This discrepancy, commonly referred to as the control error, is continuously monitored and adjusted to maintain the desired control behavior.
| |
| :---: |
| ![](/assets/controller_plant.png) |
| The system consisting of the controller and the plant (🔘 click on the image to enlarge) |

## Overview of the code
The code is a C++ program that simulates the control system shown above.
It implements a discrete-time controller and a plant model to generate an output based on a reference input. The program reads input data from a file, performs the simulation, and writes the results to another file.


## Code Structure
The code consists of several sections, including:

1. Header inclusion: The necessary header files are included, such as `iostream`, `fstream`, `sstream`, `vector`, `cmath`, as well as the custom headers `controller.h` and `plant.h`.
2. Constants and Variables: Constant values and variables are declared and initialized, including system parameters and coefficients for different components.
3. Component Initialization: Instances of various components, such as controllers and filters, are created and initialized using the specified coefficients.
4. File Handling: The program opens input and output files for reading and writing data.
5. Simulation: The main simulation loop is implemented, where each line of the input file is processed to obtain the reference input. The control algorithm is applied, and the resulting output is computed and written to the output file.
6. File Closing: The input and output files are closed.
7. Program Termination: The program terminates with a success status.

## Components
The code utilizes several components to perform the control simulation. These components include:

- Discrete Controller: Represented by the `ZTransform` class, it applies a discrete-time transfer function to process input signals.
- Reference Plant: Another instance of the `ZTransform` class, it represents a plant model for the reference signal.
- PI Controller: Implemented as the `PIController` class, it computes the compensation error using proportional and integral gains.
- Plant Discretization: The `filter4`, an instance of`ZTransform` has been used after manually computation of the Tustin bilinear transform to discretize the continuous plant model.

## File Handling
The code handles input and output files as follows:

- Input File: The program attempts to open the file named "scope_data.tsv" for reading. If the file fails to open, an error message is displayed, and the program terminates.
- Output File: The program creates a new file named "scope_data_stefano.tsv" for writing the simulation results. If the file creation fails, an error message is displayed, and the program terminates.
## Discrete Controller implementation
Here is shown the block diagram of the Discre Controller that we have to implement
| |
| :---: |
| ![](/assets/controller.png) |
| The internal block diagram of the controller (🔘 click on the image to enlarge) |

A digital filter can be defined by its transfer function or difference equation, which allows us to analyze its response to various inputs. By examining the transfer function mathematically, we can understand how the filter will behave in different scenarios. The process of designing a filter involves establishing appropriate specifications for the problem at hand, such as a desired cutoff frequency for a second-order low-pass filter. Subsequently, we derive a transfer function that satisfies these specifications.

The transfer function of a linear and time-invariant digital filter can be represented in the Z-domain. If the filter is causal, it follows a specific form.
$$H(z)={\frac  {B(z)}{A(z)}}={\frac  {{b_{{0}}+b_{{1}}z^{{-1}}+b_{{2}}z^{{-2}}+\cdots +b_{{N}}z^{{-N}}}}{{1+a_{{1}}z^{{-1}}+a_{{2}}z^{{-2}}+\cdots +a_{{M}}z^{{-M}}}}}$$


To implement it we defined a function called ZTransform that takes as input two vectors, one for $A(z)$ coeffients and one for $B(z)$.
```
ZTransform::ZTransform(std::vector<double> numCoeffs, std::vector<double> denCoeffs) {
    numeratorCoefficients = numCoeffs;
    denominatorCoefficients = denCoeffs;
    inputHistory.resize(numeratorCoefficients.size(), 0.0);
    outputHistory.resize(denominatorCoefficients.size() - 1, 0.0);
}
```
In order to compute the response y(n) it translates to the difference equation:

$$
a_0\cdot y[n] = b_0\cdot x[n] + b_1\cdot x[n-1] + \dots + b_N*x[n-N] - a_1 \cdot y[n-1] - \dots - a_N\cdot y[n-N]
$$

```
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
```
## Tustin bilinear transformation

The bilinear transform (also known as Tustin's method, after Arnold Tustin) is used in digital signal processing and discrete-time control theory to transform continuous-time system representations to discrete-time and vice versa.

The model expressed as Laplace transform is the following

$$\frac{0.9}{0.5s^2 +s}$$


To trasform it in its discrete time version we apply the following substitution $s = \frac{2(z-1)}{T_s(z+1)}$.

$$
\frac{0.9}{0.5 \left(\frac{2(z-1)}{T_s(z+1)}\right)^2 + \frac{2(z-1)}{T_s(z+1)}}
$$

Next, we can simplify the expression further by expanding the square in the denominator:

$$
\frac{0.9}{0.5 \cdot \frac{4(z-1)^2}{T_s^2(z+1)^2} + \frac{2(z-1)}{T_s(z+1)}}
$$

Simplifying the expression inside the denominator:

$$
\frac{0.9}{\frac{2(z-1)^2}{T_s^2(z+1)^2} + \frac{2(z-1)}{T_s(z+1)}}
$$

Next, we can simplify the expression by multiplying through by $T_s^2(z+1)^2$ to eliminate the fractions:

$$
\frac{0.9 \cdot T_s^2(z+1)^2}{2(z-1)^2 + 2(z-1)T_s(z+1)}
$$

Simplifying further:

$$
=\frac{0.9 \cdot T_s^2(z+1)^2}{2(z-1) \cdot ((z-1) + T_s(z+1))}
$$

$$
=\frac{0.9 \cdot T_s^2(z+1)^2}{2(z-1) \cdot (z + T_s(z+1) - T_s)}
$$

$$
=\frac{0.9 \cdot T_s^2(z+1)^2}{2(z-1) \cdot (z(1 + T_s) + T_s - 1)}
$$

$$
=\frac{0.9 \cdot T_s^2(z+1)^2}{2(z-1) \cdot (z(1 + T_s) + T_s - 1)}
$$

$$
=0.45 \cdot {T_s}^2\frac{z^2 +2z +1}{z^2(1 + T_s) -2T_s -T_s + 1)}
$$

Once we have the coefficient for the digital version we can consider it as done before for Digital Controller 1 and 2.
## PI Controllers and Formulas

The code utilizes a PI (Proportional-Integral) controller to compute the compensation error for the control system. The PI controller is a widely used feedback control algorithm that combines proportional and integral control actions to achieve desired system performance.

The PI controller is defined by two main parameters: the proportional gain (kp) and the integral gain (ki). These gains determine the controller's response to the error between the plant output and the plant reference signal.

```
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
```
The compensation error (e_comp) is computed using the following formula:

```
e_comp = kp * (plant_output - plant_reference) + ki * integral(e)
```

Where:
- `plant_output` is the current output of the plant model.
- `plant_reference` is the current reference signal for the plant.
- `kp` is the proportional gain.
- `ki` is the integral gain.
- `integral(e)` represents the integral of the error over time.

The integral term in the formula helps the controller to eliminate steady-state errors and provides a response to sustained deviations between the plant output and the reference signal.

The PI controller used in the code is implemented as the `PIController` class, which takes the proportional gain, integral gain, and sampling time (Ts) as parameters during initialization. The `compute()` method of the `PIController` class is used to compute the compensation error

 based on the current plant output and plant reference.

It is important to note that the PI controller in the code operates in a discrete-time domain, where the controller gains and the sampling time are chosen to match the desired system response. The values of `kp` and `ki` can be adjusted according to the specific control requirements and the characteristics of the controlled system.

By applying the PI control algorithm, the code aims to regulate the plant output to track the reference signal accurately and efficiently, contributing to the overall control system performance.



## Results and Plots
In this section, a comparison between the plot generated from the implemented code and the ground truth provided for the assignment was conducted. The purpose was to verify the accuracy and correctness of the implemented code by assessing its agreement with the known ground truth data
| |
| :---: |
|![Figure 1: Response Plot](/assets/response.png)|
|**Figure 1: Response Plot - IIT**|
|![Figure 1: Response Plot](/plot/scope_data_stefano_0_plot.png)|
|**Figure 1: Response Plot PI controller Kp = 10.0, Ki = 1.0 - Stefano**|
The observed similarities between the two plots indicate that the code successfully captures the underlying dynamics and faithfully reproduces the expected behavior.

## To reduce residual oscillations in a control system

__Increase Proportional Gain__ (Kp): The proportional gain determines the response of the controller to the error signal. Increasing the proportional gain can make the controller more aggressive and respond faster to deviations between the plant output and the reference signal. However, be cautious when increasing the gain too much, as it can lead to instability or overshoot.

__Decrease Integral Gain__ (Ki): The integral gain integrates the error over time and helps eliminate steady-state errors. However, a high integral gain can also lead to excessive oscillations. By reducing the integral gain, you can reduce the tendency of the controller to accumulate errors over time, which can help reduce oscillations.

__Use Derivative Control__ (Kd): The derivative control component can provide damping and help reduce oscillations by anticipating changes in the error signal. Adding derivative control can help stabilize the system and reduce overshoot and oscillations. However, be careful when using derivative control, as it can amplify high-frequency noise.

| |
| :---: |
|![Figure 1: Response Plot](/plot/scope_data_stefano_1_plot.png)|
|**Figure 1: Response Plot PI controller Kp = 15.0, Ki = 1.0 - Stefano**|
|![Figure 2: Response Plot](/plot/scope_data_stefano_2_plot.png)|
|**Figure 2: Response Plot PI controller Kp = 15.0, Ki = 0.1 - Stefano**|
Figure 1 and 2 have almost the same trajectories, to decide which is better may be a good starting point to look at the _control_output_.
|![Figure 3: Response Plot](/plot/scope_data_stefano_3_plot.png)|
|**Figure 3: Response Plot PID controller Kp = 10.0, Ki = 0.1, Kd = 1.0 - Stefano**|
Figure 3 has almost no oscillation. 
|![Figure 4: Response Plot](/plot/scope_data_stefano_4_plot.png)|
|**Figure 4: Response Plot PID controller Kp = 10.0, Ki = 0.1, Kd = 0.1 - Stefano**
Figure 4 shows that decreasing the kd value lead to a small oscillation with respect to the previous case.
|![Figure 5: Response Plot](/plot/output.png)|
|**Figure 5: Zoom Response Plot - Stefano**
A zoom of the overall response obtained is provided.


## Conclusion
The report provides an overview of a C++ code implementing a control system using the Reference Model Control (RMC) scheme. It explains the code structure, components used (including a discrete controller and a PI controller), and file handling. The report also discusses the optimization of the digital filter implementation and the application of the Tustin bilinear transformation. The PI controller formula and its role in error compensation are explained. The report concludes with a comparison of the generated plot with the ground truth data to verify the code's accuracy.

## Code updates 
### change #1
As suggested by Dr. Niscolò Genesio "Better using constexpr than const". So the following modification are presented.
```
const double kp[] = {10.0, 15.0, 15.0, 10.0, 15.0}; 
const double ki[] = {1.0 , 1.0 , 0.1 , 0.1 , 0.1 };   
const double kd[] = {0.0 , 0.0 , 0.0 , 1.0 , 0.1}; 
```
The previous code has been replaced by 
```
std::array<double,5> kp = {10.0, 15.0, 15.0, 10.0, 15.0}; 
std::array<double,5> ki = {1.0 , 1.0 , 0.1 , 0.1 , 0.1 };   
std::array<double,5> kd = {0.0 , 0.0 , 0.0 , 1.0 , 0.1};   
```
here are some reason to motivate the change:

__Type Safety__: The second code uses std::array<double, 5> instead of C-style arrays (const double[]) used in the first code. std::array is a container class provided by the C++ Standard Library that encapsulates the array and provides additional functionality, such as bounds checking and size tracking. Using std::array ensures type safety, as it maintains the size information and prevents accidental decay into a pointer, which can happen with C-style arrays.

__Improved Readability__: By using std::array, the code explicitly conveys its intent and provides self-documenting code. It's clear that kp, ki, and kd are arrays of doubles with a fixed size of 5. This improves the code's readability and makes it easier for other developers to understand the code.

__Flexibility and Compatibility__: std::array is a standard container that provides a consistent interface with other containers in the C++ Standard Library. It can be used interchangeably with other containers such as std::vector or std::deque without needing to change the code that operates on the container. This makes the code more flexible and compatible with different container types.

__Safer Array Initialization__: The second code utilizes brace initialization ({...}) syntax, which provides more explicit and safer initialization of the std::array elements. It ensures that the number of elements matches the size of the array and prevents potential errors caused by mismatched sizes.

__Integration with Modern C++ Features: std::array__ is part of the modern C++ Standard Library and supports features introduced in newer C++ versions, such as range-based for loops and auto type deduction. It allows the code to leverage these modern language features, resulting in cleaner and more expressive cod

### change #2
Another modificatio is the following
```
 const double K = 1.0;
 const double Ts = 0.01;
 const double gain = (K * Ts) / 2.0;
```
The previous code has been replaced by 
```
constexpr double K = 1.0;
constexpr double Ts = 0.01;
constexpr double gain = (K * Ts) / 2.0;
```
the reasons to motivate the change are:

__Compile-time Evaluation__: By using the constexpr specifier, the second code allows for compile-time evaluation of the expressions. constexpr variables are evaluated at compile time rather than runtime, providing potential performance benefits. In this case, the expressions (K * Ts) and (K * Ts) / 2.0 will be computed during compilation, eliminating the need for runtime calculations.

__Potential Optimization Opportunities__: constexpr variables can be optimized by the compiler. Since their values are known at compile time, the compiler can perform constant folding and propagate these values directly into the code that uses them. This can potentially lead to improved performance or code optimization opportunities.
### change #3
The following code has been replace with a modern version of it
```
  const int num_values = sizeof(kp) / sizeof(kp[0]);

    for (int i = 0; i < num_values; ++i) {
        // PID controller definition
        PIDController pid(kp[i], ki[i], kd[i], Ts);

```

The following code has some advantage that will be discussed after the code
```
for (auto& kp_value : kp) {
        // PID controller definition
        PIDController pid(kp_value, ki[kp_value], kd[kp_value], Ts);
```

__Simplicity and Readability__: New version uses a range-based for loop, which provides a more concise and expressive way of iterating over the elements of the kp array. It avoids the need for an explicit loop counter and eliminates the indexing operation (kp[i]) used in old version. This makes the code easier to read and understand, as the intent of the loop is more clear.

__Avoiding Potential Errors__: New version eliminates the possibility of errors related to array size calculation (sizeof(kp) / sizeof(kp[0])). In the old version, if the size of the kp array changes, the loop condition and the array size calculation need to be manually updated, which can be error-prone. new code, on the other hand, adapts automatically to any changes in the size of the kp array.

__Flexibility__: The new version allows for more flexible code modifications. If the kp array were to be replaced with a different container type in the future (e.g., std::vector), new code would still work without any modifications. In contrast, the old version assumes the use of a C-style array and would require changes to accommodate a different container type.

__Performance__: In terms of performance, both are likely to have similar efficiency since the loop iterations are the same. However, new code can potentially be more efficient when using certain container types (e.g., std::vector) since it avoids unnecessary index calculations.

