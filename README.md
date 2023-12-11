Take-home Assignment on Implementing a Controller
=================================================

## 📔 Assignment
You are asked to write the C/C++ code that implements the system shown below.

The system represents the ensemble of a **discrete controller** and a **continuous plant** that has been
discretized using the [Tustin bilinear transform](https://en.wikipedia.org/wiki/Bilinear_transform).

The adopted method resorts to a traditional _Reference Model Control_ scheme, where a PI compensator is
(_Proportional-Integral_) employed to regulate the error between the output of the reference plant and
the actual plant output. 

| |
| :---: |
| ![](/assets/controller_plant.png) |
| The system consisting of the controller and the plant (🔘 click on the image to enlarge) |
| ![](/assets/controller.png) |
| The internal block diagram of the controller (🔘 click on the image to enlarge) |

⚠ All the required parameters are illustrated in the figures.

### Testing data
The goodness of your implementation can be put to test by feeding the code with the input reference
contained in the file [`data/scope_data.tsv`](/data/scope_data.tsv). This latter file also contains the 
expected system outputs you need to compare your outputs against.

Here's the format of the data contained in [`data/scope_data.tsv`](/data/scope_data.tsv):

| time [s] | reference | plant reference | plant output |
| :--- | :--- | :--- | :--- |
| 0 | 0 | 0 | 0 |
| ... | ... | ... | ... |

### Plotting
You can plot data by running the script [`plot/plot.sh`](/plot/plot.sh), which calls Octave underneath.

![](/assets/response.png)

### Coding
Implement the entire system comprising the controller and the discretized plant by providing the corresponding code in the module [`src/main.cpp`](/src/main.cpp).

If you want to add up further modules to the project, just drop the files in the [src](/src) directory. Recognized extensions are: `.cpp`, `.c`, `.cc`, `.h`, `.hpp`.
1. Build and run the code:
   ```console
   cmake -S . -B build
   cmake --build build --target install
   assignment_implement-controller
   ```
1. Compare your outputs against the testing data stored in [`data/scope_data.tsv`](/data/scope_data.tsv).
1. Provide a detailed report in [Markdown](https://guides.github.com/features/mastering-markdown) to support
   and discuss the activities you carried out to achieve your results.

### ✨ Bonus 🏆
How can the residual oscillations in the plant response be further reduced?

