# PID Controller
### Description:
  A PID controller uses 3 pieces of information, the current CTE(cross track error) of our vehicle, in this case how far away from the desired trajectory the car is. We use this in our P(proportional) term, for the D(derivative) term we want to use how much the car's CTE has changed so we subtract the previous CTE we had from the current CTE and use  this. Finally the I(integral) term we want to help correct for any biases the car stay to one side of the trajectory, to do so we keep a running sum of CTE's to use in this term. Each term then also has a coefficient which needs to be tuned in order to get the best results, I tuned them using the Twiddle algorithm on a simple problem similar to this one, these were then used for the steering PID
  controller and tweaked a little to get better results, for the speed PID controller I just used the same coefficients.

This code as is will interact with the Simulator found [here](https://github.com/udacity/self-driving-car-sim/releases)
## Dependencies:
##### To build and run as is this project requires:

###### To Run:
  1. [Simulator](https://github.com/udacity/self-driving-car-sim/releases)
  2. [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)

###### To Build:
  3. cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
  4. make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
  5. gcc/g++ >= 5.4
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)

## Running the Code:

The main program can be built and run by running the following commands from the project's top directory(this has already been done):

If no changes made to code just run:
1. Run it: `cd build && ./pid `

Or build with changes and run:

1. Remove old build directory and make new one: `rm -r build && mkdir build && cd build`
2. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
3. Run it: `./pid `

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.
## Code Structure:

`NOTE: Kp,Ki,Kd values for both controllers can probably be improved, but seem to work reliably as is, with a little swerving`
In src the following files:
 1. main.cpp:
      Initializes 2 PID(Proportional Integral Derivative) Controllers with Kp, Ki, and Kd coefficients. Passes CTE(Cross Track Error) regarding position received from simulator to the 1st PID Controller, then requests a steering angle from this PID. Then passes to the 2nd PID Controller the difference between car's current speed and desired speed, and requests a throttle value from this 2nd controller, it then sends these 2 values back to the simulator.
 2. PID.cpp:
 Manages the derivative and integral of CTE,`Init(Kp_,Ki_,Kd_)` sets the values of `Kp`, `Ki`, and `Kd` `UpdateError(cte)` sets `d_error=cte-p_error`(`d_error=`current cte-last cte),`p_error=cte`,and `i_error+=cte`(accumulates all ctes received in `i_error`). Also has method `TotalError()` which performs PID calculation(`-Kp*p_error-Ki*i_error-Kd*d_error`) and returns result.

##### INPUT: values provided by the simulator to the c++ program
["cte"]

["speed"]

["steering_angle"]

##### OUTPUT: values provided by the c++ program to the simulator
["throttle"]

["steering_angle"]
