# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Project Basics
In this project, I am using a Proportional-Integral-Derivative Controller, PID in order to drive a simulated car around a virtual track. The project involves implementing the controller primarily for the steering angle of the car as well as tuning coefficients for each PID value in order to calculate a steering angle that keeps the car on the track.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
   
##Simulator. 
You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Rubric points

### Compile
#### Your code should compile.
The code compiles without any errors.No modification done in given setup

### Implementation
#### The PID procedure follows what was taught in the lessons.
Yes,PID implementation is done in the PID.cpp file. The UpdateError() method calculates proportional(p_error), integral(i_error) and derivative(d_error) errors and the TotalError() calculates the total error using the appropriate coefficients(Kp, Ki, Kd).

### Reflection
#### Describe the effect each of the P, I, D components had in your implementation.
The actual implementation of code for a basic PID controller is fairly straightforward. My implementation closely follows what was taught in the lessons.

The 'P' proportional portion of the controller tries to steer the car toward the center line against the cross-track error, cte. If used along, the car overshoots the central line very easily and go out of the road very quickly. However, if the coefficient is set too high for P, the car will oscillate a ton, as the car will constantly overcorrect and overshoot the middle. If the coefficient is too low, the car may react too slowly to curves when the car gets off-center with a higher CTE.

The "I" for integral sums up all previous cte's up to that point to eliminate a possible bias on the controlled system.If used along, it makes the car to go in circles.

The "D" for derivate is the change in CTE from one value to the next. IT helps to counteract the proportional trend to overshoot the center line by smoothing the approach to it.Too high of a coefficient leads to almost constant steering angle changes of large degrees, where although the car will be well-centered it can hardly move. Too low of a D coefficient will lead to the oscillations being too high with more overshooting.

#### Describe how the final hyperparameters were chosen.
The parameters were chosen manually by try and error. I started by using zero as parameters to see if car can drive straight. Then added the Kp and the car start going on following the road but it starts overshooting go out of it. Then added the Kd to try to overcome the overshooting. Once those values were found i tried adjusting them in conjuction with each other. After the car drove the track without going out of it, the parameters increased to minimize the average cross-track error on a single track lap. The final parameters where [P: 0.1, I: 0.005, D: 1.6].
I also tried implementing Twiddle algorithm, which helps finding the optimal parameters. I ended up not using it as the resulting parameters tended to vary every time I ran it, and didn't find improvement in driving behavior compared to using the manually tuned parameters.

### Simulation
#### The vehicle must successfully drive a lap around the track.
Here is a link to short video with the final parameters
https://youtu.be/DSCRHmfZaao

