# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Autonomous Vehicle Steering Control Project

Implement a PID controller for steering and throtle control of a vehicle in a simulator to successfully autonomously navigate itself around any track while maintaining lane centering approach, for this project a simple PID controller has been implemented for steering wheel and throttle action based on cross track error.

### Project Steps
1. Implement a Basic PID controller Lib.
2. Tune PID controller for Steering angle predection based on cross track error.

## PID Controller 

A proportional–integral–derivative controller (PID controller or three term controller) is a control loop feedback mechanism widely used in industrial control systems and a variety of other applications requiring continuously modulated control. A PID controller continuously calculates an error value e(t) as the difference between a desired setpoint and a measured process variable and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively) which give their name to the controller.

### Proportional (P) Term Effect:

The main infulancer in the controller terms is the P term as it directly generating a counter action to the cross track error to steer the vehicle back to the lane center. this controller has a side effect that while increasing the gain the vehicle tend to have oscilations around the lane center that can lead to unstable behaviour due to overshoot in switching from side to another of the road.

### Integral (I) Term Effect:

The Integral term play a role in removing the steady state error due to error in car models such as a mechanicla drift in wheels or steering actuator. also has the same side effect of P controller as inceasing this gain can tend to overshooting set point, another side effect widely known is the integral windup effect that as its acutally an error accumelator it tends to saturate its outputs.

### Derivative (D) Term Effect:

The derivative controller take  a counter measurements to the aggrission of P controller as it monitor the change in error between current state and desired one. and act as a reducing factor to overcome possible overshooting this set points.

## PID controller Gains Tunning: 

Manual tunning wass used for finding the best gains for the controller based on the following steps

1. seting all coefficients to zero.
2. increasing Kp gain untill noticing a sustainable oscilation.
3. increasing Kd gain to over dump these oscilations.
4. increasing Ki gain incase of any steady state error due to drift in the car wheels.

Finally Integral part of controller was neglected. and the final gains for the steering PD controller are:
* Kp = 0.12
* Ki = 0
* Kd = 4.0

More soficticated tuning algorithms can be used such as ( Gradients decent, Niclos-ziggler and Genetic Algorithms). but due to insuffiecnt time, manual tuning was enough for this simple case

As for throttle control a P controller was developed only to reduce max. speed in case of sharp curved road as a heleper control to the pid control 

Final Output video of the tunned Controller can be found on [youtube](https://youtu.be/k5SKKW6S-xk)


P only controller effect:
![](./P_controller_effect.gif)

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

