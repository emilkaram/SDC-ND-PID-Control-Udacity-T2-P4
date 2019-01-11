# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program
![](https://github.com/emilkaram/SDC-ND-PID-Control-Udacity-T2-P4/blob/master/img/1.png)
---
## Project Introduction
In this project I implemented a PID controller in C++ to maneuver the vehicle around the track of the simulator

The simulator provided the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

steering angle calaulated by PID control equation:
- Kp * p_error - Kd * d_error - Ki * i_error

where Kp , Kd and Ki are the gains.


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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

 
# Reflection:

P:The proportional (The current) is the difference between the refrence point and the current value. 
This difference is referred to as CTE(Cross Track Error) . 
The proportional gain (Kp) determines the ratio of controller response to the CTE. 
Increasing the proportional gain will increase the reaction of the control system response.
However, if the proportional gain is too large, the system will begin to overshoot.
If Kp is increased further, the overshoot will increase and the system will oscillate and become unstable.
when I increased kp without considering Ki and kd the car start to oscilate and become out of contol
video link to P gain only:
https://github.com/emilkaram/SDC-ND-PID-Control-Udacity-T2-P4/blob/master/results/P.mp4

![](https://github.com/emilkaram/SDC-ND-PID-Control-Udacity-T2-P4/blob/master/img/3.png)


I: The integral (the past) is the sums of errors over time.So if small error will accmulate and cause the integral component to increase slowly.
The integral response will continually increase over time unless the error is zero, so the effect is to drive the Steady-State error to zero. Steady-State error is the final difference between the process variable and set point. A phenomenon called integral windup results when integral action saturates a controller without the controller driving the error signal toward zero.

video link to I gain only:

https://github.com/emilkaram/SDC-ND-PID-Control-Udacity-T2-P4/blob/master/results/I.mp4



D:The derivative is to the rate of change of the process variable. Increasing the derivative parameter will cause the control system to react more strongly to changes in the error term and will increase the speed of the overall control system response. Most practical control systems use very small derivative parameter, because the Derivative Response is highly sensitive to noise in the process variable signal. If the sensor feedback signal is noisy or if the control loop rate is too slow, the derivative response can make the control system unstable

video link to D gain only:

https://github.com/emilkaram/SDC-ND-PID-Control-Udacity-T2-P4/blob/master/results/D.mp4


PID:
![](https://github.com/emilkaram/SDC-ND-PID-Control-Udacity-T2-P4/blob/master/img/2.png)

I tunned the hyperparmeters manulay and the final values are (Kp = 0.2 , Ki= 0.0002 , Kd = 3.2)to control the steering angle
and for the I used throttle = 0.4- fabs(steer_value * 0.5) to control the speed.


video link to PID:

https://github.com/emilkaram/SDC-ND-PID-Control-Udacity-T2-P4/blob/master/results/Pid.mp4


# Conclusion:
After tuning the hyperparmeters P, I, D the vehicle was able to successfully drive a lap around the track.

 

