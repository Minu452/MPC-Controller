# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
This project aims to build a C++ implementation of a Model Predictive  controller to control the steering and throttle actuation's for an autonomous vehicle in the udacity simulation environment. 

---

## Rubric points

### The model :

The kinematic model used in the MPC was composed of a state space made up of

* X position (x)
* Y postion (y)
* Orientation angle (psi)
* Velocity (v)
* Cross track error (cte)
* Orientation angle error (epsi)

The kinematic model assumes the following actuator inputs are input to the model and can be used to evolve the state over time t.

* Steering angle (delta)
* Acceleration (a)

The kinematic model then relates the vehicle state at time t+1 to the vehicle state at time t by,



![dastate_equations](https://github.com/joshwadd/MPC-Controller/blob/master/images/state_equations.png?raw=true)



### Timestep and Elapsed Duration (N & dt)

The number of timesteps and the duration of each timestep make up the prediction horizon T, which the MPC model optimizes the future predictions over. In the MPC,  T and therefre N and dt are hyper paramaters that require tuning. It only makes sense for the prediction horizon to be of the order of a couple of seconds as predicting futher into the future is unhelpful as the enviroment is likly to change. This is espeically true when moving at high speed.



The final hyper parameters chosen for the MPC was N = 10, dt =0.1. This was arrived at as I only considered a prediction horizon of 1 second. For controlling a vechicle at high speeds, frequent acuations are required. 0.1 seconds was found to be helpful as this timestep also happened to be the same as the system latency in the control system. This helpfully allowed taking the latency into consideration in the state model as discussed below.



When considering a very small dt, many timesteps and thus many variables need to be optimized with in each mpc loop. This greatly increases the computational cost resulting the model not being responsive enough, or not fining a adquetely accurate solution in the numerical optimization.



### Polynomial Fitting and MPC Preprocessing

Waypoints for the desired vehicle path are provided to the controller from the planner, these waypoints are in the unity co-ordinate frame. For simplification of the resulting calculations these waypoints are first transformed into the vechicle coordinate system.

This is done by simply using the known current state of the vehicle and transforming it using the triganomic relations below,  so that the first waypoint and current x and y position occures at x =0 y = 0. A third degree polynomial is then fitted to the way points in the vehicle frame.



```c++
 Eigen::VectorXd ptsx_car_frame(ptsx.size());
 Eigen::VectorXd ptsy_car_frame(ptsx.size());

 // Transform the path waypoints from the unity cordinate system,
// into the car cordinate system. (Simpfies the error calculation)
int number_of_points = ptsx.size();
for( int i = 0; i < number_of_points; i++){
     ptsx_car_frame[i] = (ptsx[i] - px) * cos(-psi) - (ptsy[i] - py) * sin(-psi);
     ptsy_car_frame[i] = (ptsx[i] - px) * sin(-psi) + (ptsy[i] - py) * cos(-psi);
}
```



### Model Predictive Control with Latency

Often in real control systems an amount of latency exsists between an actuation signal and the resulting output. To simulate this process a 100 ms latency was added in the simulation between the calculated control signal and sending it to the simlulator. We can then adjust the state equations of the model to account for this delay when computing the control inputs. The approach taken here to time lag the actuations within the constraints calcuations defined by the state equtation by a single time step for steps t >1.



```c++
 if( t > 1){ // to account for a single timestep delay in accuation
            delta0 = vars[delta_start + t - 2];
            a0 = vars[a_start + t - 2];
 }

```

As our timestep is the same duration as the latency, the actuations in the kenematic model should now occur exactly when they would happen in the simulation enviroment.



---

# Dependencies

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
