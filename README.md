# CarND-Controls-MPC
# Self-Driving Car Engineer Nanodegree Program

# Model Predictive Controller Project

## Project Description

The goal of this project is to develop a nonlinear model predictive controller (NMPC), in order to navigate a track in a Udacity-provided simulator, by sending to it steering and acceleration commands.
The simulator provides via websocket values, including the position of the car, its speed and heading direction, as well as waypoints along a reference trajectory that the car must follow.
The coordinates are provided in a global coordinate system.


## Model Used


The model used in this project is the kinematic bicycle model. 
Inertia, torque and fiction are not considered.
The state vector consists of the following elements:
The state vector of the vehicle is given as follow:
1. x - Vehicle position in forward direction
2. y - Vehicle position in lateral direction
3. psi - Angle of the vehicle (yaw angle)
4. v - Vehicle's speed

And the actuators considered, are the following:
1. delta - Steering angle (radians)
2. a - acceleration



## Update step 
The update step can be seen in the following equations:

* x\_(t+1) = x\_0 - (x\_t + v\_(t) * cos(psi\_(t)) * dt)
* y\_(t+1) = y\_0 - (y\_t + v\_t * sin(psi\_t) * dt)
* psi\_(t+1) = psi\_0 - (psi\_t - v\_t * delta\_t / Lf * dt)
* v\_(t+1) = v\_0 - (v\_t + a\_t * dt)
* cte\_(t+1) = cte\_0 - ((f(x\_t) -y\_t)+(v\_t* sin(epsi\_t)*dt))
* epsi\_(t+1) = epsi\_0 - ((psi\_t - psides\_t) - v\_t * delta\_t / Lf * dt)

(The last equation in the lecture was epsi\_(t+1) = epsi\_0 - ((psi\_t - psides\_t) + v\_t * delta\_t / Lf * dt).
However, changing the last plus to a minus, helped solve issues the car had to follow the trac, and temain inside the limits)

The current state of the car is used as input, in order compute the next state. 
Lf is the distance between the front of vehicle and its center of gravity. 
It was provided during the lectures.

## Timestep Length and Elapsed Duration (N & dt)
The values chosen for N and dt are 10 and 0.1, respectively. 
They were the ones used during the Q&A session, and also seem to work much better than other values selected for dt, like 0.5 or 1.5.
Using these values, the optimizer considers a duration of 1 sec, to determine a trajectory. 

## Polynomial Fitting and MPC Preprocessing
A third degree Polynomial was used to compute the trajectory of the car. 
The waypoints are preprocessed by transforming them to the vehicle's perspective, using the following equations:

for (int i = 0; i < ptsx.size(); i++){


  double shift\_x = ptsx[i] - px;
  
  
  double shift_y = ptsy[i] - py;
  
  
						
  ptsx[i] = shift\_x * cos(0 - psi)-shift\_y * sin(0-psi);
  
  
  ptsy[i] = shift\_x * sin(0 - psi)+shift\_y * cos(0-psi);
  
  
}

#  Model Predictive Control with Latency
Speed was limited to 80mph.
A delay of 100msec was introduced, so the kinematic equations are adjusted accordingly.
Since the latency is equal to the time step, the actuations are applied one time step later.

The cost function parameters were selected similarly to the ones used during the Q&A session.
There were some adjustments made though, which made the car seem to navigate the track more smoothly.

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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
