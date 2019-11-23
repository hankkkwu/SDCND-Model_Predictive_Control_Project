# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Result video

[![](https://i.ytimg.com/vi/7U1u-J4k2Cc/hqdefault.jpg?sqp=-oaymwEZCNACELwBSFXyq4qpAwsIARUAAIhCGAFwAQ==&rs=AOn4CLB9yM3BZcFwkfUGl8WQannQU8ndSw)](https://youtu.be/7U1u-J4k2Cc)

## Model predictive Control
Model predictive control (MPC) is an advanced method of process control that is used to control a process while satisfying a set of constraints. In this case it reframes the task of following a trajectory as an optimization problem.

Model predictive control involves simulating different actuator inputs, predicting the result trajectory and selecting that trajectory with a minimum cost. Once we found the lowest cost trajectory, we implement the first set of actuation commands, and throw away the rest of the trajectory we calculated, then we use the new state and use that to calculate a new trajectory.


## MPC algorithm
* Define T(prediction horizon), by choosing N(number of timesteps) and dt(timestep duration).

  The relation of them is : **`T = N * dt`**

  Theoretically, T should be as large as possible, while dt should be as small as possible. But in the case of driving, if T is too large , the environment might change enough that it won't make sense to predict any further into the future. In the case of driving in simulator, setting **T = 1** second is reasonable. For dt and N, a large velue of dt would result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. After some experiment, I found that **dt = 0.05** and **N = 20** works pretty well.

* Define the vehicle model.
  In this projecet I use the kinematic model, the state of vehicle can be defined as **[x, y, psi, v, cte, epsi]**, where x, y are the current position of the vehicle, psi is the heading direction, v is the velocity, cte is the cross track error and epsi is the orientation error.  

* Define the vehicle constraints.
  In a real vehicle, actuators are limited by the design of the vehicle in fundamental physics, such as steering angle and acceleration, in this project we set the **steering angle = [-25°, 25°]** and **acceleration = [-1, 1]** (-1: full brake, 1: full acceleration).

* Define a cost function.
  Cost function can be the errors we would like to minimize. We already captured 2 errors in our state(cte and epsi). But it is not limited to the state, we could also include the control inputs(steering angle and acceleration), such as the magnitude of the inputs and the change rate of the inputs.

* Finally, begin the state feedback loop:
  **vehicle current state  ->  MPC solver  ->  vehicle control inputs**


## Latency problem
In a real vehicle, an actuation command won't execute instantly, there will be a delay as the command propagate through the system, this problem is called **latency**, a realistic delay might be on the order of 100 milliseconds.
In this project we set the latency to 100ms. To deal with this problem, I don't use the current state, instead I use the state that is 100ms in the future.



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

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
