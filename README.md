# Write-up

In this project I implemented MPC given a kinematic model of the vehicle.

### Errors

The following errors was considered in our MPC implementation:
- Cross track error (cte)
- Angle error (e\psi)

MPC predicts these errors for `N` next steps and tries to minimize these errors.


### Vehicle model

We used a simple kinematic model. The states and actuators were selected as follows:

- State = `[x, y, psi, v]`
- Actuators = `[delta, act.]`   &nbsp;&nbsp;&nbsp;&nbsp;   delta:  steering angle     &nbsp; &nbsp;&nbsp;&nbsp;&nbsp;act.: throttle pedal


The next state is calculated as follows:

```
x' = x  +  v cos(psi) dt
y' = y  +  v sin(psi) dt
psi' = psi  +  v/Lf  delta dt
v' = v + act. dt
```

Now, we can predict the next errors:

```
cte' = cte  +  v sin(e\psi) dt
e\psi' = e\psi +  v/Lf  delta dt
```

### Cost function

We defined the cost function as follows:

```
Cost = w1 . cte ^ 2
     + w2 . e\psi ^ 2 
     + w3 . (v - 70) ^ 2 
     + w4 . delta ^ 2 
     + w5 .act. ^ 2 
     + w6 .(delta_next - delta) ^ 2 
     + w7 .(act_next - act) ^ 2
```

The weights `w1...w7` were set empirically, but they can also be learnt.


### How to set the timestep length and duration

I set `N=8` and `dt=0.07` empirically. Choosing a large period (`N*dt`), for example `N=12`, caused a big error in trajectory  estimation and resulted in large oscillation.

To have a smoother transition, I `w6` and `w7` as follows:
```
w1 = w_cte = 10.0
w2 = w_epsi = 50.0
w3 = w_v = 1.0
w4 = w_thr = 10.0
w5 = w_str = 3000.0
w6 = w_dif_thr = 10.0
w7 = w_dif_str = 3000.0
```

### Preprocessing data

After receiving waypoints, I transferred those points to the vehicle coordinate system. Also, the predicted trajectory were sent to the simulator to overlay on the screen.


### Account for latency

I consider `100 mili sec.` for command propagation latency as follows:
```
double latency = 0.1; // 100 mili sec.

double delta = - steer_value;
double npx = px + v * cos(psi) * latency;
double npy = py + v * sin(psi) * latency;
double npsi = psi + v * (delta / Lf) * latency;
double nv = v + throttle_value * latency;

// convert states to vehicle space
double xd = npx - px;
double yd = npy-  py;
double x = xd * cos(psi) + yd * sin(psi);
double y = yd * cos(psi) - xd * sin(psi);
v = nv;
psi = npsi - psi;
```

# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program



---

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
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `sudo install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.8`. Note Actually the Ipopt package should contain a "ThirdParty" folder where it contains Blas etc. Note you need Sudo! 
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
