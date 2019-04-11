# CarND-Controls-PID
A project for the Self-Driving Car Engineer Nanodegree Program, ending May 21st, 2019.

## Running the project
``` bash
# First, download and run the simulator (instructions in further sections)

# Then, build the path planner:
mkdir build
cd build
cmake ..
make

# From project root, run the build path planner:
build/path_planning
```

## Overview
The project required me to implement a PID controller to drive a car in a simulation. An easy one, given that I've done PID tuning in university, and I've seen the lecture videos in previous Udacity courses 3 times already. Below you'll find a brief description of what PID controller is and how I've used the Twiddle algorithm to tune the parameters. A writeup with images will be available on my website soon, at [https://linasko.github.io/portfolio/](https://linasko.github.io/portfolio/).

## PID Control
PID control (Proportional Integral Derivative) - is a process regulation method that, given an error, regulates an output such that over time, subsequent errors are minimized. The generated output is a sum of three components:
* The proportional component is simply the weighted error at the current time step. It regulates how strongly the controller will respond to the value of the error. (e.g. if error is high, steer by a similarly high amount)
* The integral component is a weighted historical measure of the error, computed by summing all the past errors. It helps when we need to correct a steady state error, that always impacts the error by a constant amount. (e.g. one wing of a plane is slightly differently shaped, making the plane always drift to the right a bit)
* The derivative component attempts to react to error's rate of change. It is computed by finding the difference in speeds, divided by the time difference, and multiplied by a weight factor.

I don't know how popular it is, but it's certainly possible to also use coefficients representing higher order derivatives. For example, there could be a component that reacts to acceleration and jerk of the error, in addition to the derivative component here.

To use the PID control, the weights for each of these components needs to be set. The next section will explain how I did it.

## Twiddle
Twiddle is an algorithm that I used to tune the parameters. It attempts to minimize and error by trying out different parameters and loosely following the gradient, leading to the smallest error. Yes, it's very prone to local minima, but for simple PID tuning, it works well.

At every iteration of Twiddle we will be given coefficient values, as well as the error resulting from those. Twiddle will then devise its own coefficients, that will be used to increase or decrease the value of the given coefficients. Those are then returned to compute the new error metric, that is again returned to twiddle. If it was lower, the twiddle coefficient value increases, and if it's higher, the values decreases. It allows twiddle to gain momentum when it's going in the right direction and is successfully minimizing the error, but quickly decrease if it's going the wrong way.

Now, the above is commonly known - the creative step was applying Twiddle to this particular problem. First, it's always important to let the car run a bit without passing the params to twiddle, as it starts off slowly, and we want to help it correct itself over time.

Then, it's important to pass the right error measure to twiddle. Initially I tried passing a multiple of `(carError / carSpeed)` as the error, so that runs that end in the car crashed at rock at some point would produce a high value. This simply resulted in poor results that didn't lead to much. Next, I attempted to grant the car a longer grace period initially, and start applying twiddle only at later stages of the run. This took way too much time, and I found that coefficients learned at the start of the journey would sometimes not apply later. Then, a fairly simple solution came to mind - I'll have a `high_number - run_time` as the error passed to twiddle. This produced very good results, but was very slow, as failure cases were not terminated early. So that's what I solved next - I would reset the run and pass a high error when the car went too far off centre or reached a speed of 0 (got stuck).

Another trick, was that I set the initial twiddle coefficients to a reasonable values of `0.1`.

As with deep learning, good initialization speeds the algorithm up. Lastly, I let the car run on the track, until it either ran for a long time, or the twiddle coefficients got too small.

And - that's it. I let the car run, and maybe 10-15 minutes later it came up with PID coefficients of `0.152734, 0, 0.820703` that let it circle the track with the initially speed, without stepping out of the lines. Of course, there's lots of room for improvement, particularly if using higher speeds, but at this point, I know a decent result can be achieved, know how to do it, and would rather get on to the final project :)

---
# Original README
The information below this point was given to us in the original README and was not modified by me in any way.

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

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
