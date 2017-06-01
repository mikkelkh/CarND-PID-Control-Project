# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Describe the effect each of the P, I, D components had in your implementation.
Below, I have described the meaning of each of the three terms, P, I, and D.
I have implemented the Twiddle algorithm for tuning the parameters, and describe below my observations, as the algorithm converged.

- The P component describes an error proportional to the CTE (cross track error). If the CTE is large, the corresponding P error will be large. In the current project, this will correspond to a large steering angle, positive or negative depending on the direction. With Twiddle, the P term was the first to converge, such that the car stayed on the road. However, at first, this resulted in very unstable behaviour where the car kept overshooting the center trajectory.
- The I term describes an error proportional to the integral of the CTE. It thus depends on all past values of the CTE and is used to account for a potential offset/bias in the steering mechanism of the car. As it appears that there's only a very small bias in the simulator car, this was the last parameter to converge.
- The D term describes an error proportional to the differential of the CTE. It thus depends on the rate of change of the steering angle. It is calculated as the difference between the current and previous steering angles. The D term serves to dampen the wobbling behaviour introduced by the P term. A high P term must so to say be stabilized by a corresponding higher D term. With Twiddle, this was clearly seen, as the D term increased along with the P term.

## Describe how the final hyperparameters were chosen.
As described above, I used the Twiddle algorithm to fit all three hyperparameters.

Ideally, one would let the car traverse the entire track before calculating the total error used in the Twiddle algorithm.
However, since initial parameters did not succeed in having the car stay on the road, and since evaluation time increases linearly with driving time, I used a coarse-to-fine approach:

1. I first limited the number of iterations (callbacks from the simulator) to 200, and initialized all parameters (Kp, Kd, and Ki) with 0. The increments were all set to 0.1.
After 40 iterations, the algorithm converged, with very coarse/rough estimates of the three parameters (Kp=0.771561, Kd=0.199, Ki=0).

2. I then increased the number of iterations to 500 and initialized both parameters and increments to the converged values of the first run. After 120 iterations, the algorithm converged, with refined estimates of the three parameters (Kp=0.485012, Kd=4.25632, Ki=0.000852756).

3. Finally, I increased the number of iterations to 1500, corresponding almost to an entire lap on the track. The execution time of the algorithm increased with a factor of three compared to the second run. After 53 iterations, the final estimates of the three parameters were found to be:
    - Kp = 0.871971
    - Kd = 7.38248
    - Ki = 0.00466666

With these parameters, the car traverses the entire track with no tires leaving the drivable portion of the track surface.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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
