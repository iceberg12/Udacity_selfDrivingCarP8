# Udacity Self-Driving Car Engineer Nanodegree
## Kidnapped Vehicle Project

[overview]: ./images/overview.png
[particles]: ./images/particles.png
[nn_pros_cons]: ./images/nn_pros_cons.png

---

## Project Introduction
My robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project I will implement a 2 dimensional particle filter in C++. My particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data. 
- http://robots.stanford.edu/papers/thrun.pf-in-robotics-uai02.pdf

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Command line to run the codes:

./clean.sh
./build.sh
./run.sh

### The Map
Map data provided by 3D Mapping Solutions GmbH. `map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### Performance Measurement

1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.
2. **Performance**: your particle filter should complete execution within the time of 100 seconds.

If everything worked you should see something like the following output at the end:

```
Success! Your particle filter passed!
```
## Logics

The iterative steps to use the particle filter algorithm to track the car position within a global map from a noisy GPS position are described as followed

![alt text][overview]

1. Initialize the car position with noisy position data from the GPS. If already initialized, predict the vehicle's next state from previous (noiseless control) data.
2. Receive landmarks observation data from the simulator.
3. Update the particle weights and resample particles.
4. Calculate and output the average weighted error of the particle filter over all time steps so far.

Let's look at Steps 1 and 3 in details.

In Step 1, beside initializing the car position, we also need to initialize the particles around the GPS-identified position with a predefined Gaussian distribution for x, y positions and orientation. Once initialized, we predict the next position using time, velocity and yaw rate. Remember to check if the yaw rate is too small to avoid Number Overflow.

For Step 3, let's look at how particles, acting as satellites around the car, evolves to estimate the locations of surrounding landmarks.

![alt text][particles]

First, in order to relate particles to the global map, one needs to transform the particle positions observed by the car into particle positions in the global map. This can be done with coordinate transformation and rotation. Second, we want to know which landmark a particular particle is observing by finding the shortest distance from all landmarks to the particle. Certainly, if we know the sensor range of our car, we can narrow down the search to only localized landmarks near the car. Third, we want to ask ourselves, how much we can trust each particle. The weightage is done by comparing how accuracy (how close) each particle can estimate its landmarks.

In the above approach, we assume that we use nearest neighbor seach for landmarks. However, this method can have procs and cons, detailed in the following table. You can use the particle map above to make sense about these pros and cons.

| Pros                          | Cons          |
| -------------                 |:-------------:|
| Easy to understand            | Not robust to high density of measurements or map landmarks   |
| Easy to implement             | Not robust to sensor noise                                    |
| Work well in many situation   | Not robust in position estimates                              |
|                               | Inefficient to calculate                                      |
|                               | Does not take different sensor uncertainties into account     |


## Implementation
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

You can find the inputs to the particle filter in the `data` directory, which is the global MAP data. Other data source like car control and car observation of particles are given by the simulator.




