# CarND-Path-Planning-Project

### Overview

The goal of this project is to write a computer program that will enable a virtual self-driving car make path planning decisions to optimally navigate its way around a busy lane highway within a simulated environment.

The car must choose optimal paths based on its current state and surroundings, making decisions that are continually updated in real-time from information received from on-board sensors. In partiular the car must satisfy the following criteria:

  - Navigate the entire course while staying within one of the lanes except when overtaking
  - Drive close to but not exceed the maximum speed limit except when obstructed
  - Avoid collisions with other vehicles
  - Minimise acceleration and jerk to ensure passenger comfort and safety
  - Overtake slower vehicles when it is safe to do so

### Code Layout

There are four main files associated with the project:

  - main.cpp: The central interface to the simulator. Receives live data from the sensors and generates the car's future trajectory
  - car.hpp: The outline of the Car class, which contains: 
    - Functions needed to analyse the sensor information and determine the optimal trajectory based on prevailing conditions and constraints
    - The car's current state variables
    - Static variables such as the lane width
  - car.cpp: The implementations of the functions within the car class
  - helpers.h: Some utility functions for interpreting the sensor information
 
### The Car Class 

An instance of the Car class, called 'ego' is instantiated in the main function on startup and its state variables are initialised. These include: 

- lane: The car's current lane (ranges from 0-2)
- velocity: The speed at which the car is currently travelling
- car_in_frot / car_to_left / car_to_right: Boolean indicators signalling whether other vehicles are to the front, left and right of the car respectively
- car_front_speed: The speed of the car located directly in front of the vehicle


Each time new data is received from the sensor the following functions are called in sequence:

  - update_predictions: Checks for vehicles to the car's front and sides within a specified threshold and measures the speed of the car in front if applicable. These indicators are updated within the car's state
  - generate_successor_trajectories: Generates a series of potential successor trajectories based on the car's current state. Potential trajectories include:
    - speeding up (if travelling at less than the speed limit and unobstructed)
    - keeping speed (if near to speed limit and unobstructed) 
    - slowing down (if obstructed in front)
    - changing lane (if obstructed and safe to do so)
  - get_lowest_cost_trajectory: Finds the optimal future trajectory based on a predefined set of cost functions
  - create_trajectory: Builds a series of points in the lowest cost trajectory, which are then sent back to the simulator as observed behaviour

### Cost Functions

Three cost functions are defined which ensure that the car does not break the rules of the road and completes the course smoothly:

 - Speed Cost: Ensures that the car drives as close as possible to the speed limit but does not breach the speed limit (set at 50 m/h), compelling the car to make effective progress toward its goal. It also influences the car to prefer faster routes over slower ones where available (e.g. overtaking preferred to staying behind a slow vehicle)
 - Acceleration Cost: Stops the car choosing trajectories that change speeds to quickly, e.g. speeding up suddenly to overtake a vehicle. Ensures lane changes are taken smoothly
 - Lane Cost: Presses the car toward favouring the middle lane. This is the optimal lane in which to travel since it provides more options for overtaking and avoids getting 'boxed in' in one of the outer lanes. Also deters changing lanes too often which should be unnecessary. The middle lane should be preferred in a real life highway setting, since one outer lane is usually reserved for fast moving vehicles and the opposite outer lane is used for vehicles merging in and out of the highway

