# FIRE Rover Simulator

This is a guide to the Rover simulation setup with the capability of simulating the Electromagnetic Interference (EMI) CPV. 

There are two types of simulation:

- Python based simulator - low fidelity with poor accuracy but fast integration strategy
- Matlab based simulator - medium fidelity with accurate integration strategy using ODE45 integrator.

## Python Installation Instructions:

__requires__: Python >= 3.7

To setup the environment, install the [`uv`](https://docs.astral.sh/uv) tool then run the command `uv sync`.

## Execution Instructions:

We are simulating the rover where the rover starts at [0,0] then when the simulation starts, the rover moves forward along x axis until 7 m. That is when the Rover starts a right turn. Completes a 90 degree turn and goes for a further 7 m when the simulation stops. The x axis and y axis are not up to scale. 
In this simulation setup there is a magnet placed at [X,Y]. The [X,Y] co-ordinate of the magnet can be set in Line 146 of the code. Line 145 is the start of an EMI function that simulates the magnetic field of a magnet according to the model described in (https://www.ndss-symposium.org/wp-content/uploads/2023/02/ndss2023_f616_paper.pdf)
The model is simple. The magnet has a range that can be set in line 147 in metres. If the Rover comes within the range, then the magnet adds a gaussian noise to the magnetometer whose maximum magnitude is goverened by the variable maxM (line 149). maxM is essentially proportional to the strength of the magnet. The gaussian noise decreaes as the distance of the Rover increases from the magnet. The noise value decreases proportional to the square of the distance.  

The simulator can simulate several scenarios two of which are described here:

* Scenario 1 - No CPV: Just put the magnet very far off. In line 146 just put the value [-55, -55]. The Rover will take its usual path. 
* Scenario 2 - EMI attach CPV: Put the magnet close by. In line 146 just put the value [8,-2]. The Rover will start going in circles.

## Python

To run a single simulation, use the command `uv run RoverSim.py [args]`. To run the search-based test generation
module, use the command `uv run RoverSimSearch.py`.

## MATLAB

To run the matlab version you need to open up the main.m file in skid_steering_model_main.zip and hit run. 
It can run the same two scenarios. To run no CPV open the open_loop_reference.m file and change line 108 to [-55, -55]
To run the EMI CPV case change line 108 to [8, -2]
