Interactive Robotics - Spring 2024 - Instructor: Prof. Andrea Bajcsy
Student Code Template - HW 2 
Written by: Andrea Bajcsy (abajcsy@cmu.edu) 
Adapted from:
Prof. Somil Bansal's EE599 @ USC | Written by Kaustav Chakraborty (kaustavc@usc.edu) 
*******************************************************************************************

TASK 1: You need to compute the BRT for this pursuit-evasion game. @Air3D folder contains the relative dynamical system used for BRT computation. 
Specifically, you are required to fill in the files:

1. BRT_computation.m 
   This will require you to define the grid and the obstacle function for 
   the which you want to compute the unsafe set. 

Note: 
1. Please add the helperOC and ToolboxLS libraries to your matlab path for the BRT computation to happen
2. If implemented successfully, you can visualize the growing BRT by running the following line:
     
    BRT_computation(get_params())

TASK 2: Implement the least restrictive safety filter that will take agent A
from the start to their goal without colliding into agent B. 
Specifially, you are required to fill in the file:

    1. get_safety_controller.m
    This will require you to complete the function 
    "get_safety_controller(current_state, u_nom, params)"
    which also takes in the nominal controller "u_nom", the "curent_state" of the robot
    and the packed parameter struct "param", to return the least restrictive safety 
    controller at that state. 

The "main.m" file is the main driver for the code.

The organization is as follows:
    - parameter declaration
    - BRT computation
    - robot trajectory initialization 
    - while the robot does not reach the goal do the following,
        - get the nominal controller 
        - filter the controller according to the controller choice
        - get agent B's control action according to their simulation model
        - simulate each agent's next state add it to the trajectory
        - plot the environment with the trajectory uptil current time
    - plot agent A's control profile

Parameters: The parameter for the code can be set in the "get_params.m" file. This file 
    is called at the beginning of the main file and returns the struct called params. Please
    see Note 1 (below) for usage.

Chosing controllers: 
     The nominal planner is already provied to you. To generate the nominal trajectory,
     set "params.controller_choice = 0;" in the "get_params.m" file then execute
     the "main.m" file. This will display the environment with the evolving trajectory
     under the nominal controller. The nominal controller profile will then be plotted at the end.
	
     To chose the least restrictive safety filtering, set "params.controller_choice = 1;" in 
     the "get_params.m" file then execute the "main.m" file. Make sure that you have imple-
     mented the "get_safety_controller.m" file before doing so.

Choosing agent B models:
     To simulate agent B under a worst-case adversarial model
     set "params.disturbance_model = 0;" in the "get_params.m" file then execute
     the "main.m" file. This will display the environment with the evolving trajectory
     under an adversarial interaction. 

     To simulate agent B under a goal-driven planning model, 
     set "params.disturbance_model = 1;" in the "get_params.m" file then execute
     the "main.m" file. This will display the environment with the evolving trajectory
     under a goal-driven interaction. 

Additional files (no need for modification):

    * dynConst.m - defines the nonlinear contraints for the dynamics
    * get_nominal_controller.m - returns the nominal controller at a given state 
    * mpc.m - returns the MPC solution for the optimal control problem of reaching the goal
    * plot_env.m - plots the environment - obstacles, goal, states, trajectory, etc.
    * simulate.m - simulates the next state given the current state, controls, disturbance, 
                    noise, step time
    * simulate_trajectory.m - simulates the MPC trajectory by taking the optimization solution 
                   and initial state
    * stopping_criteria.m - takes the current state and goal conditions and returns true 
                    if the state has reached the goal else returns false
    * plot_controller.m - plots the choice of the controller over the nominal controller,
                    over the entire trajectory.
    * global_to_rel_state.m - converts a state vector from global frame to agent A's local coordinate frame.
    * snap_to_grid.m - for any relative state that is out of the compute grid bounds, snaps to the nearest state.

NOTE: 
    1. You are allowed to add additional parameters to the param struct in the 
    "get_params.m" file. As an example, if you want to add the parameter "time", 
    you can do so by "param.time = 0".

    2. We recommend keeping the function signatures same, as it will help us 
    uniformly evaluate your work.

    3. In order to visualize the environment in general you can run 
    
        "plot_env([], [], [], get_params())"

    4. Saving plots is not enabled by default, so you have to save them manually from the
    popup windows or write appropriate code.

    5. You may add additional files to your solution, eg. compuation of a look up table with precomputed 
    safety controllers, learned neural networks, etc. But your code must be reproducible under any 
    general setup.
