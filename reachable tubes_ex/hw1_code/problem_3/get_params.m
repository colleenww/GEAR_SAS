function params = get_params()
    %% Pack all params

    % Relative dynamics parameters
    params.nX = 3;  % States - [p^rel_x, p^rel_y, theta^rel]
    params.nU = 1;  % Controls for agent A (controller) - [w]
    params.nD = 1;  % Controls for agent B (disturbance) - [w]
    params.dt = 0.05; % Discretization step
    params.va = 1.5; % Constant speed for agent A - m/s
    params.vb = 1.; % Constant speed for agent B - m/s
    params.uMax = 0.5; % Maximum control for agent A (angular velocity) rad/s
    params.dMax = 1.; % Maximum control for agent B (angular velocity) rad/s

    % Initial state of both agents in global coordinate frame. 
    params.xAinit = [-1.5, -1.5, pi/4]';
    params.xBinit = [1, -1, pi/2.]';

    % Setup environment parameters

    % Agent A's goal params  
    params.ugoalX = 1;
    params.ugoalY = 0.5;
    params.ugoalR = 0.25;

    % Agent B's goal params
    params.dgoalX = -2;
    params.dgoalY = 1;
    params.dgoalR = 0.25; 

    % Planner params. This is an MPC planner. 
    params.H = 10; % receeding control time-horizon
    
    % Set player A's controller choice: to be set by user
    % choice 0 -> nominal planner, 
    %        1 -> least restrictive safety controller
    params.controller_choice = 1;
    
    % Set player B's simulation model: to be set by user
    % choice 0 -> worst-case adversarial action,
    %        1 -> MPC planner towards goal
    params.disturbance_model = 1;

end