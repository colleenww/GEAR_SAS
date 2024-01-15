clear all; close all; clc;
%% Get the params of the simulation environment.
params = get_params();

%% Plot the initial environment.
plot_env([params.xAinit], [params.xBinit], [0], params);

%% TODO: Reachability analysis
% Please complete the template in the BRT_computation.m file for this 
% section to get the safety controller, optimal disturbance, value
% function (called "data"), timestamps ("tau"), grid (g), and value
% function spatial gradients ("derivatives"). 
disp('Pre-computing the safety controller with the BRT...............')
[params.safety_controller, ...
    params.worst_dist, ...
    params.data, ...
    params.tau, ...
    params.g, ...
    params.derivatives] = ...
        BRT_computation(params); % the BRT yeilds the safety controller for free

% NOTE: once your BRT computation works, you can save out the quantites
% above and load them in here instead of re-doing computation. This will
% save you time when iterating on the simulation loop. 

%% Simulate the interaction. 

% Since the BRT is computed in relative state space, we need
% to convert from global to relative state to query into the BRT.
xrel = global_to_rel_state(params.xAinit, params.xBinit);

% Save states and controls for each agent.
traj_A = [params.xAinit];
traj_B = [params.xBinit];
traj_rel = [xrel];
uSafe_bool = [0]; % tracks if safety control was applied by A
uA_traj = [];
uB_traj = [];

timeout = false;
tstep = 0;

% Simulation loop.
while ~stopping_criteria(traj_A(:,end),params,"A") && ~timeout
    
    curr_state_A = traj_A(:,end);
    curr_state_B = traj_B(:,end);
    curr_rel_state = traj_rel(:,end);
    
    % Get the nominal controller: This is given to you!
    uA_nom = get_nominal_controller(curr_state_A, params, "A");
    
    % Switch agent A's control scheme.
    switch params.controller_choice
        case 0 % nominal planner
            chosen_uA = uA_nom;
            uA_traj = [uA_traj; uA_nom]; 
            uSafe_bool = [uSafe_bool; 0];
        case 1 % safety filtered planner
            % Bookkeeping. 
            curr_rel_state_snapped = snap_to_grid(curr_rel_state, params);

            % Get the least restrictive safety filtered controller. 
            [chosen_uA, safety_override] = get_safety_controller(curr_rel_state_snapped, uA_nom, params);
            uA_traj = [uA_traj;uA_nom chosen_uA];
            uSafe_bool = [uSafe_bool; safety_override];
    end

    % Get agent B's control. 
    switch params.disturbance_model
        case 0 % worst-case behavior
            % Bookkeeping if relative state is out of bounds.
            curr_rel_state_snapped = snap_to_grid(curr_rel_state, params);

            % Get optimal worst-case disturbance.
            uB = eval_u(params.g, params.worst_dist, curr_rel_state_snapped, 'nearest')
            chosen_uB = uB;
            uB_traj = [uB_traj; uB];
        case 1 % nominal behavior
            uB = get_nominal_controller(curr_state_B, params, "B");
            chosen_uB = uB;
            uB_traj = [uB_traj; uB];
    end
    
    % Update the next state using the chosen controls. 
    next_state_A = simulate(curr_state_A, chosen_uA, params.va, 0, params.dt);
    next_state_B = simulate(curr_state_B, chosen_uB, params.vb, 0, params.dt);
    next_rel_state = global_to_rel_state(next_state_A, next_state_B);
    
    % Bookkeeping. 
    traj_A = [traj_A, next_state_A];
    traj_B = [traj_B, next_state_B];
    traj_rel = [traj_rel, next_rel_state]; % update the relative state trajectory
    plot_env(traj_A, traj_B, uSafe_bool, params); % plot the env after every step
    pause(0.05)

    tstep = tstep+1;
    if tstep > 500
        timeout = true;
    end
end

%% Plot the control over time.
plot_controller(params.controller_choice, uA_traj, "A");

figure(3)
plot(1:length(uB_traj),uB_traj(:,1), 'LineWidth', 3);