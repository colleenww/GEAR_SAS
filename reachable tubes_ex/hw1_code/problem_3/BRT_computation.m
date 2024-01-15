function [safety_controller, worst_dist, data, tau, g, derivatives] = BRT_computation(params)
% unpack params
uMax = params.uMax;
dMax = params.dMax;
va = params.va;
vb = params.vb;

%% YOUR CODE HERE
% Define the grid for the computation: g
% g = ...

%% YOUR CODE HERE
% Define the failure set: data0
% data0 = ....

% time
t0 = 0;
tMax = 4;
dt = 0.05;
tau = t0:dt:tMax;

% is ego agent (control) trying to min or max value function?
uMode = 'max';
% is opponent (disturbance) trying to min or max value function?
dMode = 'min';

% Define dynamic system
relSys = Air3D([0, 0, 0], uMax, dMax, va, vb);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = relSys;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;
% HJIextraArgs.obstacles = obs;

%% Compute value function
% HJIextraArgs.visualize = true; %show plot
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; %set figure number
HJIextraArgs.visualize.deleteLastPlot = false; %delete previous plot as you update

% uncomment if you want to see a 2D slice
HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; %plot x, y
HJIextraArgs.visualize.plotData.projpt = [0]; %project at theta = 0
HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'zero', HJIextraArgs);
vfunc = data(:,:,:,end); % get value function at final time (assume converged)
derivatives = computeGradients(g, vfunc);

% Extract the safety controller for player A (controller)
safety_controller =  relSys.optCtrl([], vfunc, derivatives, 'max');

% Extract the worst-case behavior of player B (disturbance) 
worst_dist =  relSys.optDstb([], vfunc, derivatives, 'min');

tau = tau2;

