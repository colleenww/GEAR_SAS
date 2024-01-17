%% YOUR CODE HERE. Hint: look for createGrid() function
% Define the grid for the computation: g
failure_radius = .5;
grid_min = [-1.5-failure_radius, -1.5-failure_radius];
grid_max = [1.5+failure_radius, 1.5+failure_radius];
N = 100;
pdDims = [];

g = createGrid(grid_min, grid_max, N, pdDims)

%% YOUR CODE HERE. Hint: look into shapeSphere() function 
% Define the failure set as encoded by the signed distance function. 
% We use data0 to denote this initial signed distance function from 
% which the value function is computed. 
failure_center = [0, 0];

data0 = shapeSphere(g, failure_center, failure_radius);

%% time vector
t0 = 0;
tMax = 3;
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% input bounds
speed = 1;
thetaMax = pi;

% is the human control trying to min or max value function?
uMode = 'min';

%% Pack problem parameters

% Define dynamic system
human = Human([0, 0], thetaMax, speed);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = human;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;

%% Compute value function
% HJIextraArgs.visualize = true; %show plot
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; %set figure number
HJIextraArgs.visualize.deleteLastPlot = false; %delete previous plot as you update

% uncomment if you want to see a 2D slice
% HJIextraArgs.visualize.plotData.plotDims = [1 1]; % plot x, y
HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D

% The format of this function looks like: 
%   [data, tau, extraOuts] = ...
%           HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'zero', HJIextraArgs);