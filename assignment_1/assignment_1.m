function assignment_1()
% 3. Run Backward Reachable Tube (BRT) with a goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'minVOverTime' <-- Tube (not set)
%     compTraj = true <-- compute optimal trajectory

%% Should we compute the trajectory?
compTraj = true;

%% Grid
grid_min = [-5; -5; -pi]; % Lower corner of computation domain
grid_max = [5; 5; pi];    % Upper corner of computation domain
N = [41; 41; 41];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set
R = 2; % radius 2 circle in the x1-x2 plane
data0 = shapeCylinder(g,3,[0 0 0],R);
% also try shapeRectangleByCorners, shapeSphere, etc.

%% time vector
t0 = 0;
tMax = 5; % stay in the circle for 5 seconds
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% input bounds
speed = 1; % v = 1, constant velocity
wMax = 1;
% do dStep1 here: define a dMax 
dMax = [0, 0, 0];

% control trying to min or max value function?
uMode = 'min'; % goal is to stay within the circle
% do dStep2 here: define a dMode (opposite of uMode)
dMode = 'max';

%% Pack problem parameters

% Define dynamic system
% obj = DubinsCar(x, wMax, speed, dMax)
dCar = DubinsCar([0, 0, 0], wMax, speed, dMax); %do dStep3 here: input dMax when creating your DubinsCar

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
%do dStep4 here: add dMode to schemeData
schemeData.dMode = dMode;

%% additive random noise
%do Step8 here: Add random disturbance (white noise)
%     add the following code:
%     HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];

HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];
% Try other noise coefficients, like:
%    [0.2; 0; 0]; % Noise on X state
%    [0.2,0,0;0,0.2,0;0,0,0.5]; % Independent noise on all states
%    [0.2;0.2;0.5]; % Coupled noise on all states
%    {zeros(size(g.xs{1})); zeros(size(g.xs{1})); (g.xs{1}+g.xs{2})/20}; % State-dependent noise

%% If you have obstacles, compute them here

%% Compute value function

%HJIextraArgs.visualize = true; %show plot
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.valueFunction = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; %set figure number
HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update

% uncomment if you want to see a 2D slice
%HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; %plot x, y
%HJIextraArgs.visualize.plotData.projpt = [0]; %project at theta = 0
%HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D

HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; %plot x, y
HJIextraArgs.visualize.plotData.projpt = {'min'}; %project at theta = 0
HJIextraArgs.visualize.viewAngle = [30,45]; % view 2D
HJIextraArgs.visualize.xTitle = 'x';
HJIextraArgs.visualize.yTitle = 'y';
HJIextraArgs.visualize.zTitle = 'V';

HJIextraArgs.makeVideo = 1;

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
minWith = 'maxVOverTime'; %computeMethod
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, minWith, HJIextraArgs);

%% Compute optimal trajectory from some initial state
if compTraj
  
  %set the initial state
  xinit = [0, 1, 0];
  
  %check if this initial state is in the BRS/BRT
  %value = eval_u(g, data, x)
  value = eval_u(g,data(:,:,:,end),xinit);
  
  if value <= 0 %if initial state is in BRS/BRT
    % find optimal trajectory
    
    dCar.x = xinit; %set initial state of the dubins car

    TrajextraArgs.uMode = uMode; %set if control wants to min or max
    TrajextraArgs.dMode = 'max';
    TrajextraArgs.visualize = true; %show plot
    TrajextraArgs.fig_num = 2; %figure number
    
    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1 0]; 
    
    %flip data time points so we start from the beginning of time
    dataTraj = flip(data,4);
    
    TrajextraArgs.makeVideo = 1;

    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [traj, traj_tau] = ...
      computeOptTraj(g, dataTraj, tau2, dCar, TrajextraArgs);
  
    figure(6)
    clf
    h = visSetIm(g, data(:,:,:,end));
    h.FaceAlpha = .3;
    hold on
    s = scatter3(xinit(1), xinit(2), xinit(3));
    s.SizeData = 70;
    title('The reachable set at the end and x_init')
    hold off
  
    %plot traj
    figure(4)
    plot(traj(1,:), traj(2,:))
    hold on
    xlim([-5 5])
    ylim([-5 5])
    % add the target set to that
    [g2D, data2D] = proj(g, data0, [0 0 1]);
    visSetIm(g2D, data2D, 'green');
    title('2D projection of the trajectory & target set')
    hold off
  else
    error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
  end
end

% figure(5)
% visFuncIm(g, data0, colormap('jet'),R);
% title('Loss Function Visualization');
% xlabel('x1');
% ylabel('x2');

% Set the initial state
% xinit = [0, 0.5, 0];
% 
% % Check if the initial state is in the BRS/BRT
% value = eval_u(g, data(:,:,:,end), xinit);
% 
% if value <= 0
%     % Set initial state of the Dubins car
%     dCar.x = xinit;
% 
%     % Set trajectory plotting options
%     TrajextraArgs.uMode = uMode;
%     TrajextraArgs.dMode = 'max';
%     TrajextraArgs.visualize = true;
%     TrajextraArgs.fig_num = 3; % Adjust figure number
% 
%     % Flip data time points so we start from the beginning of time
%     dataTraj = flip(data, 4);
% 
%     % Compute optimal trajectory
%     [traj, ~] = computeOptTraj(g, dataTraj, tau2, dCar, TrajextraArgs);
% else
%     error('Initial state is not in the BRS/BRT!');
% end

end