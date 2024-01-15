classdef Human < DynSys
  properties
    % Angle bounds
    thetaMax
    
    speed % Constant speed
    
    % Disturbance
    dRange
    
    % Dimensions that are active
    dims
  end
  
  methods
      function obj = Human(x, thetaMax, speed, dRange, dims)
      % obj = Human(x, thetaMax, speed, dMax, dims)
      %     Human model class
      %
      % Dynamics:
      %    \dot{x}_1 = v * cos(theta) + d1
      %    \dot{x}_2 = v * sin(theta) + d2
      %    theta \in [-thetaMax, thetaMax]
      %    d \in [-dMax, dMax]
      %
      % Inputs:
      %   x      - state: [xpos; ypos]
      %   thetaMin   - minimum angle
      %   thetaMax   - maximum angle
      %   v - speed
      %   dMax   - disturbance bounds
      %
      % Output:
      %   obj       - a DubinsCar2D object
      
      if numel(x) ~= obj.nx
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end
      
      if nargin < 2
        thetaMax = [-1 1];
      end
      
      if nargin < 3
        speed = 5;
      end
      
      if nargin < 4
        dRange = {[0;0];[0; 0]};
      end
      
      if nargin < 5
        dims = 1:2;
      end
      
      if numel(thetaMax) <2
          thetaMax = [-thetaMax; thetaMax];
      end
      
      if ~iscell(dRange)
          dRange = {-dRange,dRange};
      end
      
      % Basic properties
      obj.pdim = [find(dims == 1) find(dims == 2)]; % Position dimensions
      obj.nx = length(dims);
      obj.nu = 1;
      obj.nd = 2;
      
      obj.x = x;
      obj.xhist = obj.x;

      obj.thetaMax = thetaMax;
      obj.speed = speed;
      obj.dRange = dRange;
      obj.dims = dims;

    end
    
  end % end methods
end % end classdef
