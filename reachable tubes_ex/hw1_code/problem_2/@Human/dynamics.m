function dx = dynamics(obj, ~, x, u, d)
% Dynamics of the Dubins Car
%    \dot{x}_1 = v * cos(u) + d1
%    \dot{x}_2 = v * sin(u) + d2
%   Control: u;
%


if nargin < 5
  d = [0; 0];
end

if iscell(x)
  dx = cell(length(obj.dims), 1);
  
  for i = 1:length(obj.dims)
    dx{i} = dynamics_cell_helper(obj, x, u, d, obj.dims, obj.dims(i));
  end
else
  dx = zeros(obj.nx, 1);
  
  dx(1) = obj.speed * cos(u);
  dx(2) = obj.speed * sin(u);
end
end

function dx = dynamics_cell_helper(obj, x, u, d, dims, dim)

switch dim
  case 1
    dx = obj.speed * cos(u);
  case 2
    dx = obj.speed * sin(u);
  otherwise
    error('Only dimension 1-2 are defined for dynamics of DubinsCar!')
end
end