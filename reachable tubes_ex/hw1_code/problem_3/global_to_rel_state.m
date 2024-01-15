function [xrel] = global_to_rel_state(xA, xB)
% Convert from global to relative state.
xrel = xB - xA;

% Note: make sure to wrap to pi!
xrel(3) = mod(xrel(3), 2*pi);
xrel(3) = xrel(3) - (2*pi * (xrel(3) > pi));

% Get rotation matrix 
rot_mat = [cos(xA(3)), sin(xA(3));
           -sin(xA(3)), cos(xA(3))];

% Get the initial relative state
xrel(1:2) = rot_mat * xrel(1:2);
