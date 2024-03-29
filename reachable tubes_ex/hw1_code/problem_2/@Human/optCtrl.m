function uOpt = optCtrl(obj, ~, ~, deriv, uMode)
% uOpt = optCtrl(obj, t, y, deriv, uMode)

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% YOUR CODE HERE
% Compute the optimal control
px_derivative = deriv{1};
py_derivative = deriv{2};
l = sqrt(px_derivative^2 + py_derivative^2) - 0.5;

if strcmp(uMode, 'max')
  uOpt = atan2(deriv{1}, deriv{2}); % Compute the optimal control for max case
elseif strcmp(uMode, 'min')
  uOpt = atan2(deriv{1}, deriv{2}); % Compute the optimal control for min case
else
  error('Unknown uMode!')
end


end