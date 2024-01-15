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
if strcmp(uMode, 'max')
  %uOpt = % Compute the optimal control for max case
elseif strcmp(uMode, 'min')
  % uOpt = % Compute the optimal control for min case 
else
  error('Unknown uMode!')
end


end