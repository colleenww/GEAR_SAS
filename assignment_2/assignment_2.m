%% Problem 2:
close all; clear all; clc;

% ODE function
ode = @(t, x) -x; 
tspan = [0 5]; % t∈[0,5]
x0 = 2; % initial condition

% solve numerically
[t_num, x_num] = ode45(ode, tspan, x0);

% analytical soln
x_a = x0 * exp(-t_num);

figure(1)
plot(t_num, x_num, '-b', 'LineWidth', 2);
hold on;
plot(t_num, x_a, 'or', 'LineWidth', 2)
xlabel('Time (t)'); ylabel('x(t)'); title('Comparison of Numerical and Analytical Solutions');
legend('Numerical', 'Analytical');
grid on;
hold off;

%% Problem 3
% 3.2
ode = @(t,x) [cos(x(3)); sin(x(3)); 0]; % u(t) = 0
x0 = [0; 0; 0]; % initial position & heading angle
[t_num, x_num] = ode45(ode, tspan, x0);

figure(2)
plot(x_num(:,1), x_num(:,2), '-b', 'LineWidth', 2);
grid on; title('Numerical Solution given u(t) = 0');

% 3.3
ode = @(t,x) [cos(x(3)); sin(x(3)); sin(t)]; % u(t) = sin(t)
x0 = [0; 0; 0]; % initial position & heading angle
[t_num, x_num] = ode45(ode, tspan, x0);

figure(3)
plot(x_num(:,1), x_num(:,2), '-b', 'LineWidth', 2);
grid on; title('Numerical Solution given u(t) = sin(t)');

%% Problem 4
syms x
ode = @(t,x) [cos(x(3)); sin(x(3)); u]; % u(t) = k(x)
% if diff(ode) <= 0
%     u = -1;
%     else if diff(ode) > 0
%         u = 1;
%     end
% end

x0 = [0; 0; 0]; % initial position & heading angle
[t_num, x_num] = ode45(ode, tspan, x0);

figure(4)
plot(x_num(:,1), x_num(:,2), '-b', 'LineWidth', 2);
grid on;

%% Problem 3
sym n
t = -10:100:10
xt = cos(4*pi*t)^2;

figure(1);
subplot(2,2,1)
plot(t, xt)