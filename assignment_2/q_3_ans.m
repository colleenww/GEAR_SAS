close all; clear all; clc

% This is a toy example of CLF-QP
% \dot x1 = v*cos(u),
% \dot x2 = v*sin(u),
% \dot x3 = u,

dt = 0.1;
sim_t = [0:dt:30];
delta = 1e-3;
v = 1;
t = 0;
x0 = [ 0; 0 ; 0 ];
x = nan(3,length(sim_t));
u = nan(1,length(sim_t));
% u = pi/4;
x(:,1) = x0;


data1 = importdata('V_gamma=0_fine.mat');
g = importdata('g_fine.mat');
Deriv = computeGradients(g, data1);

%% Alternative way of defining the dynamics
wRange = [ -1 , 1 ];
dRange = {[0;0;0];[0; 0; 0]};
speed = v;
dCar = DubinsCar([0, 0, 0], wRange, speed, dRange);


%% Main loop for solving the ODE
for i = 1 : length(sim_t)
    deriv = eval_u(g,Deriv,x(:,i));
%     u(i) = dCar.optCtrl(dCar, deriv, 'min');
%     u(i) = sin(t);
    if deriv(3)>0
        u(i) = -1;
    else
        u(i) = 1;
    end
    [ts_temp, xs_temp] = ode45(@(t, s) Dcar(t, s, u(i)), [t t+dt], x(:,i));
    x(:,i+1) = xs_temp(end,:);
    t = t+dt;

end
%%
figure
plot3(x(1,1),x(2,1),x(3,1),'g*');
hold on
plot3(x(1,:),x(2,:),x(3,:),'b');
hold on
plot3(x(1,end),x(2,end),x(3,end),'ro')
grid on
xlabel('x','interpreter','latex');
ylabel('y','interpreter','latex');


figure
plot(x(1,1),x(2,1),'g*');
hold on
plot(x(1,:),x(2,:),'b');
hold on
plot(x(1,end),x(2,end),'ro')
grid on
xlabel('x','interpreter','latex');
ylabel('y','interpreter','latex');

function dydt = Dcar(t,s,u)
    v = 1;
    dydt = [v*cos(s(3));v*sin(s(3));u];
end