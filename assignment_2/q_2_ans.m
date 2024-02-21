close all; clear all; clc


dt = 0.1;
sim_t = [0:dt:30]; %simulation time
t = 0;
x0 = [ 2 ];
x = nan(1,length(sim_t));
x(:,1) = x0;


%% Main loop for solving the ODE
for i = 1 : length(sim_t)
    [ts_temp, xs_temp] = ode45(@(t, s) dyn(t, s), [t t+dt], x(i));
    x(i+1) = xs_temp(end);
    t = t+dt;

end
%%
l = length(sim_t);
figure
plot(sim_t,x(1:l),'b');
hold on
plot(sim_t,x0*exp(-sim_t),'k*');
grid on
xlabel('t','interpreter','latex');
ylabel('x','interpreter','latex');
legend('Numerical sol','analytic sol')

function dydt = dyn(t,s) %the dynamic system is a function of t and s
    dydt = -s;
end