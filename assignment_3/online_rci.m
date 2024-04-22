close all; 
clear; 
clc

%%
% Problem setup
dt = 0.01;
sim_t = [0:dt:40];
% delta = 1e-3;
v = 1;
gamma = 0.1;
cost = '2norm';

switch cost
    case '2norm'
%         data1 = importdata('V_2norm.mat');
        data1 = importdata('V_gamma=0_fine.mat');
%         data1 = importdata('Gear_V_0.mat');
%     case 'infnorm'
% %         data1 = importdata('V_infnorm.mat');
%         data1 = importdata('V_infnorm_bad.mat');
%     case 'qc'
% %         data1 = importdata('V_Quacost.mat');
%         data1 = importdata('V01_Quacost_bad.mat');
end

% data_min = min(data1,[],'all');
% data1 = data1 - data_min;

% g = importdata('g.mat');
g = importdata('g_fine.mat');

% g = importdata('Gear_g_0.mat');

Deriv = computeGradients(g, V_0);


wRange = [ -pi/2 , pi/2 ];
dRange = {[-0.1; -0.1;0];[0.1; 0.1; 0]};
speed = v;
dCar = DubinsCar([0, 0, 0], wRange, speed, dRange);

%%
t = 0;
% j = 0;
% while j ~= 1
%     x00 = randi([-3,3],3,1);
%     V00 = eval_u(g,data1,x00);
%     if V00 >= 0.5
%         x0 = x00;
%         j = 1;
%     end
% end
    
x0 = [  2 , 2 , 0 ];
x = nan(3,length(sim_t));
u = nan(1,length(sim_t));
u_delta = nan(2,length(sim_t));
x(:,1) = x0;



%% HJ opt control
ctrl = 'opt';

H_delta = [ 0 , 0 ; 0 , 1];
f_delta = [0,0];
lb_delta = [-pi;0];
ub_delta = [pi;inf];

H = 1;
f = 0;
lb = -pi;
ub = pi;
options = optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','iter');

for i = 1 : length(sim_t)
    V(i) = eval_u(g,data1,x(:,i));
    deriv = eval_u(g,Deriv,x(:,i));
    if deriv(1)>0
        d(1,i) = 0.1;
    else
        d(1,i) = -0.1;
    end
    if deriv(2)>0
        d(2,i) = 0.1;
    else
        d(2,i) = -0.1;
    end
    switch ctrl
        case 'opt'
            if deriv(3)>0
                u(i) = -pi/2;
            else
                u(i) = pi/2;
            end
        case 'qp'
            LgV = deriv(3);
            LfV = deriv(1)*(v*cos(x(3,i))+d(1,i)) + ...
                deriv(2)*(v*sin(x(3,i))+d(2,i));

            A_delta = [ LgV , -1 ; 0 , -1 ];
            b_delta = [ -LfV-gamma*V(i) ; 0];
            [u_delta(:,i),~,~] = quadprog(H_delta,f_delta,A_delta,b_delta,[],[],lb_delta,ub_delta);

            A = LgV;
            b = -LfV - gamma*V(i)+u_delta(2,i); % The 0.01 here accounts for the
            %                                               convergence threshold of
            %                                               CLVF
            [u(i),~,~] = quadprog(H,f,A,b,[],[],lb,ub);
    end
    %     u(i) = dCar.optCtrl(dCar,[],[] ,deriv, 'min');
    %     [ts_temp, xs_temp] = ode113(@(t, s) dCar.dynamics(t, s, u(i)), [t t+dt], x(:,i));
    [ts_temp, xs_temp] = ode45(@(t, s) Dcar(t, s, u(i),d), [t t+dt], x(:,i));
    x(:,i+1) = xs_temp(end,:);
    if x(3,i+1) >= pi
        x(3,i+1) = x(3,i+1) -2*pi;
    elseif x(3,i+1) <= -pi
        x(3,i+1) = x(3,i+1) + 2*pi;
    end
    t = t+dt
end


%% QP control: setup with slack variable
% H_delta = [ 0 , 0 ; 0 , 1];
% f_delta = [0,0];
% lb_delta = [-pi;0];
% ub_delta = [pi;inf];
% 
% H = 1;
% f = 0;
% lb = -pi;
% ub = pi;
% gamma = 0;
% options = optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','iter');
% 
% for i = 1 : length(sim_t)
%     V(i) = eval_u(g,data1,x(:,i));
%     deriv = eval_u(g,Deriv,x(:,i));
% 
% 
%     LgV = deriv(3);
%     LfV = deriv(1)*v*cos(x(3,i)) + deriv(2)*v*sin(x(3,i));
% 
%     A_delta = [ LgV , -1 ; 0 , -1 ];
%     b_delta = [ -LfV-gamma*V(i) ; 0];
%     [u_delta(:,i),~,~] = quadprog(H_delta,f_delta,A_delta,b_delta,[],[],lb_delta,ub_delta);
% 
%     A = LgV;
%     b = -LfV - gamma*V(i)+u_delta(2,i); % The 0.01 here accounts for the
%     %                                               convergence threshold of
%     %                                               CLVF
%     [u(i),~,~] = quadprog(H,f,A,b,[],[],lb,ub);
%     %
%     [ts_temp, xs_temp] = ode113(@(t, s) Dcar(t, s, u(i),d), [t t+dt], x(:,i));
% 
%     x(:,i+1) = xs_temp1(end,:);
%     if x(3,i+1) >= pi
%         x(3,i+1) = x(3,i+1) -2*pi;
%     elseif x(3,i+1) <= -pi
%         x(3,i+1) = x(3,i+1) + 2*pi;
%     end
%     t = t+dt;
% 
% end
% 

%% Figures 
l = length(V);
figure
plot3(x(1,:),x(2,:),x(3,:));
view(40,25)
hold on
visSetIm(g,data1,'c',0.02)

xlabel('x','interpreter','latex');
ylabel('y','interpreter','latex');
zlabel('$\theta$','interpreter','latex');

% 
% figure
% subplot(3,1,1)
% plot(sim_t(1:l),u(1:l))
% grid on
% xlabel('time')
% ylabel('control')
% 
% subplot(3,1,2)
% plot(sim_t(1:l),u_delta(2,1:l))
% grid on
% xlabel('time')
% ylabel('violation')
% 
% subplot(3,1,3)
% plot(sim_t(1:l),V(1:l))
% grid on
% xlabel('time')
% ylabel('Value')
% 
figure
set(gcf,'unit','normalized','position',[0.2,0.2,0.64,0.4]);

subplot(1,2,1)
plot(sim_t(1:l),V(1:l))
grid on
xlabel('t','interpreter','latex')
ylabel('V','interpreter','latex')

subplot(1,2,2)
plot(sim_t(1:l),u(1:l))
grid on
xlabel('t','interpreter','latex')
ylabel('u','interpreter','latex')
% 
%%

%% Videos
% MakeVideo = 0;
% [g1D, data1D] = proj(g,data1,[0 0 1],'min');
% % Traj on value function
% figure
% % visSetIm(g,data1,'c',0.01)
% visFuncIm(g1D,data1D,'c',0.6)
% view(40,25)
% set(gcf,'unit','normalized','position',[0.1,0.1,0.8,0.8]);
% hold on
% h_trajV = animatedline('Marker','o');
% 
% 
% xlabel('x1')
% ylabel('x2')
% zlabel('\theta')
% title('value function')
% if MakeVideo==1
%     v = VideoWriter('DubinsQP','MPEG-4');
%     v.FrameRate = 30;
%     open(v);
% end
% 
% for i = 1: length(V) - 1
%     addpoints(h_trajV,x(1,i),x(2,i),V(i));
%     drawnow
%     if MakeVideo == 1
%         frame = getframe(gcf);
%         writeVideo(v,frame);
%     end
% end 
% if MakeVideo == 1
%     close(v);
% end
% 
% 

%%
function dydt = Dcar(t,s,u,d)
    v = 1;
    dydt = [v*cos(s(3))+d(1);v*sin(s(3))+d(2);u];
end