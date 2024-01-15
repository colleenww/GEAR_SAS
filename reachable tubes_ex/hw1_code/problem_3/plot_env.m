function plots = plot_env(trajA, trajB, uSafe_bool, params)
figure(1);
clf;
xlim([-3, 3]);
ylim([-3, 3]);
x0=10;
y0=10;
width=1000;
height=1000;
set(gcf,'position',[x0,y0,width,height])
set(gcf,'color','w');

hold on

% Setup colors for agent A and agent B
acolor = "#ff6c03";
agcolor = "#ffd2b3";
bcolor = "#7a00f5";
bgcolor = "#e2b3ff"; 

% Scale quiver arrow
arrow_scale = 0.5;

% Kept track of safety activation, find indicies
xAsafe_active = trajA(:, uSafe_bool > 0);
xBsafe_active = trajB(:, uSafe_bool > 0);

% % Plot safety bubble. 
% if isfield(params, 'data')
%     valfunc = params.data(:,:,:,end);
%     xrel_curr = global_to_rel_state(trajA(:,end), trajB(:,end));
%     [gPlot, dataPlot] = proj(params.g, valfunc, [0,0,1], xrel_curr(3));
%     [~, h] = contour(trajA(1,end) + gPlot.xs{1}, ...
%                     trajA(2,end) + gPlot.xs{2}, ...
%                     dataPlot, [0, 0], 'color', acolor);
% end

% Create goal point for agent A
viscircles([params.ugoalX, params.ugoalY], [params.ugoalR], 'color', acolor);
rectangle('Position',[params.ugoalX-params.ugoalR params.ugoalY-params.ugoalR ...
    2*params.ugoalR 2*params.ugoalR],...
    'EdgeColor',acolor,'FaceColor',agcolor,'Curvature',[1 1], 'LineWidth', 3);

% Put text 
text(params.ugoalX,params.ugoalY+0.5,'Goal A','FontSize',16)

% Create goal point for agent B
if params.disturbance_model == 1
    viscircles([params.dgoalX, params.dgoalY], [params.dgoalR], 'color', bcolor);
    rectangle('Position',[params.dgoalX-params.dgoalR params.dgoalY-params.dgoalR ...
        2*params.dgoalR 2*params.dgoalR],...
        'EdgeColor',bcolor,'FaceColor',bgcolor,'Curvature',[1 1], 'LineWidth', 3);
    
    % Put text 
    text(params.dgoalX,params.dgoalY+0.5,'Goal B','FontSize',16)
end

if ~isempty(trajA)
    % initial state
    xcurr = trajA(:,end);
    % Plot xy over time
    plot(trajA(1, :), trajA(2, :), 'color', '#a3a3a3', 'LineWidth', 2);
    if nargin >= 4
        plot(xAsafe_active(1,:), xAsafe_active(2,:), 'color', 'r', 'LineWidth', 2);
    end
    % Plot footprint
    viscircles(xcurr(1:2,:)', [0.25], 'color', acolor, 'LineStyle', ":");
    
    % Plot current state
    viscircles(xcurr(1:2,:)', [0.1], 'color', acolor);
    quiver(xcurr(1),xcurr(2), arrow_scale*cos(xcurr(3)), arrow_scale*sin(xcurr(3)), 'Color', acolor, 'LineWidth', 2);
    
    % Put text 
    text(xcurr(1)-0.2, xcurr(2)-0.4, 'Agent A','FontSize', 11)
end

if ~isempty(trajB)
    % initial state
    xcurr = trajB(:,end);
    
    % Plot xy history over time
    plot(trajB(1, :), trajB(2, :), 'color', '#a3a3a3', 'LineWidth', 2);
    if nargin >= 4
    plot(xBsafe_active(1,:), xBsafe_active(2,:), 'color', 'r', 'LineWidth', 2);
    end

    % Plot current state
    viscircles(xcurr(1:2,:)', [0.1], 'color', bcolor);
    quiver(xcurr(1),xcurr(2), arrow_scale*cos(xcurr(3)), arrow_scale*sin(xcurr(3)), 'Color', bcolor, 'LineWidth', 2);
    
    % Put text 
    text(xcurr(1)-0.2, xcurr(2)-0.4, 'Agent B','FontSize', 11)
end
axis square;
box on

plots = 0;

