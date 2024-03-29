function plots = plot_controller(choice, cont_traj, agentName)
    figure(2);
    clf;
    plot(1:length(cont_traj),cont_traj(:,1),'LineWidth', 3);
    if choice > 0
        hold on; 
        plot(1:length(cont_traj),cont_traj(:,2),'LineWidth', 3);
        if choice == 1
            legend('nominal controller','least restrictive controller');
    end
    if choice == 0
        legend('nominal controller');
    end
    xlabel('Timesteps')
    ylabel('Control Value')
    x0=10;
    y0=10;
    width=1000;
    height=300;
    set(gcf,'position',[x0,y0,width,height])
    set(gcf,'color','w');
    title('Agent '+agentName+' Control Profile')
    plots = 0;
end