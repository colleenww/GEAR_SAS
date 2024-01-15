function [u_filtered, safety_override]= get_safety_controller(x_curr,u_nom,params)  
    
    %% YOUR CODE HERE
    % Code in the least restrictive safety filter that returns the safe
    % controller if the nominal controller leads the next state into an
    % unsafe region
    if (% safety condition)  % the system is almost doomed to enter failure set!
        u_filtered = eval_u(params.g, params.safety_controller, x_curr, 'nearest'); % get safety controller
    else
        u_filtered = u_nom;
    end

end