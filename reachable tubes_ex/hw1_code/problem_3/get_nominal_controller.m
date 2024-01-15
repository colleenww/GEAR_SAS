function u_nom = get_nominal_controller(xinit,params, agentName)
    % return the mpc based nominal controller
    if agentName == "A"
        goalX = params.ugoalX;
        goalY = params.ugoalY;
        uMax = params.uMax;
        v = params.va;
    elseif agentName == "B"
        goalX = params.dgoalX;
        goalY = params.dgoalY;
        uMax = params.dMax;
        v = params.vb;
    else
        raise Exception("Unrecognized agentName. Should be A or B.")
        return 
    end

    X = mpc(params.H, goalX, goalY, params.dt, uMax, xinit, v);
    u_nom = X(3*(params.H+1)+1,1);
end