function stop = stopping_criteria(current_state, params, agentName)
if agentName == "A"
    g = [params.ugoalX; params.ugoalY];
    rad = params.ugoalR;
elseif agentName == "B"
    g = [params.dgoalX; params.dgoalY];
    rad = params.dgoalR;
else
    raise Exception("Invalid agentName. Should be A or B.")
end
 stop = (norm(current_state(1:2) - g) - rad) < 0;
end