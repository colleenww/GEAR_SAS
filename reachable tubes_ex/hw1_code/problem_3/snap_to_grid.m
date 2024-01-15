function [snapped_state] = snap_to_grid(state, params)
state_up = [params.g.axis(2), params.g.axis(4), params.g.axis(6)]';
state_lo = [params.g.axis(1), params.g.axis(3), params.g.axis(5)]';
snapped_state = min(state, state_up);
snapped_state = max(snapped_state, state_lo);