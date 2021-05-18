function [x,y] = plottraj(agent_state, controls)
    l = size(controls);
    state = agent_state;
    for i =1:l(1)
        state = nonhn_update(state,controls(i,1),controls(i,2),0.1);
        x(i) = state(1);
        y(i) = state(2);
    end
end