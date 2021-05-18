function [new_state] = nonhn_update(state, v, w, dt)
    new_state(3) = state(3) + w*dt;
    new_state(1) = state(1) + v*cos(new_state(3))*dt;
    new_state(2) = state(2) + v*sin(new_state(3))*dt;   
end