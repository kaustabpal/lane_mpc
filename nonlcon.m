function [c, ceq] = nonlcon(controls,plan_horizon,agent_state,agent_radius,obstacles,dt)
    n = size(obstacles);
    c = [];
    ceq = [];
    if(n ==0)
        c = [];
        ceq = [];
    else
        for j=1:n(2)
            radi_sum = (agent_radius + obstacles(j).agent_radius+0.1)*ones(plan_horizon,1);
            agnt_obs_dist = zeros(plan_horizon,1);

            new_state = agent_state;
            obs = obstacles(j).current_state;
            
            for i=1:plan_horizon
              new_state = nonhn_update(new_state, controls(i,1), controls(i,2), dt);
              obs = nonhn_update(obs, obstacles(j).v0(i),obstacles(j).w0(i),obstacles(j).dt);
              agnt_obs_dist(i,1) = norm(obs-new_state);      
            end
%             temp = radi_sum - agnt_obs_dist;
            c = [c; radi_sum - agnt_obs_dist];
        end
        new_state = agent_state;
        for i=1:plan_horizon % right lane r_y
          new_state = nonhn_update(new_state, controls(i,1), controls(i,2), dt);
          c = [c; new_state(2)-2*new_state(1)-37.77+agent_radius];
        end 
        new_state = agent_state;
        for i=1:plan_horizon % left lane l_y
          new_state = nonhn_update(new_state, controls(i,1), controls(i,2), dt);
          c = [c; -(new_state(2)-2*new_state(1)+2-agent_radius)];
        end
        ceq = [];
    end
end