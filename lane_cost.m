function cost = lane_cost(u,agent_state,dt,plan_horizon,agent_radius,lane_change)
    if(lane_change == true)
        l = -1;
    else
        l = 1;
    end
    x = zeros(plan_horizon,1);
    y = zeros(plan_horizon,1);
    cost = zeros(plan_horizon,1);
  
    x(1) = agent_state(1);
    y(1) = agent_state(2);
    theta = agent_state(3);
    
    for i = 2:plan_horizon
        theta = theta + u(i-1,2)*dt;
        x(i) = x(i-1) + u(i-1,1)*cos(theta)*dt;
        y(i) = y(i-1) + u(i-1,1)*sin(theta)*dt;
        
    end
    if(lane_change == false)
        cost = abs(sum(l*(y-2*x-10.94))); % rm_y
    else
        cost = abs(sum(l*(y-2*x-28.83))); % lm_y
    end
end