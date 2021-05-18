function goal_pred = predict_goal(u,agent_state,goal_state,dt,plan_horizon)
    x = zeros(plan_horizon,1);
    y = zeros(plan_horizon,1);
  
    x(1) = agent_state(1);
    y(1) = agent_state(2);
    theta = agent_state(3);
    
    for i = 2:plan_horizon
        theta = theta + u(i-1,2)*dt;
        x(i) = x(i-1) + u(i-1,1)*cos(theta)*dt;
        y(i) = y(i-1) + u(i-1,1)*sin(theta)*dt;
        
    end
    goal_pred = [x(plan_horizon),y(plan_horizon),theta];
end