classdef Mpc_agent < handle
    properties
        id % agent id
        init_state % starting state of the agent
        goal_state % goal state of the agent
        current_state % current state of the agent
        v0 % Initial control velocities
        w0 % Initial angular velocities
        vp % last velocity
        wp % last angular velocity
        plan_horizon % plan for 50 timesteps
        update_horizon % update controls after 20 timesteps. (20% of plan_horizon)
        agent_radius % radius of the agent
        dt
        v_list
        w_list
        obstacles % list of obstacle agents
        x_traj
        y_traj
        lane_change % flag to change lane
    end
    
    methods
        function obj = Mpc_agent(id,init_state,goal_state,v0,w0,plan_horizon,update_horizon)
            obj.id = id;
            obj.init_state = init_state; % starting state of the agent
            obj.goal_state = goal_state; % goal state of the agent
            obj.current_state = obj.init_state; % current state of the agent
            obj.v0 = v0; % Initial control velocities
            obj.w0 = w0; % Initial angular velocities
            obj.vp = 0; % last velocity
            obj.wp = 0; % last angular velocity
            obj.plan_horizon = plan_horizon; % plan for 50 timesteps
            obj.update_horizon = update_horizon; % update controls after 20 timesteps. (20% of plan_horizon)
            obj.agent_radius = 1; % radius of the agent
            obj.dt = 0.1;     
            obj.v_list = [];
            obj.w_list = [];
            obj.lane_change = false;
        end
       
        function [x,y] = plottraj(obj)
            state = obj.current_state;
            for i =1:obj.plan_horizon
                state = nonhn_update(state,obj.v0(i),obj.w0(i),obj.dt);
                x(i) = state(1);
                y(i) = state(2);
            end
        end
        
        function predict_controls(obj)
            avoid_collision = true;
            w1 = 1;   % predict goal - actual goal
            w2 = 20;  % difference between velocities
            w3 = 20;  % difference between angular velocities
            w4 = 500; % first control velocity must be same as last velocity
            w5 = 2;  % difference betwen final vel and last control vel
            w6 = 0.1;
            
            cost = @(u) ...
                  w1*(norm(predict_goal(u,obj.current_state,obj.goal_state,obj.dt,obj.plan_horizon)-obj.goal_state)) ...
                + w2*(norm(diff(u(:,1)))) ...
                + w3*(norm(diff(u(:,2)))) ...
                + w4*(norm(u(1,1)-obj.vp)+norm(u(1,2)-obj.wp)) ...
                + w5*(norm(u(obj.plan_horizon,1)-0)) ...
                + w6*lane_cost(u,obj.current_state,obj.dt,obj.plan_horizon,obj.agent_radius,obj.lane_change); 

            amin = -2; % min acceleration
            amax = 1; % max acceleration
            alphamax = 0.1; % max angular acceleration
            alphamin = -0.1; % min angular acceleration

            A = [diff(eye(obj.plan_horizon)), zeros(obj.plan_horizon-1,obj.plan_horizon)];
            A = [A; -A];
            A = [A; [zeros(obj.plan_horizon-1,obj.plan_horizon) diff(eye(obj.plan_horizon))]];
            A = [A; [zeros(obj.plan_horizon-1,obj.plan_horizon) -diff(eye(obj.plan_horizon))]];
%             disp(size(A));
            b = [amax*obj.dt*ones(obj.plan_horizon-1,1);-amin*obj.dt*ones(obj.plan_horizon-1,1); ...
                alphamax*obj.dt*ones(obj.plan_horizon-1,1);-alphamin*obj.dt*ones(obj.plan_horizon-1,1)];
%             disp(size(b));

            v_ulim = 30*ones(obj.plan_horizon,1); % max velocity = 10
            w_ulim = 0.5*ones(obj.plan_horizon,1); % max abgular vel = 1.5
            v_llim = 0*ones(obj.plan_horizon,1); % min vel = 0
            w_llim = -0.5*ones(obj.plan_horizon,1); % min angular vel = -1.5
            ub = [v_ulim w_ulim];
            lb = [v_llim w_llim];
            options = optimoptions(@fmincon,'Display','iter');
            if(avoid_collision == true)
                disp("test");
                controls = fmincon(cost,[obj.v0,obj.w0],A,b,[],[],lb,ub, ...
                    @(controls)nonlcon(controls,obj.plan_horizon,obj.current_state ,obj.agent_radius ,obj.obstacles,obj.dt), ...
                    options);
            else
                controls = fmincon(cost,[obj.v0,obj.w0],A,b,[],[],lb,ub,[],options);
            end
            obj.v0 = controls(:,1);
            obj.w0 = controls(:,2);
            obj.vp = controls(obj.update_horizon,1);
            obj.wp = controls(obj.update_horizon,2);
        end
        
        function plot_traj(obj)
            state = obj.current_state;
            for i =1:obj.plan_horizon
                state = nonhn_update(state,obj.v0(i),obj.w0(i),obj.dt);
                x(i) = state(1);
                y(i) = state(2);
            end
            obj.x_traj = x;
            obj.y_traj = y;
        end
        

    end
end