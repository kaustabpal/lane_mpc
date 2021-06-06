close all;
clear all;
clc;

plan_horizon = 40;
update_horizon = 10;

v0 = normrnd(5,2.5,plan_horizon,1); % initial guess of velocity
w0 = rand(plan_horizon,1); % initial guess of angular velocity

x=-100:0.5:100;
r_y=2*x-2;
rm_y = 2*x+10.94;
m_y=2*x+19.88;
lm_y = 2*x+28.83;
l_y = 2*x+37.77;
change_y=-0.5*x+5;
forward_y = 0.5*x;
% -40,-90  -40,30
a = Mpc_agent(1,[-17,-23.06,deg2rad(60)],[15.5,59.83,deg2rad(60)],v0,w0,plan_horizon,update_horizon);

a.obstacles = [];

% Initialize video
myVideo = VideoWriter('data/simple_lane1'); %open video file
myVideo.FrameRate = 15;  %can adjust this, 5 - 10 works well for me
open(myVideo)
f0 = figure;
threshold = 0.5;

timeout = 100;

while( (norm(a.current_state-a.goal_state)>threshold) && timeout >0)
    
   for i=1:update_horizon
        if(norm(a.current_state - a.goal_state)<=threshold)
            break;
        end
        
        if(norm(a.current_state - a.goal_state)>threshold)
            if(-a.current_state(2)-0.5*a.current_state(1)+5<=0) % lane change line
                a.lane_change = true;
            end
           a.predict_controls();
           a.current_state = nonhn_update(a.current_state,a.v0(i),a.w0(i),a.dt);
           a.v_list = [a.v_list, a.v0(i)];
           a.w_list = [a.w_list, a.w0(i)];
        end
        
       clf(f0)
       hold on;
       plot(x,r_y,'k');
       plot(x, rm_y, '--b');
       plot(x,m_y,'r');
       plot(x, lm_y, '--b');
       plot(x,l_y,'k');
       plot(x, change_y, '--r');
%        plot(x,forward_y+b.current_state(2));
       plot([a.init_state(1),a.goal_state(1)], ...
           [a.init_state(2),a.goal_state(2)],'rx');

       viscircles([a.current_state(1),a.current_state(2)], a.agent_radius,'Color','b');
       text(a.current_state(1),a.current_state(2),int2str(a.id));
       plot([a.current_state(1),a.goal_state(1)],[a.current_state(2),a.goal_state(2)],':k');
       a.plot_traj();
       plot(a.x_traj,a.y_traj,'b.');
              
       xlim([-70, 70]);
       ylim([-70, 70]);
       pause(1/60);
       timeout = timeout - 0.1;
       frame = getframe(gcf); %get frame
       writeVideo(myVideo, frame);
       hold off;
       
   end
end

close(myVideo)
close(f0)
