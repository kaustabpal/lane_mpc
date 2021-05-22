close all;
clear all;
clc;

plan_horizon = 40;
update_horizon = 10;

v0 = normrnd(5,2.5,plan_horizon,1); % initial guess of velocity
w0 = rand(plan_horizon,1); % initial guess of angular velocity
s0 = rand(plan_horizon,1); % slack variables
x=-50:0.5:50;
r_y=2*x-2;
rm_y = 2*x+2;
m_y=2*x+6;
lm_y = 2*x+10;
l_y = 2*x+14;
change_y=-0.5*x+5;
forward_y = 0.5*x;

a = Mpc_agent(1,[-12.5,-23,deg2rad(60)],[12.5,27,deg2rad(60)],v0,w0,s0,plan_horizon,update_horizon);
b = Mpc_agent(2,[-17.5,-33,deg2rad(60)],[8,26,deg2rad(60)],v0,w0,s0,plan_horizon,update_horizon);
c = Mpc_agent(3,[-20.5,-31,deg2rad(60)],[5.5,21,deg2rad(60)],v0,w0,s0,plan_horizon,update_horizon);
c.lane_change = true;

a.obstacles = [b];
b.obstacles = [a,c];
c.obstacles = [];
% a.goal_state = [a.current_state(1)+2, 2*(a.current_state(1)+2)-0.5, a.current_state(3)];
% b.goal_state = [a.current_state(1)-0.2, 2*(a.current_state(1)-0.2)-0.5, a.current_state(3)];
% Initialize video
myVideo = VideoWriter('data/lane_test_9'); %open video file
myVideo.FrameRate = 15;  %can adjust this, 5 - 10 works well for me
open(myVideo)
f0 = figure;
threshold = 0.35;

timeout = 100;

while( (norm(a.current_state-a.goal_state)>threshold ...
        || norm(b.current_state-b.goal_state)>threshold ...
        || norm(c.current_state-c.goal_state)>threshold) && timeout >0)
    
   for i=1:update_horizon
        if(norm(a.current_state - a.goal_state)<=threshold && norm(b.current_state - b.goal_state)<=threshold && norm(c.current_state - c.goal_state)<=threshold)
            break;
        end
        
        if(norm(a.current_state - a.goal_state)>threshold)
           a.predict_controls();
           a.current_state = nonhn_update(a.current_state,a.v0(i),a.w0(i),a.dt);
           a.v_list = [a.v_list, a.v0(i)];
           a.w_list = [a.w_list, a.w0(i)];
        end
        if(norm(b.current_state - b.goal_state)>threshold)
            if(-b.current_state(2)-0.5*b.current_state(1)+5<=0)
                b.lane_change = true;
            end
           b.predict_controls();
           b.current_state = nonhn_update(b.current_state,b.v0(i),b.w0(i),b.dt);
           b.v_list = [b.v_list, b.v0(i)];
           b.w_list = [b.w_list, b.w0(i)];
        end
        
        if(norm(c.current_state - c.goal_state)>threshold)
           c.predict_controls();
           c.current_state = nonhn_update(c.current_state,c.v0(i),c.w0(i),c.dt);
           c.v_list = [c.v_list, c.v0(i)];
           c.w_list = [c.w_list, c.w0(i)];
        end
        
       clf(f0)
       hold on;
       plot(x,r_y,'k');
       plot(x, rm_y, '--b');
       plot(x,m_y,'r');
       plot(x, lm_y, '--b');
       plot(x,l_y,'k');
       plot(x, change_y, '--r');
       plot(x,forward_y+b.current_state(2));
       plot([a.init_state(1),a.goal_state(1),b.init_state(1),b.goal_state(1),c.init_state(1),c.goal_state(1)], ...
           [a.init_state(2),a.goal_state(2),b.init_state(2),b.goal_state(2),c.init_state(2),c.goal_state(2)],'rx');

       viscircles([a.current_state(1),a.current_state(2)], a.agent_radius,'Color','b');
       text(a.current_state(1),a.current_state(2),int2str(a.id));
       plot([a.current_state(1),a.goal_state(1)],[a.current_state(2),a.goal_state(2)],':k');
       a.plot_traj();
       plot(a.x_traj,a.y_traj,'b.');
       
       viscircles([b.current_state(1),b.current_state(2)], b.agent_radius,'Color','b');
       text(b.current_state(1),b.current_state(2),int2str(b.id));
       plot([b.current_state(1),b.goal_state(1)],[b.current_state(2),b.goal_state(2)],':k');
       b.plot_traj();
       plot(b.x_traj,b.y_traj,'b.');
       
       viscircles([c.current_state(1),c.current_state(2)], c.agent_radius,'Color','b');
       text(c.current_state(1),c.current_state(2),int2str(c.id));
       plot([c.current_state(1),c.goal_state(1)],[c.current_state(2),c.goal_state(2)],':k');
       c.plot_traj();
       plot(c.x_traj,c.y_traj,'b.');
       
       xlim([-40, 40]);
       ylim([-40, 40]);
       pause(1/60);
       timeout = timeout - 0.1;
       frame = getframe(gcf); %get frame
       writeVideo(myVideo, frame);
       hold off;
       
   end
%    a.goal_state = [a.current_state(1)+2, 2*(a.current_state(1)+2)-0.5, a.current_state(3)];
%    b.goal_state = [a.current_state(1)+1, 2*(a.current_state(1)+ 1)-0.5, a.current_state(3)];
end

close(myVideo)
close(f0)
