close all;
clear all;
clc;
plan_horizon = 50;
update_horizon = 10;
v0 = normrnd(5,2.5,plan_horizon,1); % initial guess of velocity
w0 = rand(plan_horizon,1); % initial guess of angular velocity
a = Mpc_agent(1,[0,0,0.7],[80,80,0.7],v0,w0);
b = Mpc_agent(2,[80,80,3.9],[0,0,3.9],v0,w0);
c = Mpc_agent(3,[0,80,-0.78],[80,0,-0.78],v0,w0);
d = Mpc_agent(4,[80,0,2.3],[0,80,2.3],v0,w0);

a.obstacles = [b,c,d];
b.obstacles = [a,c,d];
c.obstacles = [a,b,d];
d.obstacles = [a,b,c];

% Initialize video
myVideo = VideoWriter('data/mpc_4agent_avoid_collision'); %open video file
myVideo.FrameRate = 15;  %can adjust this, 5 - 10 works well for me
open(myVideo)
f0 = figure;
threshold = 2.5;
while(norm(a.current_state-a.goal_state)>threshold || norm(b.current_state-b.goal_state)>threshold ...
        || norm(c.current_state-c.goal_state)>threshold || norm(d.current_state-d.goal_state)>threshold)
    
   a.predict_controls();
   b.predict_controls();
   c.predict_controls();
   d.predict_controls();
   
   for i=1:update_horizon
        if(norm(a.current_state - a.goal_state)<=threshold && norm(b.current_state - b.goal_state)<=threshold ...
                && norm(c.current_state - c.goal_state)<=threshold && norm(d.current_state - d.goal_state)<=threshold)
            break;
        end
        
        if(norm(a.current_state - a.goal_state)>threshold)
           a.current_state = nonhn_update(a.current_state,a.v0(i),a.w0(i),a.dt);
           a.v_list = [a.v_list, a.v0(i)];
           a.w_list = [a.w_list, a.w0(i)];
        end
        if(norm(b.current_state - b.goal_state)>threshold)
           b.current_state = nonhn_update(b.current_state,b.v0(i),b.w0(i),b.dt);
           b.v_list = [b.v_list, b.v0(i)];
           b.w_list = [b.w_list, b.w0(i)];
        end
        if(norm(c.current_state - c.goal_state)>threshold)
           c.current_state = nonhn_update(c.current_state,c.v0(i),c.w0(i),c.dt);
           c.v_list = [c.v_list, c.v0(i)];
           c.w_list = [c.w_list, c.w0(i)];
        end
        if(norm(d.current_state - d.goal_state)>threshold)
           d.current_state = nonhn_update(d.current_state,d.v0(i),d.w0(i),d.dt);
           d.v_list = [d.v_list, d.v0(i)];
           d.w_list = [d.w_list, d.w0(i)];
        end
       clf(f0)
       hold on;
       plot([a.init_state(1),a.goal_state(1),b.init_state(1),b.goal_state(1), ...
           c.init_state(1),c.goal_state(1),d.init_state(1),d.goal_state(1)], ...
           [a.init_state(2),a.goal_state(2),b.init_state(2),b.goal_state(2), ...
           c.init_state(2),c.goal_state(2),d.init_state(2),d.goal_state(2)],'rx');

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
       
       viscircles([d.current_state(1),d.current_state(2)], d.agent_radius,'Color','b');
       text(d.current_state(1),d.current_state(2),int2str(d.id));
       plot([d.current_state(1),d.goal_state(1)],[d.current_state(2),d.goal_state(2)],':k');
       d.plot_traj();
       plot(d.x_traj,d.y_traj,'b.');

       xlim([-20,100]);
       ylim([-20,100]);
       pause(1/60);
       frame = getframe(gcf); %get frame
       writeVideo(myVideo, frame);
       hold off;
       
   end
end
close(myVideo)
close(f0)

% f1 = figure;
% plot(trace_path(:,1),trace_path(:,2), '.')
% xlim([init_state(1)-10 goal_state(1)+10]);
% ylim([init_state(2)-10 goal_state(2)+10]);
% title("Agent's positions")
% saveas(gcf,'data/Positions.png')
% close(f1)
% 
% f2 = figure;
% plot(a.v_list,'red')
% ylim([0 21]);
% title("Velocity vs Timesteps")
% xlabel('Timesteps')
% ylabel('Velocity')
% % saveas(gcf,'data/Vel_vs_timesteps.png')
% 
% % 
% f3 = figure;
% plot(a.w_list,'blue')
% ylim([-0.5 0.5]);
% title("Angular Velocity vs Timesteps")
% xlabel('Timesteps')
% ylabel('Angular Velocity')
% % saveas(gcf,'data/AngVel_vs_timesteps.png')
