% create a model of the grid nav problem, see also help gridnav_problem
% the size along the X and Y coords respectively
cfg.size = [5 5];
% we could also configure the rewards: per regular step, field rew_step
% for collision with obstacle or wall: rew_obst; and for reaching the goal: rew_goal
max_x=cfg.size(1);
max_y=cfg.size(2);

cfg.x_goal = randi([1 5],1,2)';
cfg.x_obst = [;];
nr_of_obstacles=5;
n=0;
while n<nr_of_obstacles
   new_obst = randi([1 5],1,2)';
   if cfg.x_goal~=new_obst
      exist=0;
         for i=1:n
           if cfg.x_obst(:,i)==new_obst;
              exist=1;
           end
         end    
         if exist==0
            cfg.x_obst(:,n+1)=new_obst;
            n=n+1;
         end
    end
   
end
start_loc=[;];
while isempty(start_loc)
    new_start = randi([1 5],1,2)';
    if cfg.x_goal~=new_start
      exist=0;
      for i=1:nr_of_obstacles
         if cfg.x_obst(:,i)==new_start;
            exist=1;
         end
      end    
      if exist==0
         start_loc=new_start;
         n=n+1;
      end
    end
end
for i=1:nr_of_obstacles
   u=[1,1,1,1];
   if cfg.x_obst(:,i)==start_loc+[-1;0] | start_loc(1)==1
      u(1)=0;
   end
   if cfg.x_obst(:,i)==start_loc+[1;0] | start_loc(1)==5
      u(2)=0;
   end
   if cfg.x_obst(:,i)==start_loc+[0-1] | start_loc(2)==1
      u(3)=0;
   end
   if cfg.x_obst(:,i)==start_loc+[0;1] | start_loc(2)==5
      u(4)=0;
   end
end
if u(1)
    ustart=1;
elseif u(2)
    ustart=2;
elseif u(3)
    ustart=3;
elseif u(4)
    ustart=4;
else
    'Please re-run the code, you are blocked!'  
end
model = gridnav_problem('model', cfg);
% use the created to simulate a transition

% start from X=3, Y=1 and go right (u=2)
% xplus is the next state, rplus the reward resulting from the transition
% terminal is a boolean flag indicating whether the resulting state is
% terminal or not
[xplus, rplus, terminal] = gridnav_mdp(model, start_loc, ustart);   
xplus
rplus

%%
% show the updated state of the agent
viscfg = struct;
viscfg.model = model;
viscfg.x = xplus;
viscfg.gview = gridnav_visualize(viscfg);    % note we remember the view...

% ... so if we do another transition ...
[xplus, rplus, terminal] = gridnav_mdp(model, xplus, 2);   
% ... we can reuse it. reusing the view is recommended as creating it costs a lot of computation,
% whereas just reusing is cheap
viscfg.x = xplus;
viscfg.gview = gridnav_visualize(viscfg);

pause;
% to show a Q-function, we place it on viscfg
viscfg.x = [];  % before, remove the robot state since we don't want to show it anymore
% as an example, we initialize an arbitrary Q-function
% (in your solution the Q-function will be computed by the algorithm)
% note that Q must be an array with standard dimensions in order to be handled
% correctly by the visualization function:
% size on X (here, 5) x size on Y (here, 5) x number of actions (here, 4)
% n (= number of states = 5) rows, and 2 (number of actions) columns
viscfg.Q = rand(5, 5, 4);
viscfg.gview = gridnav_visualize(viscfg);   

pause;
% to show a policy in addition to the Q-function, add it to viscfg 
% h also has a standard structure: a matrix with 
% (size on X x size on Y) elements, each representing an action, with values 1 to 4
viscfg.h = [1 1 1 1 1; 2 2 2 2 2; 3 3 3 3 3; 4 4 4 4 4; 1 1 1 1 1];
% if we wanted to NOT reuse the view, but create a new figure, we could do:
viscfg.gview = [];
viscfg.gview = gridnav_visualize(viscfg);
