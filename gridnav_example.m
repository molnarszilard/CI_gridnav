% create a model of the grid nav problem, see also help gridnav_problem
% the size along the X and Y coords respectively
cfg.size = [5 5];
% we could also configure the rewards: per regular step, field rew_step
% for collision with obstacle or wall: rew_obst; and for reaching the goal: rew_goal
max_x=cfg.size(1);
max_y=cfg.size(2);

cfg.x_goal = randi([1 5],1,2)';
cfg.x_obst = [;];  % ?obst..?
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
         n=n+1; %folosleges
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
%%


[xplus, rplus, terminal] = gridnav_mdp(model, start_loc, ustart);   
% xplus
% rplus
%%
viscfg = struct;
viscfg.model = model;
viscfg.x = xplus;
viscfg.gview = gridnav_visualize(viscfg);
epsqiter=100; %
epshiter=3; %
epsheval=1; %
h=zeros(1,epsqiter); %
h(1)=ustart;
iter=0;
discount=0.95;
viscfg.Q = zeros(5, 5, 4); 
viscfg.Q(start_loc(1),start_loc(2),ustart)=rplus;
while iter<epsqiter & ~terminal
    movement=randi([1 4],1,1); 
    [xplus, rplus, terminal] = gridnav_mdp(model, xplus, movement);     
    viscfg.x = xplus;
    viscfg.gview = gridnav_visualize(viscfg);
    iter=iter+1;    
%     model = gridnav_problem('reset', cfg,'rand');
end
pause;
viscfg.x = [];  % before, remove the robot state since we don't want to show it anymore
maxdist=max(max(distance));
maxQ=max(max(max(viscfg.Q)));
% viscfg.Q
% if we wanted to NOT reuse the view, but create a new figure, we could do:
viscfg.gview = [];
viscfg.gview = gridnav_visualize(viscfg);
pause


%% Partea 2

Q_optim = iteratiaQ(discount,epsqiter, model, start_loc);
h_optim = legeaDeControl(discount, epshiter, epsQeval, epsQiter, model, start_loc);
xplus = start_loc;
while ~terminal
    movement=h_optim(xplus);
    [xplus, rplus, terminal] = gridnav_mdp(model, xplus, movement);     
    viscfg.x = xplus;
    viscfg.gview = gridnav_visualize(viscfg);
end
