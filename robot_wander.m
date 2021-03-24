cfg.size = [5 5];
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
model = gridnav_problem('model', cfg);
start_loc=gridnav_problem('reset', model,'rand');
% for i=1:nr_of_obstacles
%    u=[1,1,1,1];
%    if cfg.x_obst(:,i)==start_loc+[-1;0] | start_loc(1)==1
%       u(1)=0;
%    end
%    if cfg.x_obst(:,i)==start_loc+[1;0] | start_loc(1)==5
%       u(2)=0;
%    end
%    if cfg.x_obst(:,i)==start_loc+[0-1] | start_loc(2)==1
%       u(3)=0;
%    end
%    if cfg.x_obst(:,i)==start_loc+[0;1] | start_loc(2)==5
%       u(4)=0;
%    end
% end
% if u(1)
%     ustart=1;
% elseif u(2)
%     ustart=2;
% elseif u(3)
%     ustart=3;
% elseif u(4)
%     ustart=4;
% else
%     'Please re-run the code, you are blocked!'  
% end

% use the created to simulate a transition

% [xplus, rplus, terminal] = gridnav_mdp(model, start_loc, ustart);   

viscfg = struct;
viscfg.model = model;
viscfg.x = start_loc;
viscfg.gview = gridnav_visualize(viscfg);
epsqiter=100; %
runs=0;
max_runs=3;
while runs<max_runs
    iter=0;
    xplus=start_loc;
    terminal=0;
    while iter<epsqiter && ~terminal
        movement=randi([1 4],1,1); 
        [xplus, rplus, terminal] = gridnav_mdp(model, xplus, movement);     
        viscfg.x = xplus;
        viscfg.gview = gridnav_visualize(viscfg);
        iter=iter+1;    
    end
    iter
    pause;
    start_loc = gridnav_problem('reset', model,'rand')
    viscfg.x = start_loc;
    viscfg.gview = gridnav_visualize(viscfg);
    runs=runs+1;
end
