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
[xplus, rplus, terminal] = gridnav_mdp(model, start_loc, ustart);  
viscfg = struct;
viscfg.model = model;
viscfg.x = xplus;
epsQiter=100; 
epshiter=0; 
epsQeval=0.1; 
discount = 0.6;
iter=0;
tic
[Q_optim1, h_optim1] = iteratiaQ(discount,epsQiter, model, start_loc);
toc
[Q_optim2, h_optim2] = legeaDeControl(discount, epshiter, epsQeval, epsQiter, model, start_loc);
xplus = start_loc;

Q=Q_optim1;
H=h_optim1;
while iter<epsQiter && ~terminal
    movement=H(xplus(1),xplus(2));
    [xplus, rplus, terminal] = gridnav_mdp(model, xplus, movement);     
    viscfg.x = xplus;
    viscfg.gview = gridnav_visualize(viscfg);
    iter=iter+1;
end
pause
viscfg.Q = Q ;
viscfg.x = [];
viscfg.h = H;
viscfg.gview =  gridnav_visualize(viscfg);
pause
figure
mesh(max(Q,[],3))