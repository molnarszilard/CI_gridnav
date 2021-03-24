%% Initial values
epsQiter=100; 
epshiter=0; 
epsQeval=0.1; 
discount = 0.95; % initial gamma

%% creating the map
cfg.size = [5 5]; % generate a 5-by-5 map
% max_x=cfg.size(1);
% max_y=cfg.size(2);
cfg.x_goal = randi([1 5],1,2)'; %give a random goal
% cfg.x_goal = [2;5]; % an example case
% cfg.x_obst = [1 1,5,4,3;2,5,3,4,1];  % an example case
nr_of_obstacles=5;
n=0;
while n<nr_of_obstacles % generate the obstacles
   new_obst = randi([1 5],1,2)';
   if cfg.x_goal~=new_obst
      exist=0;
         for i=1:n
           if cfg.x_obst(:,i)==new_obst
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

%% Partea 2 - Calculating the Q and h + Partea 3 time/iter_nr comparation
%Iteratia Q
tic
[Q_optim1, h_optim1, iteratii] = iteratiaQ(discount,epsQiter,epsQeval, model);
iteratii
toc

%Legea de control cu epsQeval mai mic
tic
[Q_optim2, h_optim2, iteratiiMajore, iteratiiMinore] = legeaDeControl(discount, epshiter, epsQeval, epsQiter, model);
toc
iteratiiMajore
iteratiiMinore
%Legea de control cu epsQeval mai mare
epsQeval22=0.9;
tic
[Q_optim22, h_optim22, iteratiiMajore22, iteratiiMinore22] = legeaDeControl(discount, epshiter, epsQeval22, epsQiter, model);
toc
iteratiiMajore22
iteratiiMinore22

%% Move the robot according to the calculated control
viscfg = struct;
viscfg.model = model;

%% Move as said by IteratiaQ
start_loc=gridnav_problem('reset', model,'rand');
viscfg.x = start_loc;
xplus = start_loc;
Q=Q_optim1;
H=h_optim1;
iter=0;
terminal = 0;
while iter<epsQiter && ~terminal
    movement=H(xplus(1),xplus(2));
    [xplus, rplus, terminal] = gridnav_mdp(model, xplus, movement);     
    viscfg.x = xplus;
    viscfg.gview = gridnav_visualize(viscfg);
    iter=iter+1;
end
viscfg.Q = Q ;
viscfg.x = [];
viscfg.h = H;
viscfg.gview =  gridnav_visualize(viscfg);
pause
figure
mesh(max(permute(Q,[2,1,3]),[],3))

%% Move as said by legeDeControl (first epsilon)
start_loc=gridnav_problem('reset', model,'rand');
viscfg.x = start_loc;
xplus = start_loc;
Q=Q_optim2;
H=h_optim2;
iter=0;
while iter<epsQiter && ~terminal
    movement=H(xplus(1),xplus(2));
    [xplus, rplus, terminal] = gridnav_mdp(model, xplus, movement);     
    viscfg.x = xplus;
    viscfg.gview = gridnav_visualize(viscfg);
    iter=iter+1;
end
viscfg.Q = Q ;
viscfg.x = [];
viscfg.h = H;
viscfg.gview =  gridnav_visualize(viscfg);
pause
figure
mesh(max(permute(Q,[2,1,3]),[],3))

%% Partea 3 - gamma comparation
model = gridnav_problem('model', cfg);

epsQiter=100; 
epshiter=0; 
epsQeval=1; 

for gamma=0.6:0.1:0.99
% for IteratieQ
%     tic
%     [Q_optim3, h_optim3, iteratii3] = iteratiaQ(gamma,epsQiter,epsQeval, model);
%     iteratii3
%     toc
% for LegeaDeControl
    tic
    [Q_optim33, h_optim33, iteratiiMajore33, iteratiiMinore33] = legeaDeControl(gamma, epshiter, epsQeval, epsQiter, model);
    iteratiiMajore33
    iteratiiMinore33
    toc
end
%plot(1:6,[5,2,3,3,2,2],1:6,[7,2,3,3,2,2],1:6,[10,2,3,3,2,2],1:6,[19,2,3,3,2,2]); legend('\gamma=0.6','\gamma=0.7','\gamma=0.8','\gamma=0.9')
