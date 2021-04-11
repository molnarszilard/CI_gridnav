%% creating the map
cfg.size = [5 5]; % generate a 5-by-5 map
% max_x=cfg.size(1);
% max_y=cfg.size(2);
% cfg.x_goal = randi([1 5],1,2)'; %give a random goal
cfg.x_goal = [2;5]; % an example case
cfg.x_obst = [1 1,5,4,3;2,5,3,4,1];  % an example case
% nr_of_obstacles=5;
% n=0;
% while n<nr_of_obstacles % generate the obstacles
%    new_obst = randi([1 5],1,2)';
%    if cfg.x_goal~=new_obst
%       exist=0;
%          for i=1:n
%            if cfg.x_obst(:,i)==new_obst
%               exist=1;
%            end
%          end    
%          if exist==0
%             cfg.x_obst(:,n+1)=new_obst;
%             n=n+1;
%          end
%     end
%    
% end
model = gridnav_problem('model', cfg);
% S = struct('model', model,'K',1000,'T',100,'gamma', 0.95, 'alpha', 0.1, 'epsilon', 0.3, 'u', [1,2,3,4]);
%% Params
config=struct();
config.model=model;
config.T=100;
config.K=1000;
config.alpha=0.1;
config.gamma=0.9;
config.epsilon=0.3;
config.epsilondecay=1;
config.visualize=1;

%% Part 1
% [Qseq, Rseq]  = qlearning(config);

%% Part 2 
config.epsilon=1;
config.epsilondecay=0.9;
config.visualize=0;
gridnav_nearoptsol(config);
