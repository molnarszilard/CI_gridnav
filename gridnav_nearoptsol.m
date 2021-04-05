function gridnav_nearoptsol(config)
%% compute a near-optimal solution for the gridnav problem

% configure the model here
% mcfg.x_obst= [2,1;4,4;4,5]';
mcfg.rew_step=-.1; 
mcfg.rew_obst=-.1;
mcfg.rew_goal=10;

%% creating the map
mcfg.size = [5 5]; % generate a 5-by-5 map
% max_x=cfg.size(1);
% max_y=cfg.size(2);
% cfg.x_goal = randi([1 5],1,2)'; %give a random goal
mcfg.x_goal = [2;5]; % an example case
mcfg.x_obst = [1 1,5,4,3;2,5,3,4,1];  % an example case
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
% model = gridnav_problem('model', cfg);
%% configure policy iteration
qicfg.gamma = config.gamma;
qicfg.eps = config.epsilon;
qicfg.model_params = {mcfg};
qicfg.problem = 'gridnav_problem';
qicfg.verb = 0;
Qdelta=zeros(config.T,1);
for i=1:config.T
    
    qicfg.run = i;
    if i>1
        Qprev=Qstar;
    end
    Qstar = reshape(qiter(qicfg), 5, 5, 4);
    qicfg.eps=qicfg.eps*config.epsilondecay;
    if i==1
        Qdelta(i)=0;
    else
        Qdelta(i)=norm(Qstar(:)-Qprev(:));
    end
end
plot(Qdelta);
end
