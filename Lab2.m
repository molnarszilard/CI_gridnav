part=2; %1-for part 1, 2 for part 2
random_map=1; % random_map=1 - create a random map, 0- use predefined values


%% creating the map
cfg.size = [5 5]; % generate a 5-by-5 map
if random_map==1
    nr_of_obstacles=5;
    cfg.x_goal = randi([1 nr_of_obstacles],1,2)'; %give a random goal
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
    start_loc=gridnav_problem('reset', model,'rand');
else
    cfg.x_goal = [2;5]; % an example case
    cfg.x_obst = [1 1,5,4,3;2,5,3,4,1];  % an example case
    start_loc=[5;1];
    model = gridnav_problem('model', cfg);
end

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
config.start=start_loc;

%% Part 1
if part==1
    [Qseq, Rseq]  = qlearning(config);
end

%% Part 2 
if part==2
    config.visualize=0;
    config.epsilon = 1;
    i = 1;
    l = length(0.9:0.01:0.99);
    L = l(1,1);
    QDeltas = zeros(L,config.T);
    R = zeros(L,config.T);
    epsilondecay=0.9:0.01:0.99;

    qicfg.gamma = config.gamma;
    qicfg.eps = 1;
    qicfg.model_params = {model};
    qicfg.problem = 'gridnav_problem';
    qicfg.verb = 0;
    qicfg.run=1;
    Qstar = reshape(qiter(qicfg), 5, 5, 4); %qiter->kiszamol Qoptimot ->reshape-> tobb dimenziossa tenni megint a Q-t

    for k=1:L
        config.epsilondecay=epsilondecay(k);
        [Qseq,Rseq] = qlearning(config);
        Qdelta=zeros(config.T,1);
        for m=1:config.T
            Q=Qseq(m,:,:,:);
            Qdelta(m)=norm(Q(:)-Qstar(:));
        end
        QDeltas(i,:) = Qdelta;
        R(i,:) = Rseq;
        i=i+1;    
    end
        figure
    string = strings(L);
    for j=1:L
        plot(QDeltas(j,:)); title('Q-Q*'); xlabel('iterations'); ylabel('differencies');
        string(j) =strcat('Q for epsilondecay =',num2str(epsilondecay(j)));
        hold on     
    end
    legend(string(:))
    hold off;

    figure
    string = strings(L);
    for j=1:L
        plot(R(j,:)); title('Reward'); xlabel('iterations'); ylabel('reward values');
         string(j) =strcat('Returns for epsilondecay =',num2str(epsilondecay(j)));
        hold on     
    end
    legend(string(:))
    hold off;
end

