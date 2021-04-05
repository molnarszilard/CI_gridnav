function gridnav_nearoptsol(config)
    %% compute a near-optimal solution for the gridnav problem

    % configure the model here
    % mcfg.x_obst= [2,1;4,4;4,5]';
    Rseq=zeros(config.T);
    mcfg.rew_step=-.1; 
    mcfg.rew_obst=-.1;
    mcfg.rew_goal=10;
    model=config.model;
    viscfg = struct;
    start_loc=gridnav_problem('reset', model,'rand');
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
    qicfg.eps = 1e-6;
    qicfg.model_params = {mcfg};
    qicfg.problem = 'gridnav_problem';
    qicfg.verb = 0;
    Qdelta=zeros(config.T,1);
    Q = zeros(5,5,4);
    for i=1:config.T
        qicfg.run = i;
%         qicfg.eps=eps;
        
        Qstar = reshape(qiter(qicfg), 5, 5, 4);
        Qprev=Q;
        Q = zeros(5,5,4);
        iter=0;
        xplus=start_loc;
        terminal=0;
        eps=config.epsilon;
        pos=start_loc;
        
        while iter<config.K && ~terminal
            
            probability = rand(1);
             
            if probability >= eps
                 [qmax,movement] = max(Qstar(pos(1),pos(2),:)); 
            else  
                 movement=randi([1 4],1,1);
            end
             
            [xplus, rplus, terminal] = gridnav_mdp(model, pos, movement);
            if terminal 
                Q(pos(1), pos(2),movement)=rplus;
            else
                Q(pos(1), pos(2),movement) = Qstar(pos(1), pos(2),movement) + config.alpha*(rplus + config.gamma*max(Qstar(xplus(1),xplus(2),:))-Qstar(pos(1), pos(2),movement));
            end
            if config.visualize
                viscfg.x = xplus;
                viscfg.Q = Q ;
                viscfg.gview = gridnav_visualize(viscfg);
            end
            Rseq(i)=Rseq(i)+rplus;
            eps=eps*config.epsilondecay;
            pos=xplus;
            iter=iter+1;    
        end
%         Qseq(i,:,:,:) = Q;
%         Q
%         Rseq(i)
%         iter
        if i==1
            Qdelta(i)=0;
        else
            Qdelta(i)=norm(Qprev(:)-Q(:));
        end
    %     pause;
    %     start_loc = gridnav_problem('reset', model,'rand')
%           pause;
    end

    if config.visualize
        viscfg.x = start_loc;
        viscfg.Q = Q ;
        viscfg.gview = gridnav_visualize(viscfg);
    end 
%     nonepsmove
%     epsmove
    figure
    plot(Qdelta);
    figure
    plot(Rseq);
end
