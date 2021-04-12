function gridnav_nearoptsol(config)
    %% compute a near-optimal solution for the gridnav problem

    % configure the model here
    % mcfg.x_obst= [2,1;4,4;4,5]';
    Rseq=zeros(config.T,1);

    model=config.model;
    viscfg = struct;
    start_loc=gridnav_problem('reset', model,'rand');
%     start_loc=[5;1];
    
    Q = zeros(5,5,4);
    if config.visualize         
        viscfg.Q = Q;
        viscfg.model = model;
        viscfg.x = start_loc;
        viscfg.gview = gridnav_visualize(viscfg);
    end
    %% configure policy iteration
    qicfg.gamma = config.gamma;
    qicfg.eps = 1e-6;
    qicfg.model_params = {model};
    qicfg.problem = 'gridnav_problem';
    qicfg.verb = 0;
    Qdelta=zeros(config.T,1);
    qicfg.run = 1;
%     qicfg.eps=eps;        
    Qstar = reshape(qiter(qicfg), 5, 5, 4);
    eps=config.epsilon;
    for i=1:config.T

        Qprev=Q;
        iter=0;
        terminal=0;
        pos=start_loc;
        while iter<config.K && ~terminal
            
            probability = rand(1); 
            if probability >= eps
                 [qmax,movement] = max(Qprev(pos(1),pos(2),:));
            else  
                 movement=randi([1 4],1,1);
            end
             
            [xplus, rplus, terminal] = gridnav_mdp(model, pos, movement);
            if terminal 
                Q(pos(1), pos(2),movement)=rplus;
            else
                Q(pos(1), pos(2),movement) = Qprev(pos(1), pos(2),movement) + config.alpha*(rplus + config.gamma*max(Qprev(xplus(1),xplus(2),:))-Qprev(pos(1), pos(2),movement));
            end
            if config.visualize
                viscfg.x = xplus;
                viscfg.Q = Q ;
                viscfg.gview = gridnav_visualize(viscfg);
            end
            Rseq(i)=Rseq(i)+rplus;
            pos=xplus;
            iter=iter+1;    
        end
        eps=eps*config.epsilondecay;
        if i==1
            Qdelta(i)=0;
        else
            Qdelta(i)=norm(Q(:)-Qstar(:));
        end
    end

    if config.visualize
        viscfg.x = start_loc;
        viscfg.Q = Q ;
        viscfg.gview = gridnav_visualize(viscfg);
    end
    figure
    plot(Qdelta); title('Qdelta'); xlabel('iterations'); ylabel('difference');
    figure
    plot(Rseq); title('R'); xlabel('iterations'); ylabel('reward');
end
