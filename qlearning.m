function [Qseq, Rseq] = qlearning(config)

    Rseq=zeros(config.T);
    Qseq = zeros(config.T,5,5,4);
    model=config.model;
    viscfg = struct;
    start_loc=gridnav_problem('reset', model,'rand');
    Q = zeros(5,5,4);
    
    if config.visualize         
        viscfg.Q = Q;
        viscfg.model = model;
        viscfg.x = start_loc;
        viscfg.gview = gridnav_visualize(viscfg);
    end
    epsmove=0;
    nonepsmove=0;
    for i=1:config.T
        Qprev=Q;
        iter=0;
        xplus=start_loc;
        terminal=0;
        eps=config.epsilon;
        pos=start_loc;
        
        while iter<config.K && ~terminal
            
            probability = rand(1);
             
            if probability >= eps
                 [qmax,movement] = max(Qprev(pos(1),pos(2),:)); 
                 nonepsmove=nonepsmove+1;
            else  
                 movement=randi([1 4],1,1);
                 epsmove=epsmove+1;
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
            eps=eps*config.epsilondecay;
            pos=xplus;
            iter=iter+1;    
        end
        Qseq(i,:,:,:) = Q;
        Q
        Rseq(i)
        iter
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
end
