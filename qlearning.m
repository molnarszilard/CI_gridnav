function [Qseq, Rseq] = qlearning(config)

    Rseq=zeros(config.T,1);
    Qseq = zeros(config.T,5,5,4);
    model=config.model;
    viscfg = struct;
    start_loc=config.start;
    Q = zeros(5,5,4);   
    if config.visualize       
        viscfg.Q = Q;
        viscfg.model = model;
        viscfg.x = start_loc;
        viscfg.gview = gridnav_visualize(viscfg);
    end
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
        eps=eps*config.epsilondecay; %for part2 eps csokken
        Qseq(i,:,:,:) = Q;
%           pause;
    end

    if config.visualize
        viscfg.x = start_loc;
        viscfg.Q = Q ;
        viscfg.gview = gridnav_visualize(viscfg);
    end 

end
