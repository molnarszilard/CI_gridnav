function [Q_optim, h] = iteratiaQ(discount,epsQiter,epsQeval, model)
i = 0;
Q_init = zeros(model.size(1),model.size(2),4);
Q = Q_init;
norm = 100;
while i<epsQiter && epsQeval<=norm
    Qprev = Q;
    for column=1:model.size(1)
        for row=1:model.size(2)
            position = [column;row];
            nrOfObstacles = size(model.x_obst);
            not_valid=0;
            for o = 1:nrOfObstacles(2)
                if(position(1) == model.x_obst(1,o) && position(2) == model.x_obst(2,o))
                    not_valid=1;
                end
            end
            if(position(1) == model.x_goal(1) && position(2) == model.x_goal(2))
                    not_valid=1;
            end
            if ~not_valid
               for u=1:4                
                   [xplus, rplus, terminal] = gridnav_mdp(model, [column;row], u);
                   if terminal
                       Q(column,row,u) = rplus;
                   else
                       Q(column,row,u) = rplus + discount*maxq_good_solution(Qprev,column,row,u,model);
                   end                   
               end
            end                     
        end
    end
    norm = sum(sum(sum(abs(Qprev-Q))));  
    i = i+1;
end
Q_optim = Q;
[maxValue,h]=max(Q_optim,[],3);
end

