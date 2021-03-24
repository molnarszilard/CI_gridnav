function [Q,h_optim, iteratiiMajore, iteratiiMinore] = legeaDeControl(discount, epshiter, epsQeval, epsQiter, model)
i = 0;
Q_init = zeros(model.size(1),model.size(2),4);
h_init = ones(model.size(1),model.size(2));
Q = Q_init;
h = h_init;
iteratiiMinore = 0;
normH = 100;
while normH>epshiter
    hprev = h;
    j=0;
    normQ=100;
    while j<epsQiter && normQ>=epsQeval
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
                            Q(column,row,u) = rplus + discount*Q_Teta(Qprev,hprev,column,row,u,model);
                        end
                    end
                end
            end
        end
        j = j+1;
        normQ = sum(sum(sum(abs(Qprev-Q))));
    end
    toc
    [m,h]=max(Q,[],3);
    normH = sum(sum(abs(hprev-h)));
    iteratiiMinore = cat(1,iteratiiMinore, j);
end
h_optim = h;
iteratiiMajore = length(iteratiiMinore);
end
