function [Q_optim, h] = iteratiaQ(discount,epsQiter, model, start_loc)

i = 0;
Q_init = zeros(5,5,4);
movement=randi([1 4],1,1);
[xplus, rplus, terminal] = gridnav_mdp(model, start_loc, movement);
Q = Q_init;
norm = 100;
while i<epsQiter 
    Qprev = Q;
    for column=1:5
        for row=1:5
            position = [column;row];
            %if isempty(ismember(model.x_obst,position)) && isempty(ismember(model.x_goal,position))
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
                       Q(column,row,u) = rplus + discount*maxq_good_solution(Qprev,column,row,u,[5,5],model);
                   end                   
               end
            else
                Q(column,row,:) = ones(1,4)*(-100);
            end                     
        end
    end
    %norm = abs(Q - Qprev);    
    i = i+1;
end
Q_optim = Q;
[maxValue,h]=max(Q_optim,[],3);
end

