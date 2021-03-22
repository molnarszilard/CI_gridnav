function maxQ=maxq_good_solution(Q,i,j,k,mapsize,model)
nrOfObstacles =size(model.x_obst);
if k==1
    if i>1
        position=[i-1;j];
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
            maxQ=max(Q(i-1,j,:));
        else 
            maxQ=max(Q(i,j,:));
        end
    else
        maxQ=max(Q(i,j,:));
    end
end
if k==2
    if i<mapsize(1)
        position=[i+1;j];
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
            maxQ=max(Q(i+1,j,:));
        else 
            maxQ=max(Q(i,j,:));
        end
    else
        maxQ=max(Q(i,j,:));
    end
end
if k==3
    if j>1
        position=[i;j-1];
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
            maxQ=max(Q(i,j-1,:));
        else 
            maxQ=max(Q(i,j,:));
        end
    else
        maxQ=max(Q(i,j,:));
    end
end
if k==4
    if j<mapsize(2)
        position=[i;j+1];
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
            maxQ=max(Q(i,j+1,:));
        else 
            maxQ=max(Q(i,j,:));
        end
    else
        maxQ=max(Q(i,j,:));
    end
end
end
