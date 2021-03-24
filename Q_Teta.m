function Q=Q_Teta(q,h,i,j,u,model)
mapsize=model.size;
nrOfObstacles =size(model.x_obst);
if u==1
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
            Q=q(i-1,j,h(i-1,j));
        else 
            Q=q(i,j,h(i,j));
        end
    else
        Q=q(i,j,h(i,j));
    end
end
if u==2
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
            Q=q(i+1,j,h(i+1,j));
        else 
           Q=q(i,j,h(i,j));
        end
    else
        Q=q(i,j,h(i,j));
    end
end
if u==3
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
            Q=q(i,j-1,h(i,j-1));
        else 
            Q=q(i,j,h(i,j));
        end
    else
        Q=q(i,j,h(i,j));
    end
end
if u==4
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
            Q=q(i,j+1,h(i,j+1));
        else 
            Q=q(i,j,h(i,j));
        end
    else
        Q=q(i,j,h(i,j));
    end
end
end
