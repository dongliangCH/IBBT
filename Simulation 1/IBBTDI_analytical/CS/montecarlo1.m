function collisionProb = montecarlo1(MCnum, xbar, PP, N, world, param)
rng(0)
collisionProb = 0;
for k = 1:N+1
    collision_num = 0;
    X_k = mvnrnd(xbar(:,k), PP(:,:,k), MCnum)';
    X_k= X_k * world.scale;
    X_k = ceil(X_k);
    for i =1:MCnum
        p = X_k(:,i);
        if p(1)<1 || p(2)<1 || p(1)>49 || p(2)>79
            flag = 1;
        else
            flag = world.obstacle_map(p(1),p(2));
        end
        collision_num = collision_num + flag;
    end
%     collision_idx = arrayfun(@(x,y) collision(x,y,world), X_k(1,:), X_k(2,:));
%     collision_num = sum(collision_idx);
    collisionProb = max(collision_num/MCnum, collisionProb);
    if collisionProb > param.chanceConstraint
        return
    end
end

end

% function flag = collision(x,y, world)
% if x<1 || y<1 || x>199 || y>199
%     flag = 1;
% else
%     flag = world.obstacle_map(x,y);
% end
% end