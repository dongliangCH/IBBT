function collisionProb = montecarlo1(MCnum, xbar, PP, N, world, param)
rng(0)
collisionProb = 0;
for k = 1:N+1
    collision_num = 0;
    X_k = mvnrnd(xbar(:,k), PP(:,:,k), MCnum);
    for i =1:MCnum
        p = X_k(i,:);
        flag = collision_point(p, world, 2);
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