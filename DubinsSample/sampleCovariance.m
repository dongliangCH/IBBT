start_cord = [3,8,pi/6];
P0 = diag([0.10, 0.08, 0.08]);                                      % initial state distribution
mc_num = 1000;
x0_mc = mvnrnd(start_cord, P0, mc_num);
error = x0_mc - mean(x0_mc);
Cov = zeros(3,3);
for i = 1:size(error,1)
    Cov = Cov + error(i,:)'*error(i,:);
end
Cov = Cov/(mc_num-1)
Cov = cov(x0_mc)