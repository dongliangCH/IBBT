function collision_flag = collisionKnowTf(parent, node, world, dim, tf, param)

% Trajectory
states = @(t)DI_state(t, tf, parent(1), parent(2), parent(3), parent(4), node(1), node(2), node(3), node(4));

% Number of steps
% Choose time step dt = 0.2
dt = param.dt;
N = ceil (tf / dt);
N = max(4, N);   % at least 4 step, for matrix inverse
time = linspace(0, tf, N + 1);

xbar(:,1) = parent;
for i = 2 : N
    xbar(:, i) = states(time(i));
end
xbar(:,N+1) = node;

collision_flag = MeanCollisionCheck(xbar(1:2, :), world, dim);

end

function states = DI_state(t,t_s,x01,x02,x03,x04,x11,x12,x13,x14)
 states = [x11 + x13*(t - t_s) + ((t - t_s)^3*(2*x01 - 2*x11 + t_s*x03 + t_s*x13))/t_s^3 + ((t - t_s)^2*(3*x01 - 3*x11 + t_s*x03 + 2*t_s*x13))/t_s^2;
           x12 + x14*(t - t_s) + ((t - t_s)^3*(2*x02 - 2*x12 + t_s*x04 + t_s*x14))/t_s^3 + ((t - t_s)^2*(3*x02 - 3*x12 + t_s*x04 + 2*t_s*x14))/t_s^2;
           x13 + (2*(t - t_s)*(3*x01 - 3*x11 + t_s*x03 + 2*t_s*x13))/t_s^2 + (3*(t - t_s)^2*(2*x01 - 2*x11 + t_s*x03 + t_s*x13))/t_s^3;
           x14 + (2*(t - t_s)*(3*x02 - 3*x12 + t_s*x04 + 2*t_s*x14))/t_s^2 + (3*(t - t_s)^2*(2*x02 - 2*x12 + t_s*x04 + t_s*x14))/t_s^3];
end
