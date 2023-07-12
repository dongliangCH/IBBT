function [success, path, c] = FMTstar(State, NN, world, dim, param, plot_flag)

N = size(State,2);
idx = 1:N;                         % idx for every node  
A = zeros(1, N);                   % parent idx

W = true(1,N);                     % W: unvisited vertices
W(1) = false;
H = false(1,N);                    % H: open vertices
H(1) = true;
C = inf*ones(1,N);                 % C: cost-to-come
C(1) = 0;
z = 1;                             % idx of start
goal_idx = 2;

while z ~= goal_idx
    H_new = [];
    for x = NN{z}(W(NN{z}))
        Y_near = NN{x}(H(NN{x}));
        m = length(Y_near);
        seg_cost = zeros(1, m);
        for i=1:m
            [Path(i).path, seg_cost(i)] = dubins_curve(State(:,Y_near(i)), State(:,x), param.radi, param.dt, 1);
        end
        [c_min, y_idx] = min(C(Y_near) + seg_cost);
        y_min = Y_near(y_idx);
        collision_flag = MeanCollisionCheck(Path(y_idx).path(1:2, :), world, dim);  
        if ~collision_flag
            A(x) = y_min;
            C(x) = c_min;
            if plot_flag
                hold on
                plot(State(1,[x, y_min]), State(2,[x, y_min]), 'ok-');
            end
            H_new(end+1) = x;
            W(x) = false;
        end
    end
    H(H_new) = true;
    H(z) = false;
    if sum(H) == 0
        success = 0;
        path = [];
        c = 0;
        return 
    end

    [~, min_idx] = min(C(H));
    Idx = idx(H);
    z = Idx(min_idx);
end

success = 1;

path = z;
v = z;
while v ~= 1
    path = [A(v), path];
    v = A(v);
end
if plot_flag
    plot(State(1, path), State(2, path),'ob-','LineWidth',2,'MarkerFaceColor','b');
    hold off
end
c = C(z);