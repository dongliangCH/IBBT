function [collisionProb, CovCost] = MCCollisionProb(Vertices, P0, PtildePrior0, Path_idx, param, world, plot_flag)

MCnum = 500;
Xbar = [Vertices(Path_idx(1, 1), :)'];
Step = 0;
for i = 1:size(Path_idx, 2)-1        
    [meanTraj, ~, N]  = meanControl(Vertices(Path_idx(1, i), :)', Vertices(Path_idx(1, i+1), :)', param);
    Xbar = [Xbar, meanTraj(:, 2:end)];
    Step = Step + N;
end
[collisionProb, CovCost] = propagate(P0, PtildePrior0, Step, param, Xbar, world, MCnum, plot_flag);

if plot_flag
    plot(Xbar(1,:), Xbar(2,:), 'color', 'g', 'LineWidth', 1); hold on
end

end

function [collisionProb, CovCost] = propagate(P0, PtildePrior0, N, param, xbar, world, MCnum, plot_flag)

% Notation: Ak is single step of matrix, AA is 3d array with each Ak
% stacked in the 3rd dimension, A is block matrix

%% Problem setup

% Number of steps N

% Dynamics
dt = param.dt;
Ak = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
Bk = [dt ^ 2 / 2 0; 0 dt ^ 2 / 2; dt 0; 0 dt];
Gk = 1 * sqrt(dt) * diag([0.04 0.04 0.02 0.02]);

% Cost
Qk = 2 * blkdiag(2, 2, 2, 2);
Rk = 1 * blkdiag(2, 2);

%% Kalman filter
% Observation model
% ny = 4;
% Ck = eye(4); 
% Dk = zeros(ny, ny);
% DD = repmat(Dk, [1, 1, N + 1]);
% 
% for i = 1 : N + 1
%     if visible(xbar(:,i), world)
%         Dk = 0.1 * eye(4);
%     else
%         Dk = 4 * eye(4);
%     end
%     DD(:, :, i) = Dk;
% end
Marker = world.Marker;
for i = 1 : N + 1
    Ck = [];
    Dk = [];
    for k = 1 : world.NumMarker
        marker = Marker(k, :);
        p = xbar(:,i);
        vec_norm = norm(marker - [p(1), p(2)]);
        pathx = linspace(p(1), marker(1), vec_norm/0.2);
        pathy = linspace(p(2), marker(2), vec_norm/0.2);
        if ~MeanCollisionCheck([pathx; pathy], world, 2)
            Ck = [Ck; [1 0 0 0; 0 1 0 0]];
            Dk = blkdiag(Dk, 0.02 * vec_norm^2 * diag([1 1]) + diag([0.01 0.01]));
        end    
    end
    if isempty(Ck)
        Ck = zeros(2,4);
    end
    if isempty(Dk)
        Dk = diag([1,1]);
    end
    CC(i).Ck = Ck;
    DD(i).Dk = Dk;
end

% Initial estimation error covariance  PtildePrior0
% Initial estimated state covariance  PhatPrior0 
PhatPrior0 = P0 - PtildePrior0;

% Solve for sequence of kalmain gains LL, error covariances PPtilde, and prior
% error covariances PPtm
[LL, PPtilde, PPtm] = solveKF(Ak, Gk, CC, DD, N, PtildePrior0);

%% Time-varying LQR tracking
S(:,:,N+1) = Qk;
for k = N:-1:1    
    K(:,:,k) = (Bk' * S(:,:,k+1) * Bk + Rk) \ Bk' * S(:,:,k+1) * Ak;
    S(:,:,k) = Qk + Ak' * S(:,:,k+1) * Ak - Ak' * S(:,:,k+1) * Bk * K(:,:,k);
end

%% State covariance and estimated state covariance
PPhat(:,:,1) = P0 - PPtilde(:,:,1);
PP(:,:,1) = P0(1:2,1:2);
Jc = 0;
for k = 1:N
    PPhat(:,:,k+1) = (Ak - Bk * K(:,:,k)) * PPhat(:,:,k) * (Ak - Bk * K(:,:,k))' + LL(k+1).Lk * CC(k+1).Ck * PPtm(:,:,k+1);
    PP(:,:,k+1) = PPhat(1:2,1:2,k+1) + PPtilde(1:2,1:2,k+1);
    PP_all = PPhat(:,:,k+1) + PPtilde(:,:,k+1);
    Jc = Jc + trace( PP_all * (Qk + K(:,:,k)' * Rk * K(:,:,k)) );
end
CovCost = dt * Jc;

%% Monte Carlo Simulation
collisionProb = mcSimulation(MCnum, Ak, Bk, Gk, CC, DD, LL, xbar, N, PhatPrior0, PtildePrior0, K, world, plot_flag);

%% Plot result
% % Step numbers to draw covariance ellipses at
% ellipse_steps = 0:2:N;
% drawResult(xbar, PP, PPtilde, ellipse_steps)

end

function collisionProb = mcSimulation(MCnum, Ak, Bk, Gk, CC, DD, LL, xbar, N, PhatPrior0, PtildePrior0, K, world, plot_flag)

% Sizes of inputs
nx = size(Ak, 1);
nu = size(Bk, 2);
nw = size(Gk, 2);

% rng(0);
collision = 0;

for mc = 1:MCnum

    xhatPrior0_MC = mvnrnd(xbar(:,1) - xbar(:,1), PhatPrior0 + 0.01 * eye(4), 1)';
    xtildePrior0 = mvnrnd(zeros(nx,1), PtildePrior0, 1)';
    x0_MC = xhatPrior0_MC + xtildePrior0;

    U = zeros(nu,N);
    x_MC = zeros(nx,N+1);
    xhat_MC = zeros(nx,N+1);
    xhatPrior_MC = zeros(nx,N+1);
    x_MC(:,1) = x0_MC;
    
    for k = 1:(N + 1)
        Ck = CC(k).Ck;
        Dk = DD(k).Dk;
        Lk = LL(k).Lk;
        nv = size(Ck, 1);
        v = randn(nv, 1);
        w = randn(nw, 1);
    
        % Measurement
        y_MC = Ck * x_MC(:, k) + Dk * v;
    
        % Prior estimate
        if k == 1
            xhatPrior_MC(:,k) = xhatPrior0_MC;
        else
            xhatPrior_MC(:, k) = Ak * xhat_MC(:, k - 1) + Bk * U(:, k - 1);
        end
    
        xhat_MC(:,k) =  xhatPrior_MC(:,k) + Lk * (y_MC - Ck * xhatPrior_MC(:,k));
        
        % Control
        if k <= N
            U(:, k) = - K(:,:,k) * xhat_MC(:,k);
        end
    
        % Dynamics
        if k <= N
            x_MC(:, k + 1) = Ak * x_MC(:, k) + Bk * U(:, k) + Gk * w;
        end
    end
    if plot_flag
        hold on;
        plot( (x_MC(1,:) + xbar(1,:)), (x_MC(2,:) + xbar(2,:) ), 'color', 0.6 * ones(3, 1));
    end
    if MeanCollisionCheck(x_MC(1:2,:) + xbar(1:2,:), world, 2)
        collision = collision + 1;
    end

end

collisionProb = collision / MCnum;
  
end