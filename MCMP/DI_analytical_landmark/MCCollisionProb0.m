function MCCollisionProb(Vertices, P0_s, PtildePrior0_s, Path_idx, param, world)

plotCovariance(Vertices(Path_idx(1, 1), :)', P0_s); hold on;
rng(0);
mc_num = 50;

for k = 1:mc_num
    X_MC = [];
    CovCost = 0;
    x0_MC = mvnrnd(zeros(4,1), P0_s, 1)';
    xtildePrior0 = mvnrnd(zeros(4,1), PtildePrior0_s, 1)';
    xhatPrior0_MC  = x0_MC - xtildePrior0;
    P0 = P0_s;
    PtildePrior0 = PtildePrior0_s;

    for i = 1:size(Path_idx, 2)-1        
        [meanTraj, ~, N]  = meanControl(Vertices(Path_idx(1, i), :)', Vertices(Path_idx(1, i+1), :)', param);  
        plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1); 
        [P0, PtildePrior0, x_MC, x0_MC, xhatPrior0_MC, Cost] = propagate(P0, PtildePrior0, N, param, meanTraj, world, x0_MC, xhatPrior0_MC); 
        x_MC = x_MC + meanTraj;
        X_MC = [X_MC, x_MC];
        CovCost = CovCost + Cost;
        plotCovariance(meanTraj(:,end), P0);        
    end
    plot(X_MC(1,:), X_MC(2,:), 'color', 0.6 * ones(3, 1)); hold on;
end

end

function [endP0, endPtilde, x_MC, x_f, xhatPriori_f, CovCost] = propagate(P0, PtildePrior0, N, param, xbar, world, x0_MC, xhatPrior0_MC)

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

endP0 = PPhat (:, :, end) + PPtilde(:, :,end);
endPtilde = PPtilde(:, :,end);

%% Monte Carlo Simulation
% figure(10); 
% hold on
[x_MC, x_f, xhatPriori_f]  = mcSimulation(Ak, Bk, Gk, CC, DD, LL, N, x0_MC, xhatPrior0_MC, K);

end

function [x_MC, x_f, xhatPriori_f] = mcSimulation(Ak, Bk, Gk, CC, DD, LL, N, x0_MC, xhatPrior0_MC, K)

% Sizes of inputs
nx = size(Ak, 1);
nu = size(Bk, 2);
nw = size(Gk, 2);

% rng(0);

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

x_f = x_MC(:,end);
xhatPriori_f = xhatPrior_MC(:,end);
  
end