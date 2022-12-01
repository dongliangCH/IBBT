function MC_path(Vertices, BeliefNodes, Path_idx, param, world)

plotCovariance(Vertices(Path_idx(1, 1), :)', BeliefNodes{Path_idx(1, 1)}{Path_idx(2, 1)}{1});

rng(0);
P0 = BeliefNodes{Path_idx(1, 1)}{Path_idx(2, 1)}{1};
PtildePrior0 = BeliefNodes{Path_idx(1, 1)}{Path_idx(2, 1)}{2};
mc_num = 40;

for k = 1:mc_num

    x0_MC = mvnrnd(zeros(4,1), P0, 1)';
    xtildePrior0 = mvnrnd(zeros(4,1), PtildePrior0, 1)';
    xhatPrior0_MC  = x0_MC - xtildePrior0;

for i = 1:size(Path_idx, 2)-1
    
    [meanTraj, ~, N]  = meanControl(Vertices(Path_idx(1, i), :)', Vertices(Path_idx(1, i+1), :)', param);  
%     plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
    [endP0, x0_MC, xhatPrior0_MC] = propagate( BeliefNodes{Path_idx(1, i)}{Path_idx(2, i)}{1}, BeliefNodes{Path_idx(1, i)}{Path_idx(2, i)}{2}, N, param, meanTraj, world, x0_MC, xhatPrior0_MC ); 
    plotCovariance(meanTraj(:,end), endP0);
    
end

end

end

function [endP0, x_f, xhatPriori_f] = propagate(P0, PtildePrior0, N, param, xbar, world, x0_MC, xhatPrior0_MC)

% Notation: Ak is single step of matrix, AA is 3d array with each Ak
% stacked in the 3rd dimension, A is block matrix

%% Problem setup

% Number of steps N
% Choose time step dt = 0.2

% Dynamics
dt = param.dt;
Ak = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
Bk = [dt ^ 2 / 2 0; 0 dt ^ 2 / 2; dt 0; 0 dt];
Gk = sqrt(dt) * diag([0.04 0.04 0.02 0.02]);

% Sequences of system matricies

% Cost
Qk = 2 * blkdiag(2, 2, 2, 2);
Rk = 1 * blkdiag(2, 2);

%% Kalman filter
% Observation model
ny = 4;
Ck = eye(4); 
Dk = zeros(ny, ny);
DD = repmat(Dk, [1, 1, N + 1]);

Xbar_temp = xbar * world.scale;
for i = 1 : N + 1
    if world.sensor_map(ceil(Xbar_temp(1+4*(i-1))), ceil(Xbar_temp(2+4*(i-1)))) %|| world.sensor_map(floor(Xbar_temp(1+4*(i-1))), floor(Xbar_temp(2+4*(i-1))))
        Dk = 0.1 * eye(4);
    else
        Dk = 4 * eye(4);
    end
    DD(:, :, i) = Dk;
end

% Initial state covariance
PhatPrior0 = P0 - PtildePrior0;

% Initial estimation error covariance  PtildePrior0
% Initial estimated state covariance  PhatPrior0 

% Solve for sequence of kalmain gains LL, error covariances PPtilde, and prior
% error covariances PPtm
[LL, PPtilde, PPtm] = solveKF(Ak, Gk, Ck, DD, N, PtildePrior0);

%% Time-varying LQR tracking

S(:,:,N+1) = Qk;
for k = N:-1:1
    
    K(:,:,k) = (Bk' * S(:,:,k+1) * Bk + Rk) \ Bk' * S(:,:,k+1) * Ak;
    S(:,:,k) = Qk + Ak' * S(:,:,k+1) * Ak - Ak' * S(:,:,k+1) * Bk * K(:,:,k);

end

%% State covariance and estimated state covariance

% PP(:,:,1)= blkdiag(P0, P0 - PPtilde(:,:,1));
% TrPP = zeros(1, N+1);
% TrPP(1) = trace(PP(1:4, 1:4, 1));
% % TVLQG covariance control cost
% Jc = 0;
% 
% for k = 1:N
%     
%     Fk = [AA(:,:,k)                               -BB(:,:,k) * K(:,:,k);
%           LL(:,:,k+1) * CC(:,:,k+1) * AA(:,:,k)   AA(:,:,k) - BB(:,:,k) * K(:,:,k) - LL(:,:,k+1) * CC(:,:,k+1) *AA(:,:,k)];
%     [row, ~] = size(GG(:,:,k));
%     [~, col] = size(LL(:,:,k+1) * DD(:,:,k+1));
%     Ek = [GG(:,:,k)                               zeros(row, col);
%           LL(:,:,k+1) * CC(:,:,k+1) * GG(:,:,k)   LL(:,:,k+1) * DD(:,:,k+1)];
%     PP(:,:,k+1) = Fk * PP(:,:,k) * Fk' + Ek * Ek';
%     TrPP(k+1) = trace(PP(1:4, 1:4, k+1));
%     Jc = Jc + trace( PP(5:8, 5:8, k) * (Qk + K(:,:,k)' * Rk * K(:,:,k)) );
%     
% end
% 
% 
% endP0 = PP(1:4, 1:4, end);
% endPtilde = PPtilde(1:4, 1:4,end);

PPhat(:,:,1) = P0 - PPtilde(:,:,1);
Jc = 0;
for k = 1:N
    PPhat(:,:,k+1) = (Ak - Bk * K(:,:,k)) * PPhat(:,:,k) * (Ak - Bk * K(:,:,k))' + LL(:,:,k+1) * Ck * PPtm(:,:,k+1);
end

endP0 = PPhat (:, :, end) + PPtilde(:, :,end);
endPtilde = PPtilde(:, :,end);

CovCost = dt * Jc;


    %% Monte Carlo

%     figure(10); 
%     hold on
    [x_f, xhatPriori_f]  = montecarlo(Ak, Bk, Gk, Ck, DD, LL, xbar, ...
        N, x0_MC, xhatPrior0_MC, K);

    %% Plot result

%     % Step numbers to draw covariance ellipses at
%     ellipse_steps = 0:N:N;
% 
%     figure(1);
%     drawResult(xbar, PP, PPtilde, ellipse_steps)

end

function [x_f, xhatPriori_f] = montecarlo(Ak, Bk, Gk, Ck, DD, LL, xbar, ...
    N,  x0_MC, xhatPrior0_MC, K)


% Sizes of inputs
nx = size(Ak, 1);
nu = size(Bk, 2);
ny = size(Ck, 1);
nw = size(Gk, 2);
nv = ny;

% rng(0);

    U = zeros(nu,N);
    x_MC = zeros(nx,N+1);
    xhat_MC = zeros(nx,N+1);
    xhatPrior_MC = zeros(nx,N+1);
    y_MC = zeros(ny,N+1);
    x_MC(:,1) = x0_MC;

    for k = 1:(N + 1)

        v = randn(nv, 1);
        w = randn(nw, 1);

        % Measurement
        y_MC(:, k) = Ck * x_MC(:, k) + DD(:, :, k) * v;

        % Prior estimate
        if k == 1
            xhatPrior_MC(:,k) = xhatPrior0_MC;
        else
            xhatPrior_MC(:, k) = Ak * xhat_MC(:, k - 1) ...
                + Bk * U(:, k - 1);
        end

        xhat_MC(:,k) =  xhatPrior_MC(:,k) + LL(:,:,k) * (y_MC(:,k) - Ck * xhatPrior_MC(:,k));
        
        % Control
        if k <= N
            U(:, k) = - K(:,:,k) * xhat_MC(:,k);
        end

        % Dynamics
        if k <= N
            x_MC(:, k + 1) = Ak * x_MC(:, k) ...
                + Bk * U(:, k) + Gk * w;
        end

    end

    x_f = x_MC(:,end);
    xhatPriori_f = xhatPrior_MC(:,end);
    
    hold on;
    plot( (x_MC(1,:) + xbar(1,:)), (x_MC(2,:) + xbar(2,:) ), 'color', 0.6 * ones(3, 1));
    

end