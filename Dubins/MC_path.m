function MC_path(Vertices, BeliefNodes, Path_idx, param, world)

plotCovariance(Vertices(Path_idx(1, 1), :)', BeliefNodes{Path_idx(1, 1)}{Path_idx(2, 1)}{1});

rng(0);
P0 = BeliefNodes{Path_idx(1, 1)}{Path_idx(2, 1)}{1};
PtildePrior0 = BeliefNodes{Path_idx(1, 1)}{Path_idx(2, 1)}{2};
mc_num = 20;

for k = 1:mc_num

    x0_MC = mvnrnd(zeros(3,1), P0, 1)';
    xtildePrior0 = mvnrnd(zeros(3,1), PtildePrior0, 1)';
    xhatPrior0_MC  = x0_MC - xtildePrior0;

for i = 1:size(Path_idx, 2)-1
    [meanTraj, ~]  = dubins_curve(Vertices(Path_idx(1, i), :), Vertices(Path_idx(1, i+1), :), param.radi, param.dt, 1);
    meanTraj = [meanTraj, Vertices(Path_idx(1, i+1), :)'];
%     plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
    [endP0, x0_MC, xhatPrior0_MC] = propagate( BeliefNodes{Path_idx(1, i)}{Path_idx(2, i)}{1}, BeliefNodes{Path_idx(1, i)}{Path_idx(2, i)}{2}, param, meanTraj, world, x0_MC, xhatPrior0_MC ); 
    plotCovariance(meanTraj(:,end), endP0);
    
end

end

end

function [endP0, x_f, xhatPriori_f] = propagate(P0, PtildePrior0, param, xbar, world, x0_MC, xhatPrior0_MC)

% Notation: Ak is single step of matrix, AA is 3d array with each Ak
% stacked in the 3rd dimension, A is block matrix

%% Problem setup

% Number of steps N
% Choose time step dt = 0.2

% Dynamics
N = size(xbar,2) - 1;
dt = param.dt;
nx = 3;
nu = 1;
ny = 3;
nw = 3;

% Sequences of system matricies
AA = repmat(zeros(nx, nx), [1, 1, N]);
BB = repmat(zeros(nx, nu), [1, 1, N]);
for i = 1 : N
    Vk = xbar(:, i);
    At = [0 0 -sin(Vk(3));
          0 0  cos(Vk(3));
          0 0 0];
    Bt = [0;
          0;
          1];
      
    Ak = eye(nx) + At * dt; 
    Bk = Bt * dt;
    AA(:, :, i) = Ak;
    BB(:, :, i) = Bk; 
end

% Cost
Qk = 2 * blkdiag(2, 2, 2);
Rk = 1 * blkdiag(2);

Gk = sqrt(dt) * diag([0.02 0.02 0.02]);  
GG = repmat(Gk, [1, 1, N]);

%% Kalman filter
Ck = eye(3); 
CC = repmat(Ck, [1, 1, N + 1]);
Dk = zeros(ny, ny);
DD = repmat(Dk, [1, 1, N + 1]);

Xbar_temp = xbar * world.scale;
for i = 1 : N + 1
    if world.sensor_map(ceil(Xbar_temp(1, i)), ceil(Xbar_temp(2, i))) %|| world.sensor_map(floor(Xbar_temp(1+4*(i-1))), floor(Xbar_temp(2+4*(i-1))))
        DD(:, :, i) = 0.1 * eye(3);       
    else
        DD(:, :, i) = 2 * eye(3);
    end
end

% Initial state covariance
PhatPrior0 = P0 - PtildePrior0;

% Initial estimation error covariance  PtildePrior0
% Initial estimated state covariance  PhatPrior0 

% Solve for sequence of kalmain gains LL, error covariances PPtilde, and prior
% error covariances PPtm
[LL, PPtilde, PPtm] = solveKF(AA, GG, CC, DD, N, PtildePrior0);

%% Time-varying LQR tracking

S(:,:,N+1) = Qk;
for k = N:-1:1
    
    K(:,:,k) = (BB(:,:,k)' * S(:,:,k+1) * BB(:,:,k) + Rk) \ BB(:,:,k)' * S(:,:,k+1) * AA(:,:,k);
    S(:,:,k) = Qk + AA(:,:,k)' * S(:,:,k+1) * AA(:,:,k) - AA(:,:,k)' * S(:,:,k+1) * BB(:,:,k) * K(:,:,k);

end

%% State covariance and estimated state covariance

% PP(:,:,1)= blkdiag(P0, P0 - PPtilde(:,:,1));
% TrPP = zeros(1, N+1);
% TrPP(1) = trace(PP(1:3, 1:3, 1));
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
%     TrPP(k+1) = trace(PP(1:3, 1:3, k+1));
%     Jc = Jc + trace( PP(4:6, 4:6, k) * (Qk + K(:,:,k)' * Rk * K(:,:,k)) );
%     
% end
% 
% 
% endP0 = PP(1:3, 1:3, end);
% endPtilde = PPtilde(1:3, 1:3,end);

PPhat(:,:,1) = P0 - PPtilde(:,:,1);
Jc = 0;
for k = 1:N
    PPhat(:,:,k+1) = (AA(:,:,k) - BB(:,:,k) * K(:,:,k)) * PPhat(:,:,k) * (AA(:,:,k) - BB(:,:,k) * K(:,:,k))' + LL(:,:,k+1) * CC(:,:,k+1) * PPtm(:,:,k+1);
end

endP0 = PPhat (:, :, end) + PPtilde(:, :,end);
endPtilde = PPtilde(:, :,end);

CovCost = dt * Jc;

    %% Monte Carlo
    % Number of monte carlo trials to display (can be zero)
    MCnum = 20;

%     figure(10); 
%     hold on
    [x_f, xhatPriori_f] = montecarlo( AA, BB, GG, CC, DD, LL, xbar, ...
        N, x0_MC, xhatPrior0_MC, K);

    %% Plot result

%     % Step numbers to draw covariance ellipses at
%     ellipse_steps = 0:N:N;
% 
%     figure(2);
%     drawResult(xbar, PPtilde, ellipse_steps)

end

function [x_f, xhatPriori_f] = montecarlo( AA, BB, GG, CC, DD, LL, xbar, ...
    N, x0_MC, xhatPrior0_MC, K )


% Sizes of inputs
nx = size(AA, 1);
nu = size(BB, 2);
ny = size(CC, 1);
nw = size(GG, 2);
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
        y_MC(:, k) = CC(:, :, k) * x_MC(:, k) + DD(:, :, k) * v;

        % Prior estimate
        if k == 1
            xhatPrior_MC(:,k) = xhatPrior0_MC;
        else
            xhatPrior_MC(:, k) = AA(:, :, k - 1) * xhat_MC(:, k - 1) ...
                + BB(:, :, k - 1) * U(:, k - 1);
        end

        xhat_MC(:,k) =  xhatPrior_MC(:,k) + LL(:,:,k) * (y_MC(:,k) - CC(:,:,k) * xhatPrior_MC(:,k));
        
        % Control
        if k <= N
            U(:, k) = - K(:,:,k) * xhat_MC(:,k);
        end

        % Dynamics
        if k <= N
            x_MC(:, k + 1) = AA(:, :, k) * x_MC(:, k) ...
                + BB(:, :, k) * U(:, k) + GG(:, :, k) * w;
        end

    end

    x_f = x_MC(:,end);
    xhatPriori_f = xhatPrior_MC(:,end);
    
    hold on;
    plot( (x_MC(1,:) + xbar(1,:)), (x_MC(2,:) + xbar(2,:) ), 'color', 0.6 * ones(3, 1));

end