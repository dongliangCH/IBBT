function [collisionProb, CovCost] = MCCollisionProb(Vertices, P0, PtildePrior0, Path_idx, param, world, plot_flag)

MCnum = 500;
Xbar = [Vertices(Path_idx(1, 1), :)'];
Step = 0;
for i = 1:size(Path_idx, 2)-1        
    [meanTraj, ~] = dubins_curve(Vertices(Path_idx(1, i), :)', Vertices(Path_idx(1, i+1), :)', param.radi, param.dt, 1);
    Xbar = [Xbar, meanTraj(:, 2:end)];
    N = size(meanTraj, 2) - 1;
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
nx = 3;
nu = 1;
ny = 3;

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
Qk = 0.2 * diag([1, 1, 1]);
Rk = 0.1;

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

% Initial estimation error covariance  PtildePrior0
% Initial estimated state covariance  PhatPrior0 
PhatPrior0 = P0 - PtildePrior0;

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
PPhat(:,:,1) = P0 - PPtilde(:,:,1);
PP(:,:,1) = P0(1:2,1:2);
Jc = 0;
for k = 1:N
    PPhat(:,:,k+1) = (AA(:,:,k) - BB(:,:,k) * K(:,:,k)) * PPhat(:,:,k) * (AA(:,:,k) - BB(:,:,k) * K(:,:,k))' + LL(:,:,k+1) * CC(:,:,k+1) * PPtm(:,:,k+1);
    PP(:,:,k+1) = PPhat(1:2,1:2,k+1) + PPtilde(1:2,1:2,k+1);
    PP_all = PPhat(:,:,k+1) + PPtilde(:,:,k+1);
    Jc = Jc + trace( PP_all * (Qk + K(:,:,k)' * Rk * K(:,:,k)) );
end

CovCost = dt * Jc;

%% Monte Carlo Simulation
collisionProb = mcSimulation(MCnum, AA, BB, GG, CC, DD, LL, xbar, N, PhatPrior0, PtildePrior0, K, world, plot_flag);

%% Plot result
% % Step numbers to draw covariance ellipses at
% ellipse_steps = 0:2:N;
% drawResult(xbar, PP, PPtilde, ellipse_steps)

end

function collisionProb = mcSimulation(MCnum, AA, BB, GG, CC, DD, LL, xbar, N, PhatPrior0, PtildePrior0, K, world, plot_flag)

% Sizes of inputs
nx = size(AA, 1);
nu = size(BB, 2);
ny = size(CC, 1);
nw = size(GG, 2);
nv = ny;

rng(0);
collision = 0;

for mc = 1:MCnum

    xhatPrior0_MC = mvnrnd(xbar(:,1) - xbar(:,1), PhatPrior0 + 0.01 * eye(3), 1)';
    xtildePrior0 = mvnrnd(zeros(nx,1), PtildePrior0, 1)';
    x0_MC = xhatPrior0_MC + xtildePrior0;

    U = zeros(nu,N);
    x_MC = zeros(nx,N+1);
    xhat_MC = zeros(nx,N+1);
    xhatPrior_MC = zeros(nx,N+1);
    x_MC(:,1) = x0_MC;
    
    for k = 1:(N + 1)
        
        v = randn(nv, 1);
        w = randn(nw, 1);
    
        % Measurement
        y_MC = CC(:, :, k) * x_MC(:, k) + DD(:, :, k) * v;
    
        % Prior estimate
        if k == 1
            xhatPrior_MC(:,k) = xhatPrior0_MC;
        else
            xhatPrior_MC(:, k) = AA(:, :, k - 1) * xhat_MC(:, k - 1) ...
                + BB(:, :, k - 1) * U(:, k - 1);
        end
    
        xhat_MC(:,k) =  xhatPrior_MC(:,k) + LL(:,:,k) * (y_MC - CC(:,:,k) * xhatPrior_MC(:,k));
        
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