function [endP0, endPtilde, CovCost, CollisonProb, K] = propagate(P0, PtildePrior0, N, param, xbar, world)

% Notation: Ak is single step of matrix, AA is 3d array with each Ak
% stacked in the 3rd dimension, A is block matrix

%% Problem setup

% Number of steps N
% Choose time step dt = 0.2

% Dynamics
dt = param.dt;
ny = 2;
Ak = [1 0; 0 1];
Bk = [1 0; 0 1];
Gk = sqrt(dt) * diag([0.06 0.06]);

% Cost
Qk = 2 * blkdiag(2, 2);
Rk = 1 * blkdiag(2, 2);


%% Kalman filter
% Observation model

Ck = eye(2); 
CC = repmat(Ck, [1, 1, N + 1]);
DD = zeros(ny, ny, N + 1);

xbar_temp = xbar * world.scale;
for i = 1 : N + 1
    if world.sensor_map(ceil(xbar_temp(1,i)), ceil(xbar_temp(2,i))) %|| world.sensor_map(floor(xbar_temp(1,i)), floor(xbar_temp(2,i)))
        Dk = 0.2 * eye(2);
    else
        Dk = 4 * eye(2);
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

PPhat(:,:,1) = P0 - PPtilde(:,:,1);
PP(:,:,1) = P0(1:2,1:2);
Jc = 0;
for k = 1:N
    PPhat(:,:,k+1) = (Ak - Bk * K(:,:,k)) * PPhat(:,:,k) * (Ak - Bk * K(:,:,k))' + LL(:,:,k+1) * Ck * PPtm(:,:,k+1);
    PP(:,:,k+1) = PPhat(1:2,1:2,k+1) + PPtilde(1:2,1:2,k+1);
    PP_all = PPhat(:,:,k+1) + PPtilde(:,:,k+1);
    Jc = Jc + trace( PP_all * (Qk + K(:,:,k)' * Rk * K(:,:,k)) );
end

endP0 = PPhat (:, :, end) + PPtilde(:, :,end);
endPtilde = PPtilde(:, :,end);

CovCost = dt * Jc;


%% Monte Carlo
MCnum = 50;

% CollisonProb = montecarlo(MCnum, Ak, Bk, Gk, Ck, DD, LL, xbar, ...
%     N, PhatPrior0, PtildePrior0, K, world);
CollisonProb = montecarlo1(MCnum, xbar(1:2,:), PP, N, world, param);

%% Plot result

% % Step numbers to draw covariance ellipses at
% ellipse_steps = 0:N:N;
% 
% figure(1);
% drawResult(xbar, PP, PPtilde, ellipse_steps)

end

