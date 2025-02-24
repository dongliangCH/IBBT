function [endP0, endPtilde, CovCost, CollisonProb, K] = propagate(P0, PtildePrior0, N, param, xbar, world)

% Notation: Ak is single step of matrix, AA is 3d array with each Ak
% stacked in the 3rd dimension, A is block matrix

%% Problem setup

% Number of steps N
% Choose time step dt = 0.2

% Dynamics
dt = param.dt;
Ak = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
Bk = [dt ^ 2 / 2 0; 0 dt ^ 2 / 2; dt 0; 0 dt];
Gk = 1 * sqrt(dt) * diag([0.04 0.04 0.02 0.02]);

% Cost
Qk = 2 * diag([2, 2, 2, 2]);
Rk = 1 * diag([2, 2]);


%% Kalman filter
% Observation model
% Ck = eye(4); 
% DD = zeros(ny, ny, N + 1);
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
        Dk = diag([1 1]);
    end
    CC(i).Ck = Ck;
    DD(i).Dk = Dk;
end

% Initial state covariance
PhatPrior0 = P0 - PtildePrior0;

% Initial estimation error covariance  PtildePrior0
% Initial estimated state covariance  PhatPrior0 

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
% figure(1);
% drawResult(xbar, PP, PPtilde, ellipse_steps)

end

