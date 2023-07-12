function [endP0, endPtilde, CovCost, CollisonProb, K] = propagate(P0, PtildePrior0, N, param, xbar, world)

% Notation: Ak is single step of matrix, AA is 3d array with each Ak
% stacked in the 3rd dimension, A is block matrix

%% Problem setup

% Number of steps N
% Choose time step dt = 0.2

% Dynamics
dt = param.dt;
ny = 4;
Ak = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
Bk = [dt ^ 2 / 2 0; 0 dt ^ 2 / 2; dt 0; 0 dt];
Gk = sqrt(dt) * diag([0.02 0.02 0.01 0.01]);

% Cost
Qk = 2 * diag([2 2 2 2]);
Rk = 1 * diag([2 2]);


%% Kalman filter
% Observation model

Ck = eye(4); 
DD = zeros(ny, ny, N + 1);

xbar_temp = xbar * world.scale;
for i = 1 : N + 1
    if world.sensor_map(ceil(xbar_temp(1,i)), ceil(xbar_temp(2,i))) %|| world.sensor_map(floor(xbar_temp(1,i)), floor(xbar_temp(2,i)))
        Dk = 0.05 * eye(4);
    else
        Dk = 2 * eye(4);
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
%     Fk = [Ak                               -Bk * K(:,:,k);
%           LL(:,:,k+1) * Ck * Ak   Ak - Bk * K(:,:,k) - LL(:,:,k+1) * Ck *Ak];
%     [row, ~] = size(Gk);
%     [~, col] = size(LL(:,:,k+1) * DD(:,:,k+1));
%     Ek = [Gk                               zeros(row, col);
%           LL(:,:,k+1) * Ck * Gk   LL(:,:,k+1) * DD(:,:,k+1)];
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
PP(:,:,1) = P0(1:2,1:2);
Jc = 0;
for k = 1:N
    PPhat(:,:,k+1) = (Ak - Bk * K(:,:,k)) * PPhat(:,:,k) * (Ak - Bk * K(:,:,k))' + LL(:,:,k+1) * Ck * PPtm(:,:,k+1);
    PP(:,:,k+1) = PPhat(1:2,1:2,k+1) + PPtilde(1:2,1:2,k+1);
end

endP0 = PPhat (:, :, end) + PPtilde(:, :,end);
endPtilde = PPtilde(:, :,end);

CovCost = dt * Jc;


    %% Monte Carlo
    % Number of monte carlo trials to display (can be zero)
    MCnum = 100;

%     figure(10); 
%     hold on
%     CollisonProb = montecarlo(MCnum, Ak, Bk, Gk, Ck, DD, LL, xbar, ...
%         N, PhatPrior0, PtildePrior0, K, world);
CollisonProb = montecarlo1(MCnum, xbar(1:2,:), PP, N, world, param);

    %% Plot result

%     % Step numbers to draw covariance ellipses at
%     ellipse_steps = 0:N:N;
% 
%     figure(1);
%     drawResult(xbar, PP, PPtilde, ellipse_steps)

end

