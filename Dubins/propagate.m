function [endP0, endPtilde, CovCost, CollisonProb, K] = propagate(P0, PtildePrior0, param, xbar, world)

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
PP(:,:,1) = P0(1:2,1:2);
Jc = 0;
for k = 1:N
    PPhat(:,:,k+1) = (AA(:,:,k) - BB(:,:,k) * K(:,:,k)) * PPhat(:,:,k) * (AA(:,:,k) - BB(:,:,k) * K(:,:,k))' + LL(:,:,k+1) * CC(:,:,k+1) * PPtm(:,:,k+1);
    PP(:,:,k+1) = PPhat(1:2,1:2,k+1) + PPtilde(1:2,1:2,k+1);
    PP_all = PPhat(:,:,k+1) + PPtilde(:,:,k+1);
    Jc = Jc + trace( PP_all * (Qk + K(:,:,k)' * Rk * K(:,:,k)) );
end

endP0 = PPhat (:, :, end) + PPtilde(:, :,end);
endPtilde = PPtilde(:, :,end);

CovCost = dt * Jc;

%% Monte Carlo
% Number of monte carlo trials to display (can be zero)
MCnum = 40;

% figure(10); 
% hold on
% CollisonProb = montecarlo(MCnum, AA, BB, GG, CC, DD, LL, xbar, ...
%     N, PhatPrior0, PtildePrior0, K, world);
CollisonProb = montecarlo1(MCnum, xbar(1:2,:), PP, N, world, param);

%% Plot result

% % Step numbers to draw covariance ellipses at
% ellipse_steps = 0:N:N;
% 
% figure(2);
% drawResult(xbar, PPtilde, ellipse_steps)

end

