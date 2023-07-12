function [endP0, endPtilde, CovCost, CollisonProb, K] = propagate1(P0, PtildePrior0, N, param, Xbar, V, world)

% Notation: Ak is single step of matrix, AA is 3d array with each Ak
% stacked in the 3rd dimension, A is block matrix

%% Problem setup

% Number of steps N
% Choose time step dt = 0.2

% Dynamics
dt = param.dt;
nx = 4;
nu = 2;
ny = 4;
nw = 4;
Ak = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
Bk = [dt ^ 2 / 2 0; 0 dt ^ 2 / 2; dt 0; 0 dt];
Gk = sqrt(dt) * diag([0.03 0.03 0.02 0.02]);

% Sequences of system matricies
AA = repmat(Ak, [1, 1, N]);
BB = repmat(Bk, [1, 1, N]);
GG = repmat(Gk, [1, 1, N]);

% Make block matricies for dynamics
A = makeStateMatrix(AA, N);
B = makeInputMatrix(AA, BB, N);

% Cost
Qk = 2 * blkdiag(2, 2, 2, 2);
Rk = 1 * blkdiag(2, 2);

% Make block matricies for cost
Q = [];
R = [];
for i = 1:N
    Q = blkdiag(Q, Qk);
    R = blkdiag(R, Rk);
end
Q = blkdiag(Q, zeros(nx));


%% Kalman filter
% Observation model

Ck = eye(4); 
CC = repmat(Ck, [1, 1, N + 1]);
Dk = zeros(ny, ny);
DD = repmat(Dk, [1, 1, N + 1]);


for i = 1 : N + 1
    
    if Xbar(2+4*(i-1))<3
        Dk = 0.01 * eye(4);
    else
        Dk = 0.9 * eye(4);
    end

    DD(:, :, i) = Dk;
    
end

% Initial state covariance
PhatPrior0 = P0 - PtildePrior0;

% Initial estimation error covariance  PtildePrior0
% Initial estimated state covariance  PhatPrior0 

% Solve for sequence of kalmain gains LL, error covariances PPtilde, and prior
% error covariances PPtm
[LL, PPtilde, ~] = solveKF(AA, GG, CC, DD, N, PtildePrior0);

%% Time-varying LQR tracking

S(:,:,N+1) = Qk;
for k = N:-1:1
    
    K(:,:,k) = (BB(:,:,k)' * S(:,:,k+1) * BB(:,:,k) + Rk) \ BB(:,:,k)' * S(:,:,k+1) * AA(:,:,k);
    S(:,:,k) = Qk + AA(:,:,k)' * S(:,:,k+1) * AA(:,:,k) - AA(:,:,k)' * S(:,:,k+1) * BB(:,:,k) * K(:,:,k);

end

%% State covariance and estimated state covariance

PP(:,:,1)= blkdiag(P0, P0 - PPtilde(:,:,1));
TrPP = zeros(1, N+1);
TrPP(1) = trace(PP(1:4, 1:4, 1));
% TVLQG covariance control cost
Jc = 0;

for k = 1:N
    
    Fk = [AA(:,:,k)                               -BB(:,:,k) * K(:,:,k);
          LL(:,:,k+1) * CC(:,:,k+1) * AA(:,:,k)   AA(:,:,k) - BB(:,:,k) * K(:,:,k) - LL(:,:,k+1) * CC(:,:,k+1) *AA(:,:,k)];
    [row, ~] = size(GG(:,:,k));
    [~, col] = size(LL(:,:,k+1) * DD(:,:,k+1));
    Ek = [GG(:,:,k)                               zeros(row, col);
          LL(:,:,k+1) * CC(:,:,k+1) * GG(:,:,k)   LL(:,:,k+1) * DD(:,:,k+1)];
    PP(:,:,k+1) = Fk * PP(:,:,k) * Fk' + Ek * Ek';
    TrPP(k+1) = trace(PP(1:4, 1:4, k+1));
    Jc = Jc + trace( PP(5:8, 5:8, k) * (Qk + K(:,:,k)' * Rk * K(:,:,k)) );
    
end


endP0 = PP(1:4, 1:4, end);
endPtilde = PPtilde(1:4, 1:4,end);

CovCost = dt * Jc;

    xbar = zeros(nx, N + 1);
    for k = 0:N
        Ek = [zeros(nx, k * nx) eye(nx) zeros(nx,(N - k) * nx)];
        xbar(:, k + 1) = Ek * Xbar;
    end

    %% Monte Carlo
    % Number of monte carlo trials to display (can be zero)
    MCnum = 20;

    figure(10); 
    hold on
    CollisonProb = montecarlo(MCnum, AA, BB, GG, CC, DD, LL, xbar, ...
        N, PhatPrior0, PtildePrior0, V, K, world);

    %% Plot result

    % Step numbers to draw covariance ellipses at
    ellipse_steps = 0:N:N;

    figure(10);
    drawResult(xbar, PP, PPtilde, ellipse_steps)

end

