function [x0_mc, Pe_0, CovCost, CollisonProb, K] = propagate(P0, PtildePrior0, param, Traj, world)

% Notation: Ak is single step of matrix, AA is 3d array with each Ak
% stacked in the 3rd dimension, A is block matrix

%% Problem setup
xbar = Traj(1:3,:);
U = Traj(4,:);

% Number of steps N
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
Qk = 1 * blkdiag(1, 1, 1);
Rk = 0.2 * blkdiag(1);

Gk = sqrt(dt) * diag([0.08 0.08 0.05]);  
GG = repmat(Gk, [1, 1, N]);

% %% Kalman filter
% Ck = eye(3); 
% CC = repmat(Ck, [1, 1, N + 1]);
% Dk = zeros(ny, ny);
% DD = repmat(Dk, [1, 1, N + 1]);
% 
% Xbar_temp = xbar * world.scale;
% for i = 1 : N + 1
%     if world.sensor_map(ceil(Xbar_temp(1, i)), ceil(Xbar_temp(2, i))) %|| world.sensor_map(floor(Xbar_temp(1+4*(i-1))), floor(Xbar_temp(2+4*(i-1))))
%         DD(:, :, i) = 0.1 * eye(3);      
%     else
%         DD(:, :, i) = 2 * eye(3);
%     end
% end
% 
% % Initial state covariance
% PhatPrior0 = P0 - PtildePrior0;
% 
% % Initial estimation error covariance  PtildePrior0
% % Initial estimated state covariance  PhatPrior0 
% 
% % Solve for sequence of kalmain gains LL, error covariances PPtilde, and prior
% % error covariances PPtm
% [LL, PPtilde, PPtm] = solveKF(AA, GG, CC, DD, N, PtildePrior0);

%% Time-varying LQR tracking

S(:,:,N+1) = Qk;
for k = N:-1:1
    
    K(:,:,k) = (BB(:,:,k)' * S(:,:,k+1) * BB(:,:,k) + Rk) \ BB(:,:,k)' * S(:,:,k+1) * AA(:,:,k);
    S(:,:,k) = Qk + AA(:,:,k)' * S(:,:,k+1) * AA(:,:,k) - AA(:,:,k)' * S(:,:,k+1) * BB(:,:,k) * K(:,:,k);

end

x0_mc = [];
%% Monte Carlo
collision = 0;
P0 = P0';

mu = [0.5 0.5 0; -0.5 -0.5 0];
sigma = cat(3, diag([0.06, 0.04, 0.02]), diag([0.04, 0.06, 0.04]));
p  = [0.5 0.5];
gm = gmdistribution(mu,sigma,p);
W = random(gm, N);
for mc = 1:size(P0,2)
    x_MC = zeros(nx, N+1);
    x_MC(:,1) = P0(:, mc);
%     W = random(gm, N);
    % W = randn(nw, N);
    for i=1:N
        w = W(i,:)';
        u = U(i) - K(:,:,i)*(x_MC(:,i)-xbar(:,i));
        x_MC(:,i+1) = x_MC(:,i) + [cos(x_MC(3,i)); sin(x_MC(3,i)); u]*dt + Gk*w;
    end

%     hold on
%     plot( x_MC(1,:), x_MC(2,:), 'color', 0.6 * ones(3, 1));    
    if MeanCollisionCheck(x_MC(1:2,:), world, 2)
        collision = collision + 1;
    else
        x0_mc = [x0_mc x_MC(:,end)];
    end
    if collision > 10
        break
    end
end
% plot( xbar(1,:), xbar(2,:), 'color', 'r');
CollisonProb = 1-size(x0_mc,2)/100;
x0_mc = x0_mc';
Pe_0 = cov(x0_mc);
CovCost = trace(Pe_0);

end

