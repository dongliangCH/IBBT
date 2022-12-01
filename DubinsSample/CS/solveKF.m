% SOLVEKF Solve Kalman Filter equations
% 
% Inputs:
%   AA      State matrix:         	size (nx, nx, N),     A_k = AA(:, :, k)
%   GG      Process noise matrix:   size (nx, nx, N),     G_k = GG(:, :, k)
%   CC      Observation matrix:     size (nx, nx, N + 1), C_k = CC(:, :, k)
%   DD      Obs. noise matrix:      size (nx, nx, N + 1), D_k = DD(:, :, k)
%   N       Step horizon
%   PtildePrior0    Prior error covariance, size (nx, nx)
%
% Outputs:
%   LL      Kalman gain matrix:     size (nx, ny, N + 1), L_k = LL(:, :, k)
%   PPt     Error cov. matrix:      size (nx, nx, N + 1), PPt_k = PPt(:, :, k)
%   PPtm    Error cov. prior:       size (nx, nx, N + 1), PPtm_k = PPtm(:, :, k)

function [LL, PPt, PPtm] = solveKF(AA, GG, CC, DD, N, PtildePrior0)

nx = size(AA, 1);
ny = size(CC, 1);

PPt = zeros(nx, nx, N + 1);
PPtm = zeros(nx, nx, N + 1);
LL = zeros(nx, ny, N + 1);

for k = 1:(N + 1)
    
    % Update prior covariance
    if k == 1
        PPtm(:, :, 1) = PtildePrior0;
    else
        PPtm(:, :, k) = AA(:, :, k - 1) * PPt(:, :, k - 1) ...
            * AA(:, :, k - 1)' + GG(:, :, k - 1) * GG(:, :, k - 1)';
    end
    
    % Kalman gain
    LL(:, :, k) = PPtm(:, :, k) * CC(:, :, k)' ...
        * inv(CC(:, :, k) * PPtm(:, :, k) * CC(:, :, k)' ...
        + DD(:, :, k) * DD(:, :, k)');
    
    % Covariance after measurement
    PPt(:, :, k) = PPtm(:, :, k) - LL(:, :, k) ...
        * CC(:, :, k) * PPtm(:, :, k);
    
%     % Covariance after measurement
%     PPt(:, :, k) = PPtm(:, :, k) - LL(:, :, k) ...
%         * (CC(:, :, k) * PPtm(:, :, k) * CC(:, :, k)' + ...
%         DD(:, :, k) * DD(:, :, k)') * LL(:, :, k)';
    
end
