function [xbar, Mcost, N] = meanControl(startp, endp, param)

% Initial mean
xbar0 = startp;

% Target mean
xbarf = endp;

% Final time
% Choose average velocity
velavg = param.velavg;
tf = norm (xbar0 - xbarf)/velavg;

% Number of steps
% Choose time step dt = 0.2
dt = param.dt;
N = ceil (tf / dt);
N = max(4, N);   % at least 4 step, for matrix inverse

% Dynamics
nx = 4;
nu = 2;
nw = 4;
Ak = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
Bk = [dt ^ 2 / 2 0; 0 dt ^ 2 / 2; dt 0; 0 dt];

% Cost
Qk = 2 * blkdiag(2, 2, 2, 2);
Rk = 1 * blkdiag(2, 2);

% Sequences of system matricies
AA = repmat(Ak, [1, 1, N]);
BB = repmat(Bk, [1, 1, N]);

% Make block matricies for cost
Q = [];
R = [];
for i = 1:N
    Q = blkdiag(Q, Qk);
    R = blkdiag(R, Rk);
end
Q = blkdiag(Q, zeros(nx));

% Make block matricies for dynamics
A = makeStateMatrix(AA, N);
B = makeInputMatrix(AA, BB, N);

% Mean control
Qm = 0 * Q;
M = B' * Qm * B + R;
Minv = inv(M);
AN = A((nx * N + 1) : nx * (N + 1), :);
BN = B((nx * N + 1) : nx * (N + 1), :);
TempM = B' * Qm * A * xbar0;
V = Minv * ( - TempM + BN' * inv(BN * Minv * BN') * (xbarf - AN * xbar0 + BN * Minv * TempM));

% Mean trajectory
Xbar = A * xbar0 + B * V;

% Cost for mean control
Mcost = dt * (Xbar' * Qm * Xbar + V' * R * V);

for k = 0:N
    Ek = [zeros(nx, k * nx) eye(nx) zeros(nx,(N - k) * nx)];
    xbar(:, k + 1) = Ek * Xbar;
end

end