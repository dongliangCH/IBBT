function [xbar, length, N] = meanControl(startp, endp, param)

length = norm (endp - startp);

velavg = param.velavg;

% Final time
tf = length/velavg;

% Number of steps
dt = param.dt;
N = ceil (tf / dt);
N = max(4, N);   % at least 4 step, for matrix inverse

xbar = startp + (0:N).*(endp-startp)/N;

end