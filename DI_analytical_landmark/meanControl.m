function [xbar, Mcost, N] = meanControl(xbar0, xbarf, param)

% Final time
% Choose average velocity
velavg = param.velavg;
tf = norm (xbar0 - xbarf)/velavg;

% Number of steps
% Choose time step dt = 0.2
dt = param.dt;
N = ceil (tf / dt);
N = max(4, N);   % at least 4 step, for matrix inverse

% Cost
Mcost = DI_cost(tf,xbar0(1),xbar0(2),xbar0(3),xbar0(4),xbarf(1),xbarf(2),xbarf(3),xbarf(4));

% Trajectory
states = @(t)DI_state(t,tf,xbar0(1),xbar0(2),xbar0(3),xbar0(4),xbarf(1),xbarf(2),xbarf(3),xbarf(4));
time = linspace(0, tf, N + 1);
xbar(:,1) = xbar0;
for i = 2 : N
    xbar(:, i) = states(time(i));
end
xbar(:,N+1) = xbarf;

end

function cost =  DI_cost(t_s,x01,x02,x03,x04,x11,x12,x13,x14)
cost = t_s + (4*x03^2)/t_s + (12*x01^2)/t_s^3 + (4*x04^2)/t_s + (12*x02^2)/t_s^3 +...
    (4*x13^2)/t_s + (12*x11^2)/t_s^3 + (4*x14^2)/t_s + (12*x12^2)/t_s^3 +...
    (12*x01*x03)/t_s^2 + (12*x02*x04)/t_s^2 - (24*x01*x11)/t_s^3 + (12*x01*x13)/t_s^2 -...
    (12*x03*x11)/t_s^2 + (4*x03*x13)/t_s - (24*x02*x12)/t_s^3 + (12*x02*x14)/t_s^2 -...
    (12*x04*x12)/t_s^2 + (4*x04*x14)/t_s - (12*x11*x13)/t_s^2 - (12*x12*x14)/t_s^2;
end

function states = DI_state(t,t_s,x01,x02,x03,x04,x11,x12,x13,x14)
 states = [x11 + x13*(t - t_s) + ((t - t_s)^3*(2*x01 - 2*x11 + t_s*x03 + t_s*x13))/t_s^3 + ((t - t_s)^2*(3*x01 - 3*x11 + t_s*x03 + 2*t_s*x13))/t_s^2;
           x12 + x14*(t - t_s) + ((t - t_s)^3*(2*x02 - 2*x12 + t_s*x04 + t_s*x14))/t_s^3 + ((t - t_s)^2*(3*x02 - 3*x12 + t_s*x04 + 2*t_s*x14))/t_s^2;
           x13 + (2*(t - t_s)*(3*x01 - 3*x11 + t_s*x03 + 2*t_s*x13))/t_s^2 + (3*(t - t_s)^2*(2*x01 - 2*x11 + t_s*x03 + t_s*x13))/t_s^3;
           x14 + (2*(t - t_s)*(3*x02 - 3*x12 + t_s*x04 + 2*t_s*x14))/t_s^2 + (3*(t - t_s)^2*(2*x02 - 2*x12 + t_s*x04 + t_s*x14))/t_s^3];
end