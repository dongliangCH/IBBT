

function drawResult(xbar, PPtilde, ellipse_steps)

rl = 0.9545; % 2 Sigma   
hold on;

for k = ellipse_steps

%     % Total covariance
%     pl1 = error_ellipse('C', PP(1:2, 1:2, k + 1), 'mu', xbar(1:2, k + 1), ...
%         'conf', rl, 'style', 'k', 'linewidth', 0.567);
       
    % State error covariance
    pl3 = error_ellipse('C', PPtilde(1:2, 1:2, k + 1), 'mu', ...
        xbar(1:2, k + 1), ...
        'conf', rl, 'style', 'k:', 'linewidth', 0.567);
    set(pl3, 'Color', 0.2 *  ones(3, 1))
    
end

% lh = legend([pl1, pl3], '$P_k$', '$\tilde{P}_k$', 'location', 'southwest');
% set(lh, 'Interpreter', 'latex');
% hold off;
% grid on;
% axis equal;
x1 = xlabel('$x_1$', 'interpreter', 'latex');
y1 = ylabel('$x_2$', 'interpreter', 'latex');
set(x1,'FontSize',18);
set(y1,'FontSize',18);
set(gca,'FontSize',16,'FontName','Times');

end