figure(4)
errorbar(mean(TimeM(:, :)), mean(CostM(:, :)), sqrt(0.9) * std(CostM(:, :)), ...
    sqrt(0.9) * std(CostM(:, :)), sqrt(0.9) * std(TimeM(:, :)), sqrt(0.9) * std(TimeM(:, :)), 'LineWidth', 1); hold on

% xl = xlabel('$Time \ (s)$','Interpreter','LaTeX');
% yl = ylabel('$Cost$','Interpreter','LaTeX');
xl = xlabel('Time (s)');
yl = ylabel('Cost');
set(xl,'FontSize',18);
set(yl,'FontSize',18);
set(gca,'FontSize',16,'FontName','Times');

% figure(1)
% for i = 1:10
%     plot(timeM(i,1:14),costM(i,1:14),'o--','LineWidth',1); hold on
% end
% xl = xlabel('$Time \ (s)$','Interpreter','LaTeX');
% yl = ylabel('$Cost$','Interpreter','LaTeX');
% set(xl,'FontSize',18);
% set(yl,'FontSize',18);
% set(gca,'FontSize',16,'FontName','Times');
% 
% figure(2)
% for i = 1:10
%     plot(ItsM(i,1:14),timeM(i,1:14),'o-','LineWidth',1); hold on
% end
% xl = xlabel('$Nodes$','Interpreter','LaTeX');
% yl = ylabel('$Time$','Interpreter','LaTeX');
% set(xl,'FontSize',18);
% set(yl,'FontSize',18);
% set(gca,'FontSize',16,'FontName','Times');
% 
% figure(3)
% for i = 1:10
%     plot(ItsM(i,1:14),costM(i,1:14),'o-','LineWidth',1); hold on
% end
% xl = xlabel('$Nodes$','Interpreter','LaTeX');
% yl = ylabel('$Cost$','Interpreter','LaTeX');
% set(xl,'FontSize',18);
% set(yl,'FontSize',18);
% set(gca,'FontSize',16,'FontName','Times');

% figure(5)
% errorbar(mean(ItsM(:, 2:14)), mean(costM(:, 2:14)), sqrt(0.9) * std(costM(:, 2:14)), 'LineWidth', 1); hold on
% xl = xlabel('$Nodes$','Interpreter','LaTeX');
% yl = ylabel('$Cost$','Interpreter','LaTeX');
% set(xl,'FontSize',18);
% set(yl,'FontSize',18);
% set(gca,'FontSize',16,'FontName','Times');
