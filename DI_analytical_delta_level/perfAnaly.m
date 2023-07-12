CostM = ones(10,1) * [38.3315   38.3315   33.4052   33.4052   29.1781   28.0872   28.0872   28.0872   28.0872   28.0872   27.3222];
TimeM = [0.1145    0.1429    0.2946    0.3800    0.5806    0.7883    0.9340    1.1225    1.3225    1.5226    1.8499;
         0.1410    0.1796    0.3396    0.4429    0.6610    0.8818    1.0272    1.2033    1.4022    1.6028    1.9054;
         0.1215    0.1526    0.2925    0.3798    0.5963    0.8014    0.9465    1.1222    1.3131    1.5116    1.8188;
         0.1459    0.1782    0.3332    0.4263    0.6411    0.8642    1.0185    1.1984    1.4052    1.6107    1.9239;
         0.1104    0.1358    0.2703    0.3550    0.5523    0.7570    0.8984    1.0792    1.2721    1.4667    1.7867;
         0.1040    0.1298    0.2645    0.3496    0.5630    0.7696    0.9172    1.0933    1.2908    1.4893    1.7948;
         0.1255    0.1525    0.3039    0.3891    0.5918    0.8024    0.9508    1.1566    1.3531    1.5478    1.8554;
         0.1048    0.1304    0.2699    0.3591    0.5549    0.7605    0.9023    1.0804    1.2688    1.4624    1.7614;
         0.0983    0.1218    0.2582    0.3432    0.5442    0.7549    0.9058    1.0862    1.2785    1.4780    1.7854;
         0.1026    0.1265    0.2640    0.3493    0.5484    0.7568    0.9002    1.0777    1.2691    1.4672    1.7794];

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
