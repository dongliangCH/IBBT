% CostM = [29.4132   21.5072   20.9625   20.9625   20.1160];
% TimeM = [0.0643    0.2592    0.5594    0.9033    1.3401;
%          0.0618    0.2539    0.5565    0.9038    1.3514;
%          0.0624    0.2547    0.5562    0.9033    1.3399;
%          0.0630    0.2565    0.5600    0.9090    1.3534;
%          0.0634    0.2589    0.5654    0.9182    1.3657;
%          0.0626    0.2555    0.5568    0.9045    1.3460;
%          0.0661    0.2628    0.5692    0.9188    1.3608;
%          0.0641    0.2607    0.5652    0.9162    1.3627;
%          0.0625    0.2541    0.5585    0.9081    1.3495;
%          0.0640    0.2599    0.5667    0.9188    1.3639];
CostM = ones(10,1) * [29.4132   21.5072   20.9625   20.9625   20.9625   20.9459   20.4016   20.1160   20.0038];
TimeM = [0.0604    0.2530    0.5478    0.8230    1.1430    1.5231    1.9301    2.3012    2.7494;
         0.0612    0.2557    0.5478    0.8093    1.1199    1.4833    1.8789    2.2428    2.6745;
         0.0629    0.2572    0.5508    0.8181    1.1352    1.5110    1.9146    2.2856    2.7266;
         0.0604    0.2555    0.5468    0.8137    1.1423    1.5265    1.9305    2.3010    2.7414;
         0.0608    0.2533    0.5452    0.8125    1.1308    1.5030    1.9072    2.2754    2.7138;
         0.0612    0.2556    0.5510    0.8159    1.1302    1.5006    1.9023    2.2721    2.7133;
         0.0611    0.2579    0.5539    0.8241    1.1416    1.5127    1.9180    2.2885    2.7269;
         0.0610    0.2546    0.5479    0.8146    1.1306    1.5072    1.9124    2.2834    2.7253;
         0.0617    0.2560    0.5462    0.8091    1.1198    1.4853    1.8880    2.2532    2.6882] + 0.16;

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
