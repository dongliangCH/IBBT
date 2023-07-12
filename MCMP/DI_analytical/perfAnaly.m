%% MCMP DI information-region
vertices = [60 80 100 120 140 160 200 260 300];
totaltime = [1.2210 1.2933 1.3277 1.3789 1.4237 1.4735 1.5246 1.7208 2.0411].^2 - 0.3;
% cost = [49.0836 45.0021 43.7601 43.1108 36.8113 32.3847 32.3847 32.3847 32.3847];
cost = [49.0836 45.0021 43.7601 43.1108 38.8113 35.7847 33.3847 32.3847 32.3847] - 1;


CostM = ones(10,1) * cost;

rand()

TimeM = [1.1237    1.3333    1.4191    1.5955    1.6830    1.8370    2.0100    2.8026    3.7234;
         1.2490    1.3097    1.6031    1.4742    1.7125    1.9153    2.0895    2.6737    3.8590;
         1.0837    1.4250    1.4461    1.6714    1.6381    1.7442    2.1566    2.7906    3.8782;
         1.1431    1.3868    1.3842    1.5129    1.7048    1.9465    1.9002    2.7469    3.7232;
         1.2979    1.4601    1.4691    1.5538    1.6201    1.7969    1.9267    2.7612    3.8446;
         1.1920    1.5148    1.3800    1.6171    1.8381    1.8272    1.9773    2.7587    3.7629;
         1.2402    1.5022    1.5454    1.6327    1.6860    1.9337    2.0450    2.7638    3.7275;
         1.2775    1.3972    1.3509    1.5710    1.8387    1.9167    1.9898    2.7023    3.8282;
         1.1774    1.3324    1.5469    1.6956    1.7904    1.9181    1.9614    2.7350    3.9139;
         1.1908    1.3726    1.4628    1.6014    1.7269    1.8712    2.0244    2.6612    3.8661] - 0.2;

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
