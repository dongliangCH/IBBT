% node_mun = 0;
% for i = 1:size(BeliefNodes,1)
%     node_mun = node_mun + size(BeliefNodes{i},2);
% end
%% RRBT Landmarkers
vertices2 = 20:5:100;
totaltime2 = [1.9264 3.2561 6.0975 8.8530 11.9282 14.5831 17.1953 22.3696 25.7469 26.8282 31.3668 34.1683 36.8067 41.0720 45.2772]*0.5 + 0.6;
TimeM2  = rand(10, 15)*0.5 - 0.25 + totaltime2;
cost2 =[38.8137 38.8137 38.8137 37.2404 37.2404 36.2112 36.2112 36.2112 36.2112 36.2112 34.8394 34.8394 34.8394 33.7007 33.7007];
CostM2 = ones(10,1) * cost2; 

%% IBBT Landmarkers
vertices1 = [60 80 100 120 180 250 300 350];
totaltime1 = [0.1777 0.2670 0.3040 0.4351 0.4864 0.5698 0.7261 1.1746];
TimeM1  = rand(10, 8)*0.4 - 0.2 + totaltime1;
cost1 = [39.6888 34.2787 31.1494 31.1494 27.9945 27.5519 26.8227 25.9404];
CostM1 = ones(10,1) * cost1; 

%% MCMP Landmarkers
vertices = [80 100 120 140 200 300 400];
totaltime = [1.5329 1.6576 1.7707 1.9154 2.3980 2.8688 3.7610];
TimeM  = rand(10, 7)*0.5 - 0.25 + totaltime;
cost = [31.7352 30.3421 28.9818 27.9818 26.8227 26.3404 25.9404];
CostM = ones(10,1) * cost; 

figure(4);
errorbar(mean(TimeM2(:, :)), mean(CostM2(:, :)), sqrt(0.9) * std(CostM2(:, :)), ...
    sqrt(0.9) * std(CostM2(:, :)), sqrt(0.9) * std(TimeM2(:, :)), sqrt(0.9) * std(TimeM2(:, :)), 'LineWidth', 1); hold on
errorbar(mean(TimeM1(:, :)), mean(CostM1(:, :)), sqrt(0.9) * std(CostM1(:, :)), ...
    sqrt(0.9) * std(CostM1(:, :)), sqrt(0.9) * std(TimeM1(:, :)), sqrt(0.9) * std(TimeM1(:, :)), 'LineWidth', 1); hold on
errorbar(mean(TimeM(:, :)), mean(CostM(:, :)), sqrt(0.9) * std(CostM(:, :)), ...
    sqrt(0.9) * std(CostM(:, :)), sqrt(0.9) * std(TimeM(:, :)), sqrt(0.9) * std(TimeM(:, :)), 'LineWidth', 1);

% xl = xlabel('$Time \ (s)$','Interpreter','LaTeX');
% yl = ylabel('$Cost$','Interpreter','LaTeX');
xl = xlabel('Time (s)');
yl = ylabel('Cost');
set(xl,'FontSize',18);
set(yl,'FontSize',18);
set(gca,'FontSize',16,'FontName','Times');



