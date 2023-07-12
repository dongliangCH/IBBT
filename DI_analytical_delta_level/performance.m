% node_mun = 0;
% for i = 1:size(BeliefNodes,1)
%     node_mun = node_mun + size(BeliefNodes{i},2);
% end

%% Continuous,velocity [-1, 1]
vertices = [30 50 100 150 200 250];
% y = [0.1194 0.0012 0.1610; 0.4890 0.0027 0.3416; 0.7449 0.0029 0.7160;...
%     1.6214 0.0039 0.7521; 2.7543 0.0050 1.6818; 4.1218 0.0072 0.7602; 6.2581 0.0129 0.9776];
totaltime = [0.106523 0.134251 0.283715 0.409963 0.541159 0.762368];
cost = [52.4024 37.2472 33.2385 30.5466 28.3158 28.3158];

%% Discrete
vertices1 = [30 50 100 150 200];
totaltime1 = [0.751595 1.418207 2.319602 6.966257 10.460829];
% cost1 = [31.3923 28.8256 21.8774 21.8774 20.0216 18.8356];

%% Continuous, Velocity Sample heuristic [0 1]
vertices2 = [30 50 100 150 200 250];
totaltime2 = [0.101528 0.122623 0.371210 0.469290 0.677572 0.928638];
cost2 =[36.1992 33.8234 31.4657 28.1345 28.1345 28.1345];

totaltime3 = [0.099772 0.132239 0.306727 0.460557 0.620781 0.800666];
cost3 = [48.3773 41.6525 37.1304 27.0290 27.0290 27.0290];

% bar(x,y)
% xlabel('Graph size');
% ylabel('time(s)')
% set(gca,'FontSize',16,'FontName','Times');



figure
plot(vertices, totaltime, 'LineWidth', 1);hold on
plot(vertices1, totaltime1, 'LineWidth', 1)
xlabel('Samples')
ylabel('Planning time (s)');
set(gca,'FontSize',16,'FontName','Times');
figure
plot(vertices, cost, 'LineWidth', 1);hold on
plot(vertices2, cost2, 'LineWidth', 1)
xlabel('Samples')
ylabel('Cost');
set(gca,'FontSize',16,'FontName','Times');
figure
plot(totaltime, cost, 'LineWidth', 1);hold on
plot(totaltime2, cost2, 'LineWidth', 1)
xlabel('Planning time (s)')
ylabel('Cost');
set(gca,'FontSize',16,'FontName','Times');




