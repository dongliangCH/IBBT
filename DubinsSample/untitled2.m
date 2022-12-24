% node_mun = 0;
% for i = 1:size(BeliefNodes,1)
%     node_mun = node_mun + size(BeliefNodes{i},2);
% end

x = [40 80 100 150 200 250 300];
y = [0.1194 0.0012 0.1610; 0.4890 0.0027 0.3416; 0.7449 0.0029 0.7160;...
    1.6214 0.0039 0.7521; 2.7543 0.0050 1.6818; 4.1218 0.0072 0.7602; 6.2581 0.0129 0.9776];
totaltime = [0.2816 0.8332 1.4639 2.3775 4.4411 4.8931 7.2486];
cost = [33.4835 26.2024 23.3283 20.6551 20.1970 18.9619 18.7065];

% bar(x,y)
% xlabel('Graph size');
% ylabel('time(s)')
% set(gca,'FontSize',16,'FontName','Times');

vertices = [50 60 70 80 90 100];
totaltime1 = [0.513533 0.593168 0.695883 1.029838 1.418207 2.319602];
cost1 = [31.3923 28.8256 21.8774 21.8774 20.0216 18.8356];

figure
plot(totaltime1, cost1, 'LineWidth', 1);hold on
plot(totaltime, cost, 'LineWidth', 1)
xlabel('Planning time (s)');
ylabel('Cost')
set(gca,'FontSize',16,'FontName','Times');

