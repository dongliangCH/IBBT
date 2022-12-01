Traj = zeros(3, size(meanTraj,2));
Traj(:,1) = meanTraj(1:3, 1);
dt = param.dt;
for i = 2:size(meanTraj,2)
    Traj(:,i) = Traj(:,i-1) + [cos(Traj(3,i-1)); sin(Traj(3,i-1)); meanTraj(4,i-1)]*dt;
end
figure
plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);hold on
plot(Traj(1,:), Traj(2,:), 'color', 'r', 'LineWidth', 1);