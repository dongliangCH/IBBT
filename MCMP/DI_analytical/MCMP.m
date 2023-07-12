clear
addpath('CS');
rng(0);
dim = 2;   

start_cord = [2,8,0,0];                          % start_cord = [9,6,0,0];
P0 = diag([0.10, 0.08, 0.01, 0.01]);
PtildePrior0 = 0.6 * P0.* diag([1 1 1 1]);       % Initial estimation error covariance
PhatPrior0 = P0 - PtildePrior0;                  % Initial estimated state covariance
goal_cord = [18,18,0,0];                         % goal_cord = [17,18,0,0];

world = createKnownWorld(dim);                   % Create random world

velRand = rand(500, 2)-0.5;
param.dt = 0.2;
param.velavg = 1;
param.chanceConstraint = 0.12;
param.Imax = 4;
param.Imin = 0;
param.bisec_step = 10;
% param.deltaI = 0.2;
% param.iniI = 0.2;

r = 6;                                           % Neighbor distance
segmentLength = 5.8;                             % Maximum steplength

samples = 200;
Edges = cell(500,1);
EdgesCost = cell(500,1);
BeliefNodes = cell(500,1);

tic
pos = [start_cord(1:2); goal_cord(1:2)];
for l = 1:samples
    sample_succ = 0;
    while sample_succ == 0
        min_dist = 0;
        while min_dist < 1
            randomPoint = rand(1,2)*20;
            [min_dist, idx] = min(sum((pos - randomPoint).^2,2)); % find the vertix that is closest to randomPoint
        end
        
        Vect = randomPoint-pos(idx,:);
        Vect = Vect/norm(Vect);     
        if min_dist > segmentLength^2                             % find new_point that is within the range of idx  
            randomPoint = pos(idx, :) + Vect * segmentLength;     % generate a new point segmentLength away from pos(idx, :)
        end     
        
        if collision_point(randomPoint, world) == 0          % check if the new_point is in collision
            sample_succ = 1;
            pos = [pos; randomPoint];
        end
    end      
end
vel = [start_cord(3:4); goal_cord(3:4); velRand(1:samples, :)];
Vertices = [pos,vel];    

Vertices = Vertices(arrayfun(@(x,y) ~collision_point([x,y], world), Vertices(:,1), Vertices(:,2)), :);
D = squareform(pdist(Vertices(:,1:2)));                            % distance matrix
N = size(Vertices,1);
idx = 1:N; 

NN = arrayfun(@(v) [idx(D(v,1:v-1) < r), v + idx(D(v,v+1:end) < r)], 1:N, 'uniformoutput', false);  % neighbor idx of all vertices

Imax = param.Imax;
Imin = param.Imin;
CollisionProb = [];
Cost = [];
k = 1;
for i = 1:param.bisec_step
    inflate = (Imax + Imin)/2;
    current_world = inflate_world(world, inflate);

    [success, Path_condidate, MCost] = FMTstar(Vertices', NN, current_world, dim, param, 1);
    if success
        [collisionProb, CovCost] = MCCollisionProb(Vertices, P0, PtildePrior0, Path_condidate, param, world, 0);
    else
        collisionProb = 0;
    end
    if collisionProb > param.chanceConstraint
        Imin = (Imax + Imin)/2;
    else
        Imax = (Imax + Imin)/2;
        if success
            Cost = [Cost, MCost];
            Path(k).path = Path_condidate;
            CollisionProb = [CollisionProb, collisionProb];
            k = k + 1;
        end
    end
    if Imax - Imin < 0.05
        break
    end
end

CostM = []; Time = [];
Time = [Time toc]
CostM = [CostM, Cost(end)]

figure; hold on
plot(start_cord(1), start_cord(2), 'Marker','s','MarkerSize',10,'MarkerEdgeColor','[0.8500 0.3250 0.0980]','MarkerFaceColor','[0.8500 0.3250 0.0980]')
plotWorld(world, dim); 
Path_condidate = Path(end).path;
[collisionProb, CovCost] = MCCollisionProb(Vertices, P0, PtildePrior0, Path_condidate, param, world, 1);
