clear
addpath('CS');
rng(0);
dim = 2;   

% Initial state distribution (mean/covariance)
start_cord = [1,4,0,0];
P0 = diag([0.10, 0.08, 0.01, 0.01]);

% Initial estimation error covariance
PtildePrior0 = 0.2 * P0.* diag([1 1 1 1]);

% Initial estimated state covariance
PhatPrior0 = P0 - PtildePrior0;

goal_cord = [4,9,0,0];

% Create random world
world = createKnownWorld([5,10],[0,0],dim);

param.dt = 0.1;
param.velavg = 1;

% Neighbor distance
r = 3;
% Maximum steplength
segmentLength = 2.5;

samples = 15;
Vertices = start_cord;
Edges = cell(samples,1);
% coder.varsize('Edges')
node_init(1) = {P0}; node_init(2) = {PtildePrior0};  node_init(3) = {0};  node_init(4) = {[]}; node_init(5) = {[1;1]}; node_init(6) = {[]}; % cost, child, coordinate in BeliefNodes, parent_node
BeliefNodes{1,1} = {node_init};

figure(10); hold on
plot(start_cord(1), start_cord(2), 'Marker','s','MarkerSize',10,'MarkerEdgeColor','[0.8500 0.3250 0.0980]','MarkerFaceColor','[0.8500 0.3250 0.0980]')
plotCovariance(start_cord, P0);
plotWorld(world, dim); 
tic
for l = 2:samples
    sample_succ = 0;
    while sample_succ == 0
        randomPoint = zeros(1, 4);
        min_dist = 0;
        while min_dist < 1
            for j = 1:dim
                randomPoint(1, j) = world.origincorner(j) + (world.endcorner(j) - world.origincorner(j)) * rand;
            end
            % find the vertix that is closest to randomPoint (Eucl. dist. between positions)
            tmp = Vertices(:, 1 : dim) - randomPoint(1 : dim);
            sqr_dist = sqr_eucl_dist(tmp, dim);
            [min_dist, idx] = min(sqr_dist);
        end
        min_parent_idx = idx;
        
        Vect = randomPoint(1:dim)-Vertices(idx,1:dim);
        Vect = Vect/norm(Vect);
        % find new_point that is within the range of idx
        if min_dist > segmentLength^2
            % generate a new point that is closest to randomPoint, segmentLength away from tree(idx,1:dim)
            new_point(1 : dim) = Vertices(idx, 1 : dim) + Vect * segmentLength;
        else
            new_point(1 : dim) = randomPoint(1 : dim);
        end
        
        if dim == 2
            % new_point(dim + 1 : 2 * dim) = zeros(1,2);
            new_point(dim + 1 : 2 * dim) = 0.2 * [-1 + (1 - (-1)) * rand, -1 + (1 - (-1)) * rand];     % vmin = -1£¬ vmax = 1
        end
        
        % check if the new_point is in collision
        if collision_point(new_point, world) == 0
            [meanTraj, V, MCost, N, Xbar]  = meanControl(Vertices(idx, :)', new_point', param);                                                                                                                                     
            if ~MeanCollisionCheck(meanTraj(1:2, :), world, dim)
                for k = 1:size(BeliefNodes{idx},2)
                    if ~isempty(BeliefNodes{idx}{k})
                    [endP0, endPtilde, CovCost, CollisionProb, K] = propagate( BeliefNodes{idx}{k}{1}, BeliefNodes{idx}{k}{2}, N, param, Xbar, V, world ); 
                    if CollisionProb < 0.3

                        sample_succ = 1;
                        
                        tmp_dist = Vertices(:, 1 : dim) - new_point(1 : dim);
                        dist_sqr = sqr_eucl_dist(tmp_dist, dim);
         
                        % find near neighbors   
                        if dim == 2
                            gamma = 60;
                        end      
                        nun = size(Vertices, 1);
                        ner = gamma * ( log(nun + 1) / nun )^(1 / dim);
                        r1 = min(ner, r);    
                        near_idx = find(dist_sqr <= r1^2);
                        
                        Vertices = [Vertices; new_point];
                        new_verti_idx = size(Vertices,1);
                        Edges(idx) = {[Edges{idx} new_verti_idx]};
                        plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
                        
                        [meanTraj, V, MCost, N, Xbar]  = meanControl(new_point', Vertices(idx, :)', param);  
                        if ~MeanCollisionCheck(meanTraj(1:2, :), world, dim)
                            Edges(new_verti_idx) = {[Edges{new_verti_idx} idx]};
                            plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
                        end
                        
                        Belief_queue = BeliefNodes{idx};

                        size_near = size(near_idx, 1);
                        if size_near >= 1          
                            for i = 1 : size_near                                              
                                if near_idx(i) ~= idx
                                    
                                    [meanTraj, V, MCost, N, Xbar]  = meanControl(Vertices(near_idx(i), :)', new_point', param);  
                                    if ~MeanCollisionCheck(meanTraj(1:2, :), world, dim)
                                        Edges(near_idx(i)) = {[Edges{near_idx(i)} new_verti_idx]};
                                        plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
                                    end
                                    
                                    [meanTraj, V, MCost, N, Xbar]  = meanControl(new_point', Vertices(near_idx(i), :)', param);  
                                    if ~MeanCollisionCheck(meanTraj(1:2, :), world, dim)
                                        Edges(new_verti_idx) = {[Edges{new_verti_idx} near_idx(i)]};
                                        plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
                                    end

                                    Belief_queue = [Belief_queue(), BeliefNodes{near_idx(i)}];
                                end
                            end             
                        end            
                        break;                 
                    end
                    end
                end
                
                empty_idx = [];
                for ell = 1:size(Belief_queue, 2)
                    if isempty(Belief_queue{ell})
                        empty_idx = [empty_idx ell];
                    end
                end
                for i = length(empty_idx):-1:1
                    Belief_queue = {Belief_queue{1:empty_idx(i)-1}, Belief_queue{empty_idx(i)+1 : end}};
                end
                
                % Search the graph to find all nondominated belief nodes
                [Belief_queue, BeliefNodes] = SearchGraph(Vertices, Edges, Belief_queue, BeliefNodes, param, world);
            end
        end
    end  
    
end
toc
[goal_idx, goalP0, min_cost, meanTraj]= ConnectGoalprune(Vertices, goal_cord, BeliefNodes, param, world, r);
% goal_idx = [7;1];

Path_idx = findpath(BeliefNodes, goal_idx);
figure(2); hold on
plot(start_cord(1), start_cord(2), 'Marker','s','MarkerSize',10,'MarkerEdgeColor','[0.8500 0.3250 0.0980]','MarkerFaceColor','[0.8500 0.3250 0.0980]')
plotWorld(world, dim); 
plot_path(Vertices, BeliefNodes, Path_idx, param, world);
%% plot last segment of the path
plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
plotCovariance(meanTraj(:,end), goalP0);
plot(goal_cord(1), goal_cord(2), 'Marker','d','MarkerSize',10,'MarkerFaceColor','[0.9290 0.6940 0.1250]')


%% Search the graph to find all nondominated belief nodes
function [Belief_queue, BeliefNodes] = SearchGraph(Vertices, Edges, Belief_queue, BeliefNodes, param, world)
                queue_size = size(Belief_queue, 2);
                while queue_size > 0
                    Cost = [];
                    for j = 1:queue_size
                        Cost(j) = Belief_queue{j}{3};
                    end
                    [~, cost_idx] = min(Cost);
                    pop_node = Belief_queue{cost_idx};
                    Belief_queue = {Belief_queue{1:cost_idx-1}, Belief_queue{cost_idx+1 : queue_size}};
                    
                    vertix_idx = pop_node{5}(1);
                    for j = 1:size(Edges{vertix_idx},2)
                        
                        [meanTraj, V, MCost, N, Xbar]  = meanControl(Vertices(vertix_idx, :)', Vertices(Edges{vertix_idx}(j),:)', param);
                        [endP0, endPtilde, CovCost, CollisionProb, ~] = propagate( pop_node{1}, pop_node{2}, N, param, Xbar, V, world );                         
                        if CollisionProb < 0.3
                            cost_to_come = pop_node{3} + MCost + CovCost;
                            if Edges{vertix_idx}(j) > size(BeliefNodes,1)  % Edges{vertix_idx}(j) is the newly added vertix, there is no node at this vertix and the new node is directly added
                                
                                plotCovariance(meanTraj(:,end), endP0);
                                BeliefNodes{Edges{vertix_idx}(j), 1} = {{endP0, endPtilde, cost_to_come, [], [Edges{vertix_idx}(j); 1], [pop_node{5}(1); pop_node{5}(2)]}};
                                % update child list
                                BeliefNodes{pop_node{5}(1)}{pop_node{5}(2)}{4} = [BeliefNodes{pop_node{5}(1)}{pop_node{5}(2)}{4} [Edges{vertix_idx}(j); 1]];
                                
                                Belief_queue = [Belief_queue(), BeliefNodes{Edges{vertix_idx}(j)}];
                            else
                                % check if the new node is dominated by any existing node at that vertix
                                dominated = 0;
                                for m = 1:size(BeliefNodes{Edges{vertix_idx}(j)},2)
                                    if ~isempty(BeliefNodes{Edges{vertix_idx}(j)}{m})
                                    if ~MatrixNotPD(endP0(1:2,1:2) + 0.02 * eye(2) - BeliefNodes{Edges{vertix_idx}(j)}{m}{1}(1:2,1:2)) &&...
                                            cost_to_come + 0.02 > BeliefNodes{Edges{vertix_idx}(j)}{m}{3}
                                            % && ~MatrixNotPD(endPtilde(1:2,1:2) + 0.02 * eye(2) - BeliefNodes{Edges{vertix_idx}(j)}{m}{2}(1:2,1:2))...              
                                        dominated = 1;
                                        break;
                                    end
                                    end
                                end
                                % if the new node is not dominated, add it to the tree
                                if dominated == 0
                                    plotCovariance(meanTraj(:,end), endP0);
                                    col_idx = size(BeliefNodes{Edges{vertix_idx}(j)},2) + 1;
                                    BeliefNodes{Edges{vertix_idx}(j), 1}(col_idx) = {{endP0, endPtilde, cost_to_come, [], [Edges{vertix_idx}(j); col_idx], [pop_node{5}(1); pop_node{5}(2)]}};
                                    % update child list
                                    BeliefNodes{pop_node{5}(1)}{pop_node{5}(2)}{4} = [BeliefNodes{pop_node{5}(1)}{pop_node{5}(2)}{4} [Edges{vertix_idx}(j); col_idx]];
                                    
                                    Belief_queue = [Belief_queue(), BeliefNodes{Edges{vertix_idx}(j)}(end)];
                                    
                                    % check if any existing node is dominated by the new node and prune
                                    for m = 1:size(BeliefNodes{Edges{vertix_idx}(j)},2)
                                        if ~isempty(BeliefNodes{Edges{vertix_idx}(j)}{m})
                                        if ~MatrixNotPD(BeliefNodes{Edges{vertix_idx}(j)}{m}{1}(1:2,1:2) - endP0(1:2,1:2)) &&...
                                            BeliefNodes{Edges{vertix_idx}(j)}{m}{3} > cost_to_come
                                            % && ~MatrixNotPD(BeliefNodes{Edges{vertix_idx}(j)}{m}{2}(1:2,1:2) - endPtilde(1:2,1:2))... 
                                            
                                            % delete node from BeliefNodes, first find its parent and update its parent's child list
                                            parent_idx = BeliefNodes{Edges{vertix_idx}(j)}{m}{6};
                                            child_list = BeliefNodes{parent_idx(1)}{parent_idx(2)}{4};
                                            idx_in_list = find( sum(child_list - BeliefNodes{Edges{vertix_idx}(j)}{m}{5}, 2) == 0);
                                            BeliefNodes{parent_idx(1)}{parent_idx(2)}{4} = [child_list(:, 1:idx_in_list-1) child_list(:, idx_in_list+1:end)];
                                            
                                            % delete node from Belief_queue
                                            queue_size = size(Belief_queue, 2);
                                            queue_list = [];
                                            for i = 1 : queue_size
                                                queue_list = [queue_list Belief_queue{i}{5}];
                                            end
                                            idx_in_queue = find( sum( queue_list - BeliefNodes{Edges{vertix_idx}(j)}{m}{5}, 2 ) == 0 );
                                            if idx_in_queue
                                                Belief_queue = {Belief_queue{1:idx_in_queue-1}, Belief_queue{idx_in_queue+1 : queue_size}};
                                            end
                                            
                                            Child_idx = BeliefNodes{Edges{vertix_idx}(j)}{m}{4};
                                            BeliefNodes{Edges{vertix_idx}(j)}{m} = {};
                                            [BeliefNodes, Belief_queue] = prune(BeliefNodes, Belief_queue, Child_idx);
                                        end
                                        end
                                    end
                                end
                            end
 
                        end
                        
                    end
                    queue_size = size(Belief_queue, 2);
                end
end

%% prune function
function [BeliefNodes, Belief_queue] = prune(BeliefNodes, Belief_queue, Child_idx)

for i = 1:size(Child_idx,2)
    
    % delete node from Belief_queue
    queue_size = size(Belief_queue, 2);
    queue_list = [];
    for j = 1 : queue_size
        queue_list = [queue_list Belief_queue{j}{5}];
    end
    idx_in_queue = find( sum( queue_list - Child_idx(:,i), 2 ) == 0 );
    if idx_in_queue
        Belief_queue = {Belief_queue{1:idx_in_queue-1}, Belief_queue{idx_in_queue+1 : queue_size}};
    end
    
    next_Child_idx = BeliefNodes{Child_idx(1,i)}{Child_idx(2,i)}{4};
    BeliefNodes{Child_idx(1,i)}{Child_idx(2,i)} = {};
    
    [BeliefNodes, Belief_queue] = prune(BeliefNodes, Belief_queue, next_Child_idx);
end

end


