clear
addpath('CS');
% deter_vertix = load('deterministicVertices1');
% deter_Vertices = deter_vertix.Vertices;
rng(0);
dim = 2;   

start_cord = [2,8,0,0];                      % start_cord = [9,6,0,0];
P0 = diag([0.10, 0.08, 0.01, 0.01]);
PtildePrior0 = 0.6 * P0.* diag([1 1 1 1]);   % Initial estimation error covariance
PhatPrior0 = P0 - PtildePrior0;              % Initial estimated state covariance

goal_cord = [18,18,0,0];                     % goal_cord = [17,18,0,0];
fig = figure(1);
world = createKnownWorld(dim);               % Create random world

velRand = rand(500, 2)-0.5;
param.dt = 0.2;
param.velavg = 1;
param.chanceConstraint = 0.1;
param.saveAnim = 1;

if param.saveAnim
    title("Planning Environment")
    set(gca,'FontSize',18,'FontName','Times');
    drawnow
    frame{1} = getframe(fig);
    im{1} = frame2im(frame{1});
end

r = 6;                                       % Neighbor distance
segmentLength = 5.8;                         % Maximum steplength

samples = 40;
Edges = cell(300,1);
EdgesCost = cell(300,1);
BeliefNodes = cell(300,1);

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
        if min_dist > segmentLength^2                          % find new_point that is within the range of idx  
            randomPoint = pos(idx, :) + Vect * segmentLength;  % generate a new point segmentLength away from pos(idx, :)
        end     
        
        if collision_point(randomPoint, world) == 0            % check if the new_point is in collision
            sample_succ = 1;
            pos = [pos; randomPoint];
        end
    end      
end
vel = [start_cord(3:4); goal_cord(3:4); velRand(1:samples, :)];
Vertices = [pos,vel];

% pos = [start_cord(1:2); goal_cord(1:2)];
% for l = 1:samples
%     min_dist = 0;
%     while min_dist < 1
%         randomPoint = rand(1,2)*20;
%         [min_dist, idx] = min(sum((pos - randomPoint).^2,2)); % find the vertix that is closest to randomPoint
%     end   
%     pos = [pos; randomPoint];
% end
% vel = [start_cord(3:4); goal_cord(3:4); 1-rand(samples, 2)*2];
% Vertices = [pos,vel];

% % Vertices = [ start_cord; goal_cord; deter_Vertices(2:samples, :)];                       % start_cord already in deter_vertices
% Vertices = [ start_cord; goal_cord; [rand(samples, 2)*20, 1-rand(samples, 2)*2]];        

Vertices = Vertices(arrayfun(@(x,y) ~collision_point([x,y], world), Vertices(:,1), Vertices(:,2)), :);

if param.saveAnim
    scatter(Vertices(:,1), Vertices(:,2), 12, 'k', 'filled');
    plot(start_cord(1), start_cord(2), 'Marker','s','MarkerSize',10,'MarkerEdgeColor','[0.8500 0.3250 0.0980]','MarkerFaceColor','[0.8500 0.3250 0.0980]')
    plot(goal_cord(1), goal_cord(2), 'Marker','o','MarkerSize',8,'MarkerEdgeColor','[0.8500 0.3250 0.0980]','MarkerFaceColor','[0.8500 0.3250 0.0980]')
    title("Batch Sampling")
    set(gca,'FontSize',18,'FontName','Times');
    drawnow
    frame{size(im,2) + 1} = getframe(fig);    
    im{size(im,2) + 1} = frame2im(frame{size(im,2) + 1} );
end

D = squareform(pdist(Vertices(:,1:2)));                                  % distance matrix
N = size(Vertices,1);
idx = 1:N; 
NN = arrayfun(@(v) [idx(D(v,1:v-1) < r), v + idx(D(v,v+1:end) < r)], 1:N, 'uniformoutput', false);  % neighbor idx of all vertices
for i = 1:N
    for child_idx = NN{i}
        [meanTraj, MCost, N]  = meanControl(Vertices(i, :)', Vertices(child_idx, :)', param); 
        if ~MeanCollisionCheck(meanTraj(1:2, :), world, dim)
            Edges(i) = {[Edges{i} child_idx]};
            EdgesCost{i} = [EdgesCost{i} MCost];
            Edges_data{i, child_idx} = {meanTraj, N};
            if param.saveAnim
                plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
                plot(meanTraj(1,1), meanTraj(2,1), 'Marker','.','MarkerSize',8,'MarkerEdgeColor','[0.5 0.5 0.5]')
            end
        end
    end
end

if param.saveAnim
    title("Nominal Trajectory Graph")
    drawnow
    frame{size(im,2) + 1} = getframe(fig);
    im{size(im,2) + 1} = frame2im(frame{size(im,2) + 1});
end

CostValue = [];
CostValue = VI(Vertices, Edges, EdgesCost, CostValue);

if param.saveAnim
    plotCovariance(start_cord, P0);
    title("Graph Search After Computing Heuristic")
    drawnow
    frame{size(im,2) + 1} = getframe(fig);
    im{size(im,2) + 1} = frame2im(frame{size(im,2) + 1});
end

node_init(1) = {P0}; node_init(2) = {PtildePrior0};                 % node state covariance, estimation error covariance,
node_init(3) = {[0, CostValue(1)]};                                 % cost=[cost-to-come, g]
node_init(4) = {[]}; node_init(5) = {[1;1]}; node_init(6) = {[]};   % children node idx, coordinate in BeliefNodes, parent_node idx
BeliefNodes{1,1} = {node_init};

%% Iteratively search the graph
Belief_queue_current = {node_init};
queue_size_current = size(Belief_queue_current, 2);
CostM = []; Time = []; BeliefTrees = {}; TreesVertices = {}; Connected_flag = 0;

while queue_size_current > 0    
    [Belief_queue_current, BeliefNodes, success] = SearchGraph(Edges, EdgesCost, CostValue, Edges_data, Belief_queue_current, BeliefNodes, param, world);    
    if param.saveAnim
        drawnow
        frame{size(im,2) + 1} = getframe(fig);
        im{size(im,2) + 1} = frame2im(frame{size(im,2) + 1});
    end

    if success       
        Time = [Time toc];
        BeliefTrees = [BeliefTrees {BeliefNodes}];
        TreesVertices = [TreesVertices, {Vertices}];
        break;
    end 
    queue_size_current = size(Belief_queue_current, 2);    
end  
PathCost = [];
for i = 1:size(BeliefNodes{2},2)
    PathCost(i) = BeliefNodes{2}{i}{3}(2);
end
CostM = [CostM  min(PathCost)];

if param.saveAnim
    title("Current Best Solution Found")
    drawnow
    frame{size(im,2) + 1} = getframe(fig);
    im{size(im,2) + 1} = frame2im(frame{size(im,2) + 1});

    [~, idx] = min(PathCost);
    goal_idx = [2; idx];
    Path_idx = findpath(BeliefNodes, goal_idx);    
    fig2 = figure(2); hold on
    plot(start_cord(1), start_cord(2), 'Marker','s','MarkerSize',10,'MarkerEdgeColor','[0.8500 0.3250 0.0980]','MarkerFaceColor','[0.8500 0.3250 0.0980]')
    plot(goal_cord(1), goal_cord(2), 'Marker','o','MarkerSize',10,'MarkerEdgeColor','[0.8500 0.3250 0.0980]','MarkerFaceColor','[0.8500 0.3250 0.0980]')
    plotWorld(world, dim); 
    MC_path(Vertices, BeliefNodes, Path_idx, param, world);
    plot_path(Vertices, BeliefNodes, Path_idx, param, world);
    title("Solution")
    set(gca,'FontSize',18,'FontName','Times');
    drawnow
    frame{size(im,2) + 1} = getframe(fig2);
    im{size(im,2) + 1} = frame2im(frame{size(im,2) + 1});
    close;
end


%% Additional Batches
for k = 1:10
    figure(1)
    [Vertices, Edges, EdgesCost, Edges_data, Belief_queue_current] = RRGD_Random(Vertices, Edges, EdgesCost, Edges_data, Belief_queue_current, BeliefNodes, 20, dim, segmentLength, r, world, param);
    if param.saveAnim
        title("Batch Sampling And New Graph")
        drawnow
        frame{size(im,2) + 1} = getframe(fig);
        im{size(im,2) + 1} = frame2im(frame{size(im,2) + 1});
    end
    CostValue = VI(Vertices, Edges, EdgesCost, CostValue);
    for i =1:size(Belief_queue_current,2) 
        Belief_queue_current{i}{3}(2) = Belief_queue_current{i}{3}(1) + CostValue(Belief_queue_current{i}{5}(1));
    end

    queue_size_current = size(Belief_queue_current, 2);
    while queue_size_current > 0        
        [Belief_queue_current, BeliefNodes, success] = SearchGraph(Edges, EdgesCost, CostValue, Edges_data, Belief_queue_current, BeliefNodes, param, world);   
        if param.saveAnim
            title("Graph Search After Computing Heuristic")
            drawnow
            frame{size(im,2) + 1} = getframe(fig);
            im{size(im,2) + 1} = frame2im(frame{size(im,2) + 1});
        end
        if success       
            Time = [Time toc];
            BeliefTrees = [BeliefTrees {BeliefNodes}];
            TreesVertices = [TreesVertices, {Vertices}];
            break;
        end        
        queue_size_current = size(Belief_queue_current, 2);        
    end
    PathCost = [];
    for i = 1:size(BeliefNodes{2},2)
        PathCost(i) = BeliefNodes{2}{i}{3}(2);
    end
    CostM = [CostM  min(PathCost)];

    if param.saveAnim
        title("Current Best Solution Found")
        drawnow
        frame{size(im,2) + 1} = getframe(fig);
        im{size(im,2) + 1} = frame2im(frame{size(im,2) + 1});

        [~, idx] = min(PathCost);
        goal_idx = [2; idx];
        Path_idx = findpath(BeliefNodes, goal_idx);        
        fig2 = figure(2); hold on
        plot(start_cord(1), start_cord(2), 'Marker','s','MarkerSize',10,'MarkerEdgeColor','[0.8500 0.3250 0.0980]','MarkerFaceColor','[0.8500 0.3250 0.0980]')
        plot(goal_cord(1), goal_cord(2), 'Marker','o','MarkerSize',10,'MarkerEdgeColor','[0.8500 0.3250 0.0980]','MarkerFaceColor','[0.8500 0.3250 0.0980]')
        plotWorld(world, dim); 
        MC_path(Vertices, BeliefNodes, Path_idx, param, world);
        plot_path(Vertices, BeliefNodes, Path_idx, param, world);
        title("Solution")
        set(gca,'FontSize',18,'FontName','Times');
        drawnow
        frame{size(im,2) + 1} = getframe(fig2);
        im{size(im,2) + 1} = frame2im(frame{size(im,2) + 1});    
        close;
    end
end
CostM
Time

if param.saveAnim
    filename = "testAnimated6.gif";
    for idx = 1:size(im,2)
        [A,map] = rgb2ind(im{idx},256);
        if idx == 1
            imwrite(A,map,filename,"gif","LoopCount",Inf,"DelayTime",1);
        else
            imwrite(A,map,filename,"gif","WriteMode","append","DelayTime",1);
        end
    end

    writerObj=VideoWriter('test3.avi'); 
    writerObj.FrameRate = 1;
    open(writerObj); 
    for i = 1:size(frame,2)
        writeVideo(writerObj,frame{i});
    end
    close(writerObj);   
end

% % for i = 1:size(Vertices,1)
% %     for j = 1:size(BeliefNodes{i},2)
% %         if ~isempty(BeliefNodes{i}{j})
% %             plotCovariance(Vertices(i,:), BeliefNodes{i}{j}{1});
% %         end
% %     end  
% % end


%% Iteratively Search the graph
function [Belief_queue_current, BeliefNodes, success] = SearchGraph(Edges, EdgesCost, CostValue, Edges_data, Belief_queue_current, BeliefNodes, param, world)
Belief_queue_new = {};
queue_size = size(Belief_queue_current, 2);
success = 0;
Cost = [];
for j = 1:queue_size
    Cost(j) = Belief_queue_current{j}{3}(2);
end
[min_cost, cost_idx] = min(Cost);
pop_node = Belief_queue_current{cost_idx};
vertix_idx = pop_node{5}(1); 
if vertix_idx == 2
    success = 1;
    return
end
Belief_queue_current = {Belief_queue_current{1:cost_idx-1}, Belief_queue_current{cost_idx+1 : queue_size}};
                
                                   
for j = 1:size(Edges{vertix_idx},2)   % neighbor of vertix vertix_idx  
                    
    if ~isempty(pop_node{4})
        if find(pop_node{4}(1,:) == Edges{vertix_idx}(j),1)   % find Edges{vertix_idx}(j) in pop_node{4}(1,:), pop_node has already been expand to Edges{vertix_idx}(j)
           continue;
        end
    end   
                    
    [meanTraj, N] = Edges_data{vertix_idx, Edges{vertix_idx}(j)}{1:end};
    MCost = EdgesCost{vertix_idx}(j);
    [endP0, endPtilde, CovCost, CollisionProb, ~] = propagate( pop_node{1}, pop_node{2}, N, param, meanTraj, world );    
    CovCost = 0;
%     CollisionProb = 0;
    if CollisionProb <= param.chanceConstraint            
        cost_to_come = [pop_node{3}(1) + MCost + CovCost, pop_node{3}(1) + MCost + CovCost + CostValue(Edges{vertix_idx}(j))];
        if isempty(BeliefNodes{Edges{vertix_idx}(j)})   % there is no belief node at Edges{vertix_idx}(j), the new node is directly added    
            if param.saveAnim
                plot(meanTraj(1,:), meanTraj(2,:), 'color', 'r', 'LineWidth', 1);
                plotCovariance(meanTraj(:,end), endP0);
            end
            BeliefNodes{Edges{vertix_idx}(j), 1} = {{endP0, endPtilde, cost_to_come, [], [Edges{vertix_idx}(j); 1], [pop_node{5}(1); pop_node{5}(2)]}};
            % update child list
            BeliefNodes{pop_node{5}(1)}{pop_node{5}(2)}{4} = [BeliefNodes{pop_node{5}(1)}{pop_node{5}(2)}{4} [Edges{vertix_idx}(j); 1]];
                            
            Belief_queue_new = [Belief_queue_new(), BeliefNodes{Edges{vertix_idx}(j)}];
        else
            % check if the new node is dominated by any existing node at that vertix
            dominated = 0;
            for m = 1:size(BeliefNodes{Edges{vertix_idx}(j)},2)
                if ~isempty(BeliefNodes{Edges{vertix_idx}(j)}{m})
                   if cost_to_come(1) + 0.01 > BeliefNodes{Edges{vertix_idx}(j)}{m}{3}(1) && ...
                       ~MatrixNotPD(endP0(1:2,1:2) + 0.01 * eye(2) - BeliefNodes{Edges{vertix_idx}(j)}{m}{1}(1:2,1:2)) && ...
                       ~MatrixNotPD(endPtilde(1:2,1:2) + 0.01 * eye(2) - BeliefNodes{Edges{vertix_idx}(j)}{m}{2}(1:2,1:2))               
                       dominated = 1;
                       break;
                   end
                end
            end
            % if the new node is not dominated, add it to the tree
            if dominated == 0
                if param.saveAnim
                    plot(meanTraj(1,:), meanTraj(2,:), 'color', 'r', 'LineWidth', 1);
                    plotCovariance(meanTraj(:,end), endP0);
                end
                col_idx = size(BeliefNodes{Edges{vertix_idx}(j)},2) + 1;
                BeliefNodes{Edges{vertix_idx}(j), 1}(col_idx) = {{endP0, endPtilde, cost_to_come, [], [Edges{vertix_idx}(j); col_idx], [pop_node{5}(1); pop_node{5}(2)]}};
                % update child list
                BeliefNodes{pop_node{5}(1)}{pop_node{5}(2)}{4} = [BeliefNodes{pop_node{5}(1)}{pop_node{5}(2)}{4} [Edges{vertix_idx}(j); col_idx]];
                                
                Belief_queue_new = [Belief_queue_new(), BeliefNodes{Edges{vertix_idx}(j)}(end)];                               
            end
        end 
    end                        
end
Belief_queue_current = [Belief_queue_current(), Belief_queue_new()];
end