function [Vertices, Edges, EdgesCost, Edges_data, Belief_queue_next]= RRGD_Random(Vertices, Edges, EdgesCost, Edges_data, Belief_queue_next_old, BeliefNodes, ...
    samples, dim, segmentLength, r, world, param) %, rand_idx, randM)

vertix_number_old = size(Vertices, 1);
vertices_queue = [];
for k = 1:samples
    sample_succ = 0;
    while sample_succ == 0
        randomPoint = zeros(1, 2);
        min_dist = 0;
        while min_dist < 0.5
            for j = 1:dim
                randomPoint(1, j) = world.origincorner(j) + (world.endcorner(j) - world.origincorner(j)) * rand; %randM(rand_idx);
%                 rand_idx = rand_idx + 1;
            end
            % find the vertix that is closest to randomPoint (Eucl. dist. between positions)
            tmp = Vertices(:, 1 : dim) - randomPoint(1 : dim);
            sqr_dist = sqr_eucl_dist(tmp, dim);
            [min_dist, ~] = min(sqr_dist);
        end
        new_point = randomPoint;
        % check if the new_point is in collision
        if collision_point(new_point, world) == 0
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
            size_near = size(near_idx, 1);   
            for i = 1 : size_near                                              
                [meanTraj, MCost, N]  = meanControl(Vertices(near_idx(i), :)', new_point', param);  
                if ~MeanCollisionCheck(meanTraj(1:2, :), world, dim)
                    Edges(near_idx(i)) = {[Edges{near_idx(i)} new_verti_idx]};
                    EdgesCost{near_idx(i)} = [EdgesCost{near_idx(i)} MCost];
                    Edges_data{near_idx(i), new_verti_idx} = {meanTraj, N};
%                     plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
                    if near_idx(i) <= vertix_number_old
                        if isempty(find (vertices_queue == near_idx(i), 1))
                            vertices_queue = [vertices_queue near_idx(i)];
                        end                                            
                    end                                        
                end
                
                [meanTraj, MCost, N]  = meanControl(new_point', Vertices(near_idx(i), :)', param);  
                if ~MeanCollisionCheck(meanTraj(1:2, :), world, dim)
                    Edges(new_verti_idx) = {[Edges{new_verti_idx} near_idx(i)]};
                    EdgesCost{new_verti_idx} = [EdgesCost{new_verti_idx} MCost];
                    Edges_data{new_verti_idx, near_idx(i)} = {meanTraj, N};
%                     plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
                end
            end             
        end  
    end
end

% add goal vertice to queue, goal_idx = 2
if isempty(find (vertices_queue == 2, 1))
    vertices_queue = [vertices_queue 2];
end
% add nodes to Belief_queue
Belief_queue_next = {};
for i = 1:size(vertices_queue,2)
    try
        Belief_queue_next = [Belief_queue_next(), BeliefNodes{vertices_queue(i)}];
    catch
    end
end            
for i = 1:size(Belief_queue_next_old,2)
    if ~ismember(Belief_queue_next_old{i}{5}(1), vertices_queue)
        Belief_queue_next = [Belief_queue_next, Belief_queue_next_old(i)];
    end
end

% % delete any empty node from Belief_queue
% empty_idx = [];
% for ell = 1:size(Belief_queue_next, 2)
%     if isempty(Belief_queue_next{ell})
%         empty_idx = [empty_idx ell];
%     end
% end
% for i = length(empty_idx):-1:1
%     Belief_queue_next = {Belief_queue_next{1:empty_idx(i)-1}, Belief_queue_next{empty_idx(i)+1 : end}};
% end
end