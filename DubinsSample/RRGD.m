function [Vertices, Edges, Edges_data, Belief_queue_next]= RRGD(Vertices, Edges, Edges_data, Belief_queue_next_old, BeliefNodes, samples, deter_vertix, dim, segmentLength, r, world, param)

vertix_number_old = size(Vertices, 1);
vertices_queue = [];
for k = 1:samples
    l = k + vertix_number_old;
    sample_succ = 0;
    while sample_succ == 0
        randomPoint = zeros(1, 4);
        min_dist = 0;
        while min_dist < 0.5
            for j = 1:dim
                randomPoint(j) = deter_vertix.Vertices(l,j);
            end
            % find the vertix that is closest to randomPoint (Eucl. dist. between positions)
            tmp = Vertices(:, 1 : dim) - randomPoint(1 : dim);
            sqr_dist = sqr_eucl_dist(tmp, dim);
            [min_dist, idx] = min(sqr_dist);
        end
        
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
            new_point(dim + 1 : 2 * dim) = deter_vertix.Vertices(l, 3:4);
        end
        
        % check if the new_point is in collision
        if collision_point(new_point, world) == 0
            [meanTraj, MCost, N]  = meanControl(Vertices(idx, :)', new_point', param);                                                                                                                                     
            if ~MeanCollisionCheck(meanTraj(1:2, :), world, dim)
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
                
                if idx <= vertix_number_old     
                    % if isempty(find (vertices_queue == idx, 1))
                    %     vertices_queue = [vertices_queue idx];
                    % end
                    vertices_queue = [vertices_queue idx];
                end
                Vertices = [Vertices; new_point];
                new_verti_idx = size(Vertices,1);
                Edges(idx) = {[Edges{idx} new_verti_idx]};
                Edges_data{idx, new_verti_idx} = {meanTraj, MCost, N};
                plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
                [meanTraj, MCost, N]  = meanControl(new_point', Vertices(idx, :)', param);  
                if ~MeanCollisionCheck(meanTraj(1:2, :), world, dim)
                    Edges(new_verti_idx) = {[Edges{new_verti_idx} idx]};
                    Edges_data{new_verti_idx,idx} = {meanTraj, MCost, N};
                    plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
                end
                
                size_near = size(near_idx, 1);   
                for i = 1 : size_near                                              
                    if near_idx(i) ~= idx
                        [meanTraj, MCost, N]  = meanControl(Vertices(near_idx(i), :)', new_point', param);  
                        if ~MeanCollisionCheck(meanTraj(1:2, :), world, dim)
                            Edges(near_idx(i)) = {[Edges{near_idx(i)} new_verti_idx]};
                            Edges_data{near_idx(i), new_verti_idx} = {meanTraj, MCost, N};
                            plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
                            if near_idx(i) <= vertix_number_old      
                                % if isempty(find (vertices_queue == near_idx(i), 1))
                                %     vertices_queue = [vertices_queue near_idx(i)];
                                % end   
                                vertices_queue = [vertices_queue near_idx(i)];                                                                                        
                            end                                        
                        end
                        
                        [meanTraj, MCost, N]  = meanControl(new_point', Vertices(near_idx(i), :)', param);  
                        if ~MeanCollisionCheck(meanTraj(1:2, :), world, dim)
                            Edges(new_verti_idx) = {[Edges{new_verti_idx} near_idx(i)]};
                            Edges_data{new_verti_idx, near_idx(i)} = {meanTraj, MCost, N};
                            plot(meanTraj(1,:), meanTraj(2,:), 'color', 'g', 'LineWidth', 1);
                        end
                    end
                end             
            end
        end
    end    
end              
vertices_queue = unique(vertices_queue);
Belief_queue_next = {};
% add nodes to Belief_queue
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

% delete any empty node from Belief_queue
empty_idx = [];
for ell = 1:size(Belief_queue_next, 2)
    if isempty(Belief_queue_next{ell})
        empty_idx = [empty_idx ell];
    end
end
for i = length(empty_idx):-1:1
    Belief_queue_next = {Belief_queue_next{1:empty_idx(i)-1}, Belief_queue_next{empty_idx(i)+1 : end}};
end
end