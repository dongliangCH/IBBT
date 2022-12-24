%% Value Iteration
function CostValue = VI(Nodes, ChildM, EdgesCost, CostValue)
samp = size(Nodes, 1);
% Utility function / Value function 
CostValue = [CostValue; 10000 * ones(samp - length(CostValue), 1)];

% Set goal point
GoalIdx = 2;
% plot(Nodes(GoalIdx, 1) ,Nodes(GoalIdx, 2), 'o', 'color', 'b', 'linewidth', 2)
CostValue(GoalIdx) = 0;

% Convergence threshhold
epsn = 0.001;
delta = 0;

for i = 1:samp
    % The Value at the goal node is zero and not updated
    if i ~= GoalIdx
        tempValue = [];
        if ~isempty(ChildM{i})
        for j = 1:size(ChildM{i},2)
            tempValue(j) = EdgesCost{i}(j) + CostValue(ChildM{i}(j));
        end
        NewValue = min( tempValue );
        if NewValue < CostValue(i)
            if abs(CostValue(i) - NewValue) > delta 
                delta = abs(CostValue(i) - NewValue);
            end
            CostValue(i) = NewValue;  
        end
        end
    end
end

iter = 1;

while delta > epsn
    iter = iter+1;
    delta = 0;
    for i = 1:samp
        if i ~= GoalIdx
            tempValue = [];
            if ~isempty(ChildM{i})
            for j = 1:size(ChildM{i},2)
                tempValue(j) = EdgesCost{i}(j) + CostValue(ChildM{i}(j));
            end
            NewValue = min( tempValue );
            
            if NewValue < CostValue(i)
                if abs(CostValue(i) - NewValue) > delta 
                    delta = abs(CostValue(i) - NewValue);
                end
                CostValue(i) = NewValue;  
            end 
            end
        end
    end
end

%% Finding Path

% % Starting point, the first node is the starting point
% NextP = 1;
% Path = 1;
% tempValue = [];
% while NextP ~= GoalIdx    
%     for i = 1:size(ChildM{NextP},2)
%         tempValue(i) = EdgesCost{NextP}(i) + CostValue(ChildM{NextP}(i));
%     end
%     [~,Idx] = min(tempValue);
%     NextP = ChildM{NextP}(Idx);
%     Path = [Path, NextP];
% end

% for i = 1:size(Path, 2)-1
%     plot([Nodes(Path(i),1) Nodes(Path(i+1),1)], [Nodes(Path(i),2) Nodes(Path(i+1),2)],'r--'); hold on
% end