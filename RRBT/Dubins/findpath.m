function Path_idx = findpath(BeliefNodes, idx)

Path_idx=[];

while idx(1)~=1 || idx(2)~=1 
    
    Path_idx = [idx, Path_idx];
    Parent_idx = BeliefNodes{idx(1)}{idx(2)}{6};
    idx = Parent_idx;

end

Path_idx = [idx, Path_idx];

end