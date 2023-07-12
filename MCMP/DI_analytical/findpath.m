function Path_idx = findpath(BeliefNodes, idx)

Path_idx=[];

while idx(1)~=1 || idx(2)~=1 
    
    Path_idx = [idx, Path_idx];
    idx = BeliefNodes{idx(1)}{idx(2)}{6};  % Parent idx

end

Path_idx = [idx, Path_idx];

end