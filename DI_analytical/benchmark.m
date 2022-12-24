run = 10;
% CostM = [];
% TimeM = [];
for i = 1:run
    [Time, Cost] = BBT4;
    CostM = [CostM; Cost];
    TimeM = [TimeM; Time]; 
end