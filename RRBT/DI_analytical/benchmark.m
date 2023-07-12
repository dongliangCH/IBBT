run = 10;
CostM = [];
TimeM = [];
for i = 1:run
    [Time, Cost] = RRBTprune5;
    CostM = [CostM; Cost];
    TimeM = [TimeM; Time]; 
end