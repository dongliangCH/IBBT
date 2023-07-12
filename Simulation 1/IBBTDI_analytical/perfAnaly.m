
TimeIBBT = [0.0473 0.0471 0.0469 0.0541 0.0470 0.0499 0.0473 0.0475 0.0444 0.0505]'; 
TimeRRBT = [0.1704 0.1540 0.1366 0.1384 0.1381 0.1369 0.1445 0.1409 0.1377 0.1361]';

boxplot([TimeIBBT,TimeRRBT], 'Labels',{'IBBT','RRBT'})
yl = ylabel('Time (s)');
set(yl,'FontSize',18);
set(gca,'FontSize',16,'FontName','Times');