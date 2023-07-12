pointA = [ 1, 2,   0*pi/180 ];     
pointB = [ 5, 2,   100*pi/180 ];    
TurnRadius = 2;   
PathStep = 0.2;   
[length, path] = dubins_curve(pointA,pointB, TurnRadius, PathStep);