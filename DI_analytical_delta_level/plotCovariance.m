function plotCovariance(xbarf, Pf)
    
    rl = 0.9545; % 2 Sigma   
    error_ellipse('C', Pf(1:2, 1:2), 'mu', xbarf(1:2), ...
        'conf', rl, 'style', 'k', 'linewidth', 0.567);
    plot(xbarf(1), xbarf(2), 'Marker','.','MarkerSize',10,'MarkerEdgeColor','[0.8500 0.3250 0.0980]')
    
end