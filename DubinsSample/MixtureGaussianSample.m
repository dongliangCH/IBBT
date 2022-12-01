% Mixture of Gaussian sample
function X_0 = MG(mu, sigma, p)

    gm = gmdistribution(mu,sigma,p);
    X = random(gm,1000);

end


% mu = [1 2;-3 -5];
% sigma = cat(3,[2 0;0 .5],[1 0;0 1]);
% p = ones(1,2)/2;
% gm = gmdistribution(mu,sigma,p);
% 
% gmPDF = @(x,y) arrayfun(@(x0,y0) pdf(gm,[x0 y0]),x,y);
% fcontour(gmPDF,[-10 10]);
% title('Contour lines of pdf');
% 
% rng('default') % For reproducibility
% X = random(gm,1000);
% 
% hold on
% scatter(X(:,1),X(:,2),10,'.') % Scatter plot with points of size 10
% title('Contour lines of pdf and Simulated Data')

