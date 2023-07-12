function plotWorld(world,dim)

  if dim ==2

  axis([world.origincorner(1),world.endcorner(1), world.origincorner(2), world.endcorner(2)]);
  plot([world.origincorner(1) world.endcorner(1)  world.endcorner(1) world.origincorner(1) world.origincorner(1)], ...
      [world.origincorner(2) world.origincorner(2) world.endcorner(2) world.endcorner(2) world.origincorner(2)], 'color', 'k', 'linewidth', 1)
%   axis off
  hold on
  
%   N = 20;
%   th = 0:2*pi/N:2*pi;
  for i=1:world.NumObstacles
      rectangle('Position',[world.cx(i)-world.radius(i),world.cy(i)-world.radius(i),...
          2*world.radius(i),2*world.radius(i)],'EdgeColor','[0.5 0.5 0.5]','LineWidth',1,'FaceColor','[0.5 0.5 0.5]','Curvature', [1 1]);
%           2*world.radius(i),2*world.radius(i)],'EdgeColor','black','LineWidth',1,'FaceColor',[1 0 0],'Curvature', [1 1]);
%       X = world.radius(i)*sin(th) + world.cx(i);
%       Y = world.radius(i)*cos(th) + world.cy(i);
%       fill(X,Y,'blue');
  end

  for k = 1:world.NumMarker
      plot(world.Marker(k,1), world.Marker(k,2), 'kp', 'MarkerSize', 5, 'LineWidth', 2);
  end
  axis equal
  axis off
  xlim([0, 20])
  ylim([0, 20])
%   xl = xlabel('$x  (m)$','Interpreter','LaTeX');
%   yl = ylabel('$y  (m)$','Interpreter','LaTeX');
%   set(xl,'FontSize',18);
%   set(yl,'FontSize',18);
  set(gca,'FontSize',16,'FontName','Times');

   
  elseif dim ==3
  axis([world.origincorner(1),world.endcorner(1),...
      world.origincorner(2), world.endcorner(2),...
      world.origincorner(3), world.endcorner(3)]);
  hold on

  for i=1:world.NumObstacles
      [X,Y,Z] = sphere;
      X = (X*world.radius(i));
      Y = (Y*world.radius(i));
      Z = (Z*world.radius(i));
%       c = ones(size(Z));
%       surf(X+world.cx(i),Y+world.cy(i),Z+world.cz(i),c);
      surf(X+world.cx(i),Y+world.cy(i),Z+world.cz(i));
      colormap([0.5,0.5,0.5]);
  end

  xl = xlabel('$x  (m)$','Interpreter','LaTeX');
  yl = ylabel('$y  (m)$','Interpreter','LaTeX');
  zl = zlabel('$z  (m)$','Interpreter','LaTeX');
  set(xl,'FontSize',18);
  set(yl,'FontSize',18);
  set(zl,'FontSize',18);
  set(gca,'FontSize',16,'FontName','Times');
  
  end

end
