% calibration plane points
y = repmat(0:4:36,10,1);
cal_points = [mod(0:4:4*99,40);
              y(:)';
              100*ones(1,100)];
th = 0.1;          
R = [cos(th) -sin(th) 0;
     sin(th) cos(th) 0;
     0 0 1];
cal_points = R*cal_points + [-20 -20 0]'*ones(1,length(cal_points));
scatter3(cal_points(1,:),cal_points(2,:),cal_points(3,:))
hold on
scatter3(0,0,0)

% car hood surface
[X,Z] = meshgrid(-10:10,10:50);
Y = 1.5*sqrt((60-Z)) - 15;
surf(X,Y,Z)

syms z
y = 1.5*sqrt((60-z)) - 15;
slope = diff(y);

% get the normal at point z_point (b/w 10 and 50, independent of x and y coordinates)
z_point = 30;
slope = subs(slope,z,z_point);
slope = double(slope);
surf_normal = [0 1 -slope];
