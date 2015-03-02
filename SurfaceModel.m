% Get points on the plane
% img_fn = 'photos/frame0020.jpg';
% plane_points = calib_scene(img_fn);

% Cal=imread(img_fn);
% Cgray = histeq(rgb2gray(Cal));
% figure; imshow(Cgray)
% hold on

% camera matrix
K = [1417.989551681438 0 806.4020103873095;
     0 1415.627353858698 569.0503053003386;
     0 0 1];
 
% focal length, can it be found?? Not sure if needed in px or mm
f = (K(1,1) + K(2,2))/2;
% image center
C_ = K(1:2,3);

% directions for the lines through point 10, Replace with automatic
p0 = plane_points(:,69);
dp1 = p0 - plane_points(:,70); % horizontal line
dp2 = p0 - plane_points(:,60); % vertical line

% the calibration line points were found manually
l1 = [651 690 729 765;
      1026 1028 1028 1031]; %horizontal line 
l2 = [727 729 729 729;
      1064 1046 1028 1014]; %vertical line
p0_ =[729 1028]';

l1_coeffs = polyfit(l1(1,:),l1(2,:),3);
l2_coeffs = polyfit(l2(1,:),l2(2,:),3);

l1_x = min(l1(1,:)):0.5:max(l1(1,:));
l2_x = min(l2(1,:)):0.5:max(l2(1,:));
l1_y = polyval(l1_coeffs,l1_x); % DANGER POOR CONDITIONING,VERTICAL LINES.
l2_y = polyval(l2_coeffs,l2_x);

dx1 = diff(l1_x);
dy1 = diff(l1_y);
dx2 = diff(l2_x);
dy2 = diff(l2_y);

dydx1 = dy1./dx1;
dydx2 = dy2./dx2;
q_dot1 = dydx1(find(l1_x(2:end) == p0_(1)));
q_dot2 = dydx2(find(l2_x(2:end) == p0_(1))); 
% q_dot1 = dydx1(find(l1_x(1:end-1) == p0(1)));
% q_dot2 = dydx2(find(l2_x(1:end-1) == p0(1))); 

% am I misunderstanding? do you have to use three coordinates?
ratio = norm(q_dot1)^2/norm(q_dot2)^2;

v = n;% cal board normal from calibration code

s_vec = 1:0.5:30; % vector of s values to try
d = [p0_-C_; f]/norm([p0_-C_; f]); % relies on focal length but its unknown
l = f; % probably wrong units, should be in m
eqn_values = zeros(1,length(s_vec));

for idx = 1:length(s_vec)
    s = s_vec(idx);
    
    T = (l/(s*d'*v))*(eye(3) - d*v'/(d'*v));
    theta = acos(sqrt((s-d'*plane_points(:,65))/(2*norm(s*d)) + 1));
    
    % B_j
    Bu_j = (dp1(3)*cos(theta)*sin(theta) - dp1(1)*cos(theta))/norm(p0_);
    Bv_j = -dp1(2)/norm(p0_);
    B_j = [Bu_j Bv_j]';
    
    % B_k
    Bu_k = (dp2(3)*cos(theta)*sin(theta) - dp2(1)*cos(theta))/norm(p0_);
    Bv_k = -dp2(2)/norm(p0_);
    B_k = [Bu_k Bv_k]';
    
    % V
    h_j = [Bv_j -Bu_j*q_dot1 Bu_j-Bv_j*q_dot1]';
    h_k = [Bv_k -Bu_k*q_dot2 Bu_k-Bv_k*q_dot2]';
    h = cross(h_k,h_j);
    V = 1/(h(1)*h(2)-h(3)^2)*[h(2) h(3); h(3) h(1)];
    
    % the equation which we seek the roots of
    eqn_values(idx) = ([B_k'*V' 0]*T'*T*[V*B_k; 0])/([B_j'*V' 0]*T'*T*[V*B_j; 0]) - ratio;
end
[M,I] = min(abs(eqn_values)); % does not reach zero
s = s_vec(I)
% s is found where plot intersects axis
plot(s_vec,abs(eqn_values))

hood_normal = cross((p0 - (s-norm(p0-s*d))*d)/norm(p0-s*d),v)
 

% subplot(311); plot(x1_plot(1:end-1),dydx1)
% % distortion 
% distortion = [-0.2084393650486083 0.1200345054099712 -0.002414452646358044 0.001394735574824813 0];
% cameraParams = cameraParameters('IntrinsicMatrix', K, 'RadialDistortion', distortion(1:2), 'TangentialDistortion', distortion(3:4));
% 
%  J = undistortImage(Cgray, cameraParams, 'OutputView', 'full');  %
%  distorts the image even more?!
%  figure; imshowpair(Cgray, J, 'montage')
% compute neighbours
% compute lines
% compute line directions


% 
