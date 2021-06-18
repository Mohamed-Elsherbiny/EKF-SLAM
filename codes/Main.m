clc;clear;
% I. INITIALIZE
% I.1 SIMULATOR -- use capital letters for variable names
% W: set of external landmarks
%W = cloister(-4,4,-4,4,7); % Type 'help cloister' for help
load('W.mat')
% N: number of landmarks
N = size(W,2);
% R: robot pose [x ; y ; alpha]
R = [0;-2;0];
% U: control [d x ; d alpha]
U = [0.1 ; 0.05]; % fixing advance and turn increments creates a circle
% Y: measurements of all landmarks
Y = zeros(2, N);
% I.2 ESTIMATOR
% Map: Gaussian {x,P}
% x: state vector's mean
x = zeros(numel(R)+numel(W), 1);
% P: state vector's covariances matrix
P = zeros(numel(x),numel(x));
% System noise: Gaussian {0,Q}
q = [.01;0;.02]; % amplitude or standard deviation
Q = diag(q.^2); % covariances matrix
% Measurement noise: Gaussian {0,S}
s = [.1;1*pi/180]; % amplitude or standard deviation
S = diag(s.^2); % covariances matrix
% Map management
mapspace = false(1,numel(x)); % See Help Note #10 above.
% Landmarks management
landmarks = zeros(2, N); % See Help Note #11 above
% Place robot in map
r = find(mapspace==false, numel(R) ); % set robot pointer
mapspace(r) = true; % block map positions
x(r) = R; % initialize robot states
P(r,r) = 0; % initialize robot covariance
% I.3 GRAPHICS -- use the variable names of simulated and estimated
% variables, followed by a capital G to indicate 'graphics'.
% NOTE: the graphics code is long but absolutely necessary.
% Set figure and axes for Map
mapFig = figure(1); % create figure
cla % clear axes
axis([-6 6 -6 6]) % set axes limits
axis square % set 1:1 aspect ratio
% Simulated World -- set of all landmarks, red crosses
WG = line(...
'linestyle','none',...
'marker','+',...
'color','r',...
'xdata',W(1,:),...
'ydata',W(2,:));
% Simulated robot, red triangle
Rshape0 = .2*[...
2 -1 -1 2; ...
0 1 -1 0]; % a triangle at the origin
Rshape = fromFrame(R, Rshape0); % a triangle at the robot pose
RG = line(...
'linestyle','-',...
'marker','none',...
'color','r',...
'xdata',Rshape(1,:),...
'ydata',Rshape(2,:));
% Estimated robot, blue triangle
rG = line(...
'linestyle','-',...
'marker','none',...
'color','b',...
'xdata',Rshape(1,:),...
'ydata',Rshape(2,:));
% Estimated robot ellipse, magenta
reG = line(...
'linestyle','-',...
'marker','none',...
'color','m',...
'xdata',[ ],...
'ydata',[ ]);
% Estimated landmark means, blue crosses
lG = line(...
'linestyle','none',...
'marker','+',...
'color','b',...
'xdata',[ ],...
'ydata',[ ]);
% Estimated landmark ellipses, green
leG = zeros(1,N);
for i = 1:numel(leG)
leG(i) = line(...
'linestyle','-',...
'marker','none',...
'color','g',...
'xdata',[ ],...
'ydata',[ ]);
end
% II. TEMPORAL LOOP
for t = 1:200
% II.1 SIMULATOR
% a. motion
n = q .* randn(3,1); % perturbation vector
% R = move(R, U, zeros(2,1) ); % we will perturb the estimator
% instead of the simulator
% b. observations
for i = 1:N % i: landmark index
v = s .* randn(2,1); % measurement noise
z_t(:,i) = observe(R, W(:,i)) + v;
end
%% Inputs to EKF Algorithm
u_t = [U(1); 0 ;U(2)];
Sigma_t_1 = P;
Meu_t_1 = x;
R_t = Q; % Model noise
Q_t = S; % Measurement noise
%Meu_t_1(3) = atan2(sin(Meu_t_1(3)), cos(Meu_t_1(3)));
%% EKF
R = move(R,u_t);
[x, P] = Extended_Kalman_Filter(Meu_t_1,Sigma_t_1,u_t, z_t, R_t, Q_t);
l = reshape(x(4:end),[2,40]);
% II.3 GRAPHICS
% Simulated robot
if t >1
Rshape = fromFrame(R, Rshape0);
set(RG, 'xdata', Rshape(1,:), 'ydata', Rshape(2,:));
% Estimated robot
Rshape = fromFrame(x(r), Rshape0);
set(rG, 'xdata', Rshape(1,:), 'ydata', Rshape(2,:));
% Estimated robot ellipse
re = x(r(1:2)); % robot position mean
RE = P(r(1:2),r(1:2)); % robot position covariance
[xx,yy] = cov2elli(re,RE,3,16); % x- and y- coordinates of contour
set(reG, 'xdata', xx, 'ydata', yy);
% Estimated landmarks
lids = find(l(1,:)); % all indices of mapped landmarks
lx = l(1,lids);
ly = l(2,lids);
set(lG, 'xdata', lx, 'ydata', ly);
% Estimated landmark ellipses -- one per landmark
for i = lids
    
%l = landmarks(:,i);
le = l(:,i);
LE = P(i,i);
[xx,yy] = cov2elli(le,LE,3,16);
set(leG(i), 'xdata', xx, 'ydata', yy);
end
% force Matlab to draw all graphic objects before next iteration
drawnow
end
 pause(0.5)
end
