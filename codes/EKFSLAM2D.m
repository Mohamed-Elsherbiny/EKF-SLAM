clc; close all; clear;
% EKF-SLAM in 2D
% Initialization

% Model evolution noise
q = [0.005;0;0.003];
Q = diag(q.^2);
% Measurement noise
m = [.25; 1*pi/180];
M = diag(m.^2);

% Loading landmarks
load('W.mat')

% System parameters
% R -> robot pose (x, y, theta)
% u -> control (delta_t, delta_theta)
% y -> map (W -> landmarks positions from enclosed file)

R = [0;-2.5;0];
u = [0.1;0.05];
y = zeros(2,size(W,2));

% EKF-state and covariance matrix
% state (robot pose and landmark positions)
x = zeros(numel(R)+numel(W), 1);
% covariance matrix
P = zeros(numel(x),numel(x));

mapspace = 1:numel(x);
l = zeros(2, size(W,2));
r = find(mapspace,numel(R));
mapspace(r) = 0;
x(r) = R;
P(r,r) = 0;

%%%
% Graphics initialization
mapFig = figure(1);
cla;
axis([-6 6 -6 6])
axis square

% Draw landmarks
WG = line('parent',gca,'linestyle','none','marker','o','color','r',...
    'xdata',W(1,:),'ydata',W(2,:));
% Draw initial robot position
RG = line('parent',gca,'marker','.','color','r',...
    'xdata',R(1),'ydata',R(2));
% Draw initial robot position estimation
rG = line('parent',gca,'linestyle','none','marker','+','color','b',...
    'xdata',x(r(1)),'ydata',x(r(2)));

% Initialize objects for future landmarks drawing
lG = line('parent',gca,'linestyle','none','marker','+','color','b',...
    'xdata',[],'ydata',[]);
eG = zeros(1,size(W,2));
for i = 1:numel(eG)
    eG(i) = line('parent', gca,'color','g','xdata',[],'ydata',[]);
end
reG = line('parent', gca,'color','m','xdata',[],'ydata',[]);
%%%
% Rotation matrix

% EKF-SLAM simulation
xpred = x;
Ppred = P;
j = 1;
z_t = zeros(2,40);
for t = 1:200
    
    % TODO Simulate robot movement and landmarks observation
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
    %%
    % TODO Extended Kalman Filter
    % Prediction and correction with known landmarks
    
    %%%
    
    % TODO Check if there is new landmarks and addition to the map
    
    
%% Simulate sensor
% Measurement noise: Gaussian f0,Sg
s = [.1;1*pi/180]; % amplitude or standard deviation
S = diag(s.^2); % covariances matrix
N = size(W,2) ;
for i = 1:N % i: landmark index
v = s .* randn(2,1); % measurement noise
% measurement in robot frame
z_t(:,i) = Range_Bearing_Observation(R, W(:,i)) + v;
end
%% Inputs to EKF Algorithm
u_t = [u(1); 0 ;u(2)];
Sigma_t_1 = P;
Meu_t_1 = x;
R_t = Q; % Model noise
Q_t = M; % Measurement noise
Meu_t_1(3) = atan2(sin(Meu_t_1(3)), cos(Meu_t_1(3)));
%% EKF
R = move(R,u_t);
[x, P] = Extended_Kalman_Filter(Meu_t_1,Sigma_t_1,u_t, z_t, R_t, Q_t);
[xpred,Ppred] = EKF_SLAM_Prediction(xpred,Ppred,u_t, N, R_t);
l = reshape(x(4:end),[2,40]);
xprediction(t) =  xpred(1);
xcorrection(t) = x(1);
xactual(t) = R(1);
yactual(t) = R(2);
ycorrection(t) = x(2);
theta_a(t) = R(3);
theta_c(t) = x(3);
xcov(t) = P(1,1);
deltax(t) = abs(x(1) - xpred(1));
deltay(t) = abs(x(2) - xpred(2));
error_in_landmark(t) = sum(abs(x(4:end) - reshape(W,[80,1]))) ;
errorinlandmarks = [x(4:2:end), W(1,:)',x(5:2:end),W(2,:)'];
%%
    
    %%%
    % Graphics drawing
    set(RG, 'xdata', R(1), 'ydata', R(2));
    set(rG, 'xdata', x(r(1)), 'ydata', x(r(2)));
    lids = find(l(1,:));
    lx = l(1,lids);
    ly = l(2,lids);
    set(lG, 'xdata', lx, 'ydata', ly);
     if t > 1
    for lid = lids
        le = l(:,lid);
        LE = P(lid,lid);
        [X,Y] = cov2elli(le,LE,3,16);
        set(eG(lid),'xdata',X,'ydata',Y);
    end
   
        re = x(r(1:2));
        RE = P(r(1:2),r(1:2));
       [X,Y] = cov2elli(re,RE,3,16);
       set(reG,'xdata',X,'ydata',Y);
    end
    drawnow;
  
    %%%
end
t= 1:1:200;
figure
plot(t, xactual)
hold on
plot(t, xcorrection)
legend('X_{actual}','X_{EKF}')
xlabel('time')
figure
subplot(1,2,1)
plot(t, yactual, 'linewidth',2)
hold on
plot(t, ycorrection, 'linewidth',2)
legend('Y_{actual}','Y_{EKF}')
xlabel('time')
subplot(1,2,2)
plot(t, theta_a, 'linewidth',2)
hold on
plot(t, theta_c, 'linewidth',2)
legend('Y_{actual}','Y_{EKF}')
xlabel('time')
figure
subplot(1,2,1)
plot(t, yactual - ycorrection, 'linewidth',2)
title('Error in y position')
xlabel('time')
subplot(1,2,2)
plot(t, theta_a - theta_c, 'linewidth',2)
xlabel('time')
title('Error in \theta ')
% figure
% plot(t, deltax)
% hold on
% plot(t, deltay)
% legend('\delta_x','\delta_y')
% xlabel('time')