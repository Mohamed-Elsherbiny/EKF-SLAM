%  ============================================================================
%  Name        : Lab - Lecture Master SAAS - Sensor Fusion - UEVE 
%  Author      : Mohamed Elsherbiny
%  Version     : ver 1.0
%  Copyright   : For free
%  Description : EKF SLAM prediction Step
%  Note        : -  
%  ===========================================================================
function [Meu_bar_t,Sigma_bar_t] = EKF_SLAM_Prediction(Meu_t_1,Sigma_t_1,u_t, N, R_t)
% Extended Kalman Filter predicition for both mean and covariance 
% In:
% Meu_t_1 : mean @ time = t-1
% Sigma_t_1 : Covariance @ time = t-1
% u_t : control input @ time = t 
% N : no of landmarks
% R_t : random variables representing the process covariance noise
% Out:
% Meu_bar_t : predicted mean @ t = t
% Sigma_bar_t: predicted Covariance @ time = t

% Rotation matrix for motion model
theta = Meu_t_1(3);
Rot = [cos(theta) -sin(theta) 0;
            sin(theta) cos(theta) 0;
            0 0 1];

Fx = [eye(3) zeros(3,2*N)];  % to make the input affect the robot only (map the input)
% Motion 
Meu_bar_t = Meu_t_1 + Fx' * Rot * u_t; % Landmarks are static, so they haven't motion model

% Update Covariance 
G_x = [1, 0, -u_t(1)*sin(theta);
       0, 1,  u_t(1)*cos(theta);
       0, 0,              1];
G_t = [G_x,                zeros(3,2*N);
           zeros(2*N,3),   eye(2*N,2*N)];

cov = [R_t              zeros(3,2*N);
        zeros(2*N,3)   eye(2*N,2*N)];
Sigma_bar_t = G_t * Sigma_t_1 * G_t' + cov;

end

