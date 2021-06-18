%  ============================================================================
%  Name        : Lab - Lecture Master SAAS - Sensor Fusion - UEVE 
%  Author      : Mohamed Elsherbiny
%  Version     : ver 1.0
%  Copyright   : For free
%  Description : EKF Algorithm
%  Note        : -  
%  ===========================================================================
function [Meu_t,Sigma_t] = Extended_Kalman_Filter(Meu_t_1,Sigma_t_1,u_t, z_t, R_t, Q_t)
% Extended Kalman Filter for 2d model
% In:
% Meu_t_1 : mean @ time = t-1
% Sigma_t_1 : Covariance @ time = t-1
% u_t : control input @ time = t 
% z_t : range and bearing measurement
% Out:
% Meu_t : mean @ t = t
% Sigma_t: Covariance @ time = t
%% Initialise variables
N = size(z_t,2) ;

%% Prediction Step
[Meu_bar_t,Sigma_bar_t] = EKF_SLAM_Prediction(Meu_t_1,Sigma_t_1,u_t, N, R_t);

%% Correction Step
[Meu_t, Sigma_t] = EKF_SLAM_Correction(Meu_bar_t, Sigma_bar_t, z_t, Q_t);
end

