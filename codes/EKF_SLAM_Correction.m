%  ============================================================================
%  Name        : Lab - Lecture Master SAAS - Sensor Fusion - UEVE 
%  Author      : Mohamed Elsherbiny
%  Version     : ver 1.0
%  Copyright   : For free
%  Description : EKF SLAM correction Step
%  Note        : -  
%  ===========================================================================
function [Meu_t, Sigma_t] = EKF_SLAM_Correction(Meu_bar_t, Sigma_bar_t, z_t, Q_t)
% Extended Kalman Filter correction 
    % Known data association
    % i-th measurement observes the landmark with index j
    % Initialize landmark if unobserved
    % Compute the expected observation
    % Compute the Jacobian of h
    % Then, proceed with computing the Kalman gain
% In:
% Meu_bar_t : mean from prediction step
% Sigma_t_1 : Covariance from predection step
% z_t : measurement 
% Out:
% Meu_t : the corrected mean using kalman gain
% Sigma_t: the corrected Covariance using kalman gain
N = size(z_t,2) ;
meu = [Meu_bar_t(1); Meu_bar_t(2)];
meu_theta = Meu_bar_t(3);
for i=1: length(z_t)
% Compute expected observation according to the current estimate 
delta = Meu_bar_t(4+2*(i-1):4+2*(i-1)+1) - meu;
q = delta' * delta;
deltax = delta(1);
deltay = delta(2);  
z_hat_t = [sqrt(q);
           atan2(deltay,deltax) - meu_theta] ;
% Compute the Jacobian for each observation
h = 1/q  * [-sqrt(q)*deltax, -sqrt(q)*deltay, zeros(size(q)), sqrt(q)*deltax, sqrt(q)*deltay;
                 deltay, -deltax, -q, -deltay, deltax];
% map the jacobian to the high dimentional space
F_x1 = [eye(3),     zeros(3,2*i-2), zeros(3,2), zeros(3,2*N - 2*i)];
F_x2 = [zeros(2,3), zeros(2,2*i-2), eye(2),     zeros(2,2*N - 2*i)];
F_x = [F_x1;
       F_x2];
H = h * F_x;
psi = H * Sigma_bar_t * H' + Q_t;
% innovation
z_dash = (z_t(:,i) - z_hat_t);
if z_dash(2) > pi
z_dash(2) = z_dash(2) - 2*pi;
end
if z_dash(2) < -pi
z_dash(2) = z_dash(2) + 2*pi;
end
    if z_dash' * psi^-1 * z_dash < 9
        % Kalman Gain 
        K = Sigma_bar_t * H' * psi^-1;
        Meu_bar_t = Meu_bar_t + K * z_dash;
        I = eye(size(Sigma_bar_t));
        Sigma_bar_t = (I - K * H ) * Sigma_bar_t;
    end
end
Meu_t = Meu_bar_t;
Sigma_t =  Sigma_bar_t;
end

