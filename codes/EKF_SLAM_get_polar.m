function [z_hat_t] = EKF_SLAM_get_polar(meu,z)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
delta = z - meu;
q = delta' * delta;
deltax = delta(1);
deltay = delta(2);
z_hat_t = [sqrt(q);
           atan2(deltay,deltax)] ;
end

