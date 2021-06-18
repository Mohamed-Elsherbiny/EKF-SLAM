function [z_t] = Range_Bearing_Observation(robot, landmark)
% This function simulate the range sensor  
% In:
% robot : robot position and orientation in reference frame [x, y, theta]
% landmark : landmark x, y position
% Out:
% z_t : sensor measurement in robot frame
RobotPosition = robot(1:2);
theta = robot(3);
% 1-  Transform the landmark to the robot frame
R = [cos(theta) -sin(theta) ; sin(theta) cos(theta)];
xy = R' * (landmark - RobotPosition);
x = xy(1);
y = xy(2);
% 2- Transform cartesian to polar
z_t = [ sqrt(x^2 + y^2);
        atan2(y,x)];
end

