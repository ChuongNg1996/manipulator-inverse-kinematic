% Clean
clc;
clear;

% Modified D-H 

% Joint 1
alpha_0 = 0;            a_0 = 0;            d_1 = 0.15185;

% Joint 2
alpha_1 = pi/2;         a_1 = 0;            d_2 = 0;

% Joint 3
alpha_2 = 0;            a_2 = 0.24355;      d_3 = 0;

% Joint 4
alpha_3 = 0;            a_3 = 0.2132;       d_4 = 0;

% Joint 4
% alpha_3 = pi/2;         a_3 = 0;            d_4 = 0.13105;
% 
% % Joint 5
% alpha_4 = -pi/2;        a_4 = 0;            d_5 = 0.08535;
% 
% 
% % Joint 6
% alpha_5 = 0;            a_5 = 0;            d_6 = 0.0921;

% Angle variable
syms theta_1 theta_2 theta_3 theta_4

X_END = 0;
Y_END = 0;
Z_END = 0;

T_0_1 = [cos(theta_1) -sin(theta_1) 0 a_0;
            cos(alpha_0)*sin(theta_1) cos(alpha_0)*cos(theta_1) -sin(alpha_0) -sin(alpha_0)*d_1;
            sin(alpha_0)*sin(theta_1) sin(alpha_0)*cos(theta_1) cos(alpha_0) cos(alpha_0)*d_1;
            0 0 0 1];

T_1_2 = [cos(theta_2) -sin(theta_1) 0 a_1;
            cos(alpha_1)*sin(theta_2) cos(alpha_1)*cos(theta_2) -sin(alpha_1) -sin(alpha_1)*d_2;
            sin(alpha_1)*sin(theta_2) sin(alpha_1)*cos(theta_2) cos(alpha_1) cos(alpha_1)*d_2;
            0 0 0 1];

T_2_3 = [cos(theta_3) -sin(theta_3) 0 a_2;
            cos(alpha_2)*sin(theta_3) cos(alpha_2)*cos(theta_3) -sin(alpha_2) -sin(alpha_2)*d_3;
            sin(alpha_2)*sin(theta_3) sin(alpha_2)*cos(theta_3) cos(alpha_2) cos(alpha_2)*d_3;
            0 0 0 1];

T_3_4 = [cos(theta_3) -sin(theta_3) 0 a_3;
            cos(alpha_3)*sin(theta_3) cos(alpha_3)*cos(theta_3) -sin(alpha_3) -sin(alpha_3)*d_4;
            sin(alpha_3)*sin(theta_3) sin(alpha_3)*cos(theta_3) cos(alpha_3) cos(alpha_3)*d_4;
            0 0 0 1];
% 
% T_3_4 = [cos(theta_4) -sin(theta_4) 0 a_3;
%             cos(alpha_3)*sin(theta_4) cos(alpha_3)*cos(theta_4) -sin(alpha_3) -sin(alpha_3)*d_4;
%             sin(alpha_3)*sin(theta_4) sin(alpha_3)*cos(theta_4) cos(alpha_3) cos(alpha_3)*d_4;
%             0 0 0 1];

T_0_3 = T_0_1*T_1_2*T_2_3*T_3_4;
T_0_END = T_0_3*[X_END;Y_END;Z_END;1];
end_pose = subs(T_0_END,[theta_1, theta_2,theta_3, theta_4], [0,0,0,0]);
double(end_pose(1))
double(end_pose(2))
double(end_pose(3))
double(end_pose(4))
%{
    https://www.mathworks.com/help/symbolic/subs.html 
    https://www.mathworks.com/matlabcentral/answers/359238-output-long-numbers-for-no-reason
    -> Use double.
    
%}