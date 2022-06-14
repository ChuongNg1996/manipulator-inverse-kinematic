% Clean
clc;
clear;

% Base
alpha_base = 0;         a_base = 0;         d_base = 0;

% Joint 1
alpha_0 = pi/2;         a_0 = 0;            d_1 = 0.15185;

% Joint 2
alpha_1 = 0;            a_1 = -0.24355;     d_2 = 0;

% Joint 3
alpha_2 = 0;            a_2 = -0.2132;      d_3 = 0;

% % Joint 4
% alpha_3 = pi/2;         a_3 = 0;            d_4 = 0.13105;
% 
% % Joint 5
% alpha_4 = -pi/2;        a_4 = 0;            d_5 = 0.08535;
% 
% 
% % Joint 6
% alpha_5 = 0;            a_5 = 0;            d_6 = 0.0921;

% Angle variable
syms theta_base theta_1 theta_2 theta_3 

X_END = -0.01;
Y_END = 0;
Z_END = -0.01;

T_world_base = [cos(theta_base) -sin(theta_base) 0 a_base;
            cos(alpha_base)*sin(theta_base) cos(alpha_base)*cos(theta_base) -sin(alpha_base) -sin(alpha_base)*d_base;
            sin(alpha_base)*sin(theta_base) sin(alpha_base)*cos(theta_base) cos(alpha_base) cos(alpha_base)*d_base;
            0 0 0 1];

T_base_1 = [cos(theta_1) -sin(theta_1) 0 a_0;
            cos(alpha_0)*sin(theta_1) cos(alpha_0)*cos(theta_1) -sin(alpha_0) -sin(alpha_0)*d_1;
            sin(alpha_0)*sin(theta_1) sin(alpha_0)*cos(theta_1) cos(alpha_0) cos(alpha_0)*d_1;
            0 0 0 1];

T_1_2 = [cos(theta_2) -sin(theta_2) 0 a_1;
            cos(alpha_1)*sin(theta_2) cos(alpha_1)*cos(theta_2) -sin(alpha_1) -sin(alpha_1)*d_2;
            sin(alpha_1)*sin(theta_2) sin(alpha_1)*cos(theta_2) cos(alpha_1) cos(alpha_1)*d_2;
            0 0 0 1];

T_2_3 = [cos(theta_3) -sin(theta_3) 0 a_2;
            cos(alpha_2)*sin(theta_3) cos(alpha_2)*cos(theta_3) -sin(alpha_2) -sin(alpha_2)*d_3;
            sin(alpha_2)*sin(theta_3) sin(alpha_2)*cos(theta_3) cos(alpha_2) cos(alpha_2)*d_3;
            0 0 0 1];


T_world_3 = T_world_base*T_base_1*T_1_2*T_2_3;
T_world_END = T_world_3*[X_END;Y_END;Z_END;1];
end_pose = subs(T_world_END,[theta_base, theta_1, theta_2,theta_3], [1,1,1,1]);
double(end_pose(1))
%{
    https://www.mathworks.com/help/symbolic/subs.html 
    https://www.mathworks.com/matlabcentral/answers/359238-output-long-numbers-for-no-reason
    -> Use double.
    
%}