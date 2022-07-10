% Clean
clc;
clear;


%% Modified D-H 

L0 = 270.35;
L1 = 69.00;
L2 = 364.35;
L3 = 69.00;
L4 = 374.29;
L5 = 10.00;
L6 = 368.30;

theta_2_offset = -pi/2;
theta_3_offset = -atan(L3/L2);
theta_4_offset = -atan(L5/L4);

% Joint 1
alpha_0 = 0;            a_0 = 0;            d_1 = 0;

% Joint 2
alpha_1 = -pi/2;         a_1 = L1;            d_2 = 0;

% Joint 3
alpha_2 = 0;            a_2 = sqrt(L2^2+L3^2);      d_3 = 0;

% Joint 4
alpha_3 = 0;            a_3 = sqrt(L4^2+L5^2);       d_4 = 0;

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
Y_END = L6;
Z_END = 0;

T_0_1 = [   cos(theta_1) -sin(theta_1) 0 a_0;
            cos(alpha_0)*sin(theta_1) cos(alpha_0)*cos(theta_1) -sin(alpha_0) -sin(alpha_0)*d_1;
            sin(alpha_0)*sin(theta_1) sin(alpha_0)*cos(theta_1) cos(alpha_0) cos(alpha_0)*d_1;
            0 0 0 1];

T_1_2 = [   cos(theta_2) -sin(theta_2) 0 a_1;
            cos(alpha_1)*sin(theta_2) cos(alpha_1)*cos(theta_2) -sin(alpha_1) -sin(alpha_1)*d_2;
            sin(alpha_1)*sin(theta_2) sin(alpha_1)*cos(theta_2) cos(alpha_1) cos(alpha_1)*d_2;
            0 0 0 1];

T_2_3 = [   cos(theta_3) -sin(theta_3) 0 a_2;
            cos(alpha_2)*sin(theta_3) cos(alpha_2)*cos(theta_3) -sin(alpha_2) -sin(alpha_2)*d_3;
            sin(alpha_2)*sin(theta_3) sin(alpha_2)*cos(theta_3) cos(alpha_2) cos(alpha_2)*d_3;
            0 0 0 1];

T_3_4 = [   cos(theta_4) -sin(theta_4) 0 a_3;
            cos(alpha_3)*sin(theta_4) cos(alpha_3)*cos(theta_4) -sin(alpha_3) -sin(alpha_3)*d_4;
            sin(alpha_3)*sin(theta_4) sin(alpha_3)*cos(theta_4) cos(alpha_3) cos(alpha_3)*d_4;
            0 0 0 1];

T_0_3 = T_0_1*T_1_2*T_2_3*T_3_4;
T_0_END = T_0_3*[X_END;Y_END;Z_END;1];

%% Forward Kinematic
x_fk = T_0_END(1);
y_fk = T_0_END(2);
z_fk = T_0_END(3);

%% Jacobian 
J(1,1) = diff(x_fk,theta_1);
J(1,2) = diff(x_fk,theta_2);
J(1,3) = diff(x_fk,theta_3);
J(1,4) = diff(x_fk,theta_4);

J(2,1) = diff(y_fk,theta_1);
J(2,2)= diff(y_fk,theta_2);
J(2,3) = diff(y_fk,theta_3);
J(2,4)= diff(y_fk,theta_4);

J(3,1) = diff(z_fk,theta_1);
J(3,2) = diff(z_fk,theta_2);
J(3,3) = diff(z_fk,theta_3);
J(3,4) = diff(z_fk,theta_4);

inv_J = pinv(J);

%% Inverse Kinematic
% goal = [0.4; 0.1; 0.2];
% init_joint = [0.1,0.1,0.1,0.1];
% init_pos = double(subs(T_0_END,[theta_1, theta_2,theta_3, theta_4], init_joint));
% iteration = 100;
% step_x = (goal(1) - init_pos(1))/iteration;
% step_y = (goal(2) - init_pos(2))/iteration;
% step_z = (goal(3) - init_pos(3))/iteration;
% current_joint = init_joint;
% x_plot = zeros(1, iteration);
% y_plot = zeros(1, iteration);
% z_plot = zeros(1, iteration);
% for i = 1:iteration
%     x_plot(i) = double(subs(x_fk,[theta_1, theta_2,theta_3, theta_4], current_joint)); 
%     y_plot(i) = double(subs(y_fk,[theta_1, theta_2,theta_3, theta_4], current_joint)); 
%     z_plot(i) = double(subs(z_fk,[theta_1, theta_2,theta_3, theta_4], current_joint));
%     
%     inv_J_result = double(subs(inv_J,[theta_1, theta_2,theta_3, theta_4], current_joint));
%     joint_step = inv_J_result*[step_x;step_y;step_z];
%     current_joint = current_joint + transpose(joint_step);
% end
% grid on
% plot3(x_plot,y_plot,z_plot);

% end_pose = subs(T_0_END,[theta_1, theta_2,theta_3, theta_4], [0,0,0,0]);
% double(end_pose(1))
% double(end_pose(2))
% double(end_pose(3))
% double(end_pose(4))

%{
    https://www.mathworks.com/help/symbolic/subs.html 
    https://www.mathworks.com/matlabcentral/answers/359238-output-long-numbers-for-no-reason
    -> Use double.
    https://www.mathworks.com/matlabcentral/answers/62992-to-take-the-partial-derivative-of-a-function-using-matlab
    [0,0,0,0] = [0.4567, 0, 0.1519, 1]
    pseudoinverse method: https://www.intechopen.com/chapters/57610 
    https://cseweb.ucsd.edu/classes/sp16/cse169-a/slides/CSE169_09.pdf
    
    Joints at 0 are very likely to cause DIVISION by 0 (Singularity) in
    inverse Jacobian matrix. Thus, init position should avoid 0 and there
    should be a condition in the iteration loop to avoid 0 position.
%}