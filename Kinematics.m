clearvars, clc
%% Initialisation

displayDHmatrices = true;
displayRmatrices = true;
calculateJacobian = false;
displayJacobian = false;
calculateForwardKinematics = true;
calculateInverseKinematics = true;
checkInverseKinematics = false;
displayInverseKinematicsCheck = true;
showSimulation = true;

% Initialisation of symbolic variables theta_i
syms t1 t2 t3 t4 t5 t6

theta_1 = t1 + sym(pi/2);
theta_2 = t2 - sym(pi/2);
theta_3 = t3;
theta_4 = t4;
theta_5 = t5;
theta_6 = t6;

%Initialisation of alpha_i
alpha_1 = sym(-pi/2);
alpha_2 =     sym(0);
alpha_3 = sym(-pi/2);
alpha_4 = sym(+pi/2);
alpha_5 = sym(-pi/2);
alpha_6 =     sym(0);

%Initialisation of linke lengths and offsets a_i and d_i
a_1 = sym(0.025); d_1 =  sym(0.400);
a_2 = sym(0.560); d_2 =      sym(0);
a_3 = sym(0.025); d_3 =      sym(0);
a_4 =     sym(0); d_4 =  sym(0.515);
a_5 =     sym(0); d_5 =      sym(0);
a_6 =     sym(0); d_6 =  sym(0.090);

a = [a_1, a_2, a_3, a_4, a_5, a_6];
d = [d_1, d_2, d_3, d_4, d_5, d_6];

%% Declaring the DH matrices

A_1 = DHmatrix(theta_1,alpha_1,a_1,d_1);
A_2 = DHmatrix(theta_2,alpha_2,a_2,d_2);
A_3 = DHmatrix(theta_3,alpha_3,a_3,d_3);
A_4 = DHmatrix(theta_4,alpha_4,a_4,d_4);
A_5 = DHmatrix(theta_5,alpha_5,a_5,d_5);
A_6 = DHmatrix(theta_6,alpha_6,a_6,d_6);

%% Computing the homogeneous transformation matrices

H_01 = A_1;
H_02 = A_1*A_2;
H_03 = A_1*A_2*A_3;
H_04 = A_1*A_2*A_3*A_4;
H_05 = A_1*A_2*A_3*A_4*A_5;
H_36 = A_4*A_5*A_6;
H_06 = A_1*A_2*A_3*A_4*A_5*A_6;

%% Defining the corresponding relevant rotation matrices

R_03 = H_03(1:3,1:3);
R_36 = H_36(1:3,1:3);
R_06 = H_06(1:3,1:3);

%% Determining the Jacobian matrix

if calculateJacobian

    % Determining z_j
    z_0 = [0, 0, 1]';
    z_1 = H_01(1:3,3);
    z_2 = H_02(1:3,3);
    z_3 = H_03(1:3,3);
    z_4 = H_04(1:3,3);
    z_5 = H_05(1:3,3);

    % Determining o_j
    o_0 = [0, 0, 0]';
    o_1 = H_01(1:3,4);
    o_2 = H_02(1:3,4);
    o_3 = H_03(1:3,4);
    o_4 = H_04(1:3,4);
    o_5 = H_05(1:3,4);
    o_6 = H_06(1:3,4);

    % Determining J_i
    J_1 = [cross(z_0, o_6-o_0); z_0];
    J_2 = [cross(z_1, o_6-o_1); z_1];
    J_3 = [cross(z_2, o_6-o_2); z_2];
    J_4 = [cross(z_3, o_6-o_3); z_3];
    J_5 = [cross(z_4, o_6-o_4); z_4];
    J_6 = [cross(z_5, o_6-o_5); z_5];

    % Jacobian matrix
    J = [J_1, J_2, J_3, J_4, J_5, J_6];

end

%% Display DH matrices

if displayDHmatrices
    disp('A_1 = '), disp(simplify(A_1))
    disp('A_2 = '), disp(simplify(A_2))
    disp('A_3 = '), disp(simplify(A_3))
    disp('A_4 = '), disp(simplify(A_4))
    disp('A_5 = '), disp(simplify(A_5))
    disp('A_6 = '), disp(simplify(A_6))
    disp('H_06 = '), disp(simplify(H_06))
    
end


%% Display Rotation matrices

if displayRmatrices
    disp('R_03 = '), disp(simplify(R_03))
    disp('R_36 = '), disp(simplify(R_36))
    disp('R_06 = '), disp(simplify(R_06))
    
end

%% Display Jacobian
if calculateJacobian
    if displayJacobian
        disp('J_1 = '), disp(simplify(J_1))
        disp('J_2 = '), disp(simplify(J_2))
        disp('J_3 = '), disp(simplify(J_3))
        disp('J_4 = '), disp(simplify(J_4))
        disp('J_5 = '), disp(simplify(J_5))
        disp('J_6 = '), disp(simplify(J_6))
        disp('J = '), disp(simplify(J))
        
    end
    
end
%% Plotting the forward kinematics (position and orientation)

if calculateForwardKinematics
    t = 0:100;
    t1_val = sym(0*t); %sym(deg2rad(15).*sin(0.1*t)); 
    t2_val = sym(deg2rad(15).*sin(0.1*t));
    t3_val = sym(0*t); %sym(deg2rad(15).*sin(0.2*t));
    t4_val = sym(0*t); %sym(deg2rad(15).*sin(0.1*t));
    t5_val = sym(0*t); %sym(deg2rad(15).*sin(0.1*t));
    t6_val = sym(0*t); %sym(deg2rad(15).*sin(0.1*t));
    
    [o, thetaX, thetaY, thetaZ] = ForwardKinematics6DOF(H_06,t1_val,t2_val,t3_val,t4_val,t5_val,t6_val,t);
    disp('Forward Kinematics: Done')
    
end

%% Calculating the inverse kinematics (position and orientation)
if calculateInverseKinematics
    [q1,q2,q3,q4,q5,q6] = InverseKinematicsKUKA(R_03,a,d,o,thetaX,thetaY,thetaZ,t);
    Q = [q1; q2; q3; q4; q5; q6];
    disp('Inverse Kinematics: Done')
end

%% Plotting the forward and inverse kinematics
%RR = sym('r%d%d',[3 3],'real');
%RRR = R_03.'*RR;

figure, clf, hold on

subplot(6,3,1),  t1plot = plot(t,t1_val);  title('\theta_1');
subplot(6,3,2), xplot  = plot(t,o(1,:)); title ('x');
subplot(6,3,4),  t2plot = plot(t,t2_val);  title('\theta_2');
subplot(6,3,5), yplot  = plot(t,o(2,:)); title ('y');
subplot(6,3,7),  t3plot = plot(t,t3_val);  title('\theta_3');
subplot(6,3,8), zplot  = plot(t,o(3,:)); title ('z');
subplot(6,3,10), t4plot = plot(t,t4_val);  title('\theta_4');
subplot(6,3,11), txplot = plot(t,thetaX); title ('\theta_x');
subplot(6,3,13), t5plot = plot(t,t5_val);  title('\theta_5');
subplot(6,3,14), tylot  = plot(t,thetaY); title ('\theta_y');
subplot(6,3,16), t6plot = plot(t,t6_val);  title('\theta_6');
subplot(6,3,17), tzplot = plot(t,thetaZ); title ('\theta_z');
subplot(6,3,3),  q1plot = plot(t,q1); title('q_1');
subplot(6,3,6),  q2plot = plot(t,q2); title('q_2');
subplot(6,3,9),  q3plot = plot(t,q3); title('q_3');
subplot(6,3,12), q4plot = plot(t,q4); title('q_4');
subplot(6,3,15), q5plot = plot(t,q5); title('q_5');
subplot(6,3,18), q6plot = plot(t,q6); title('q_6');
sgtitle('Forward Kinematics to Inverse Kinematics')
disp('Done')

hold off

%% Check Inverse Kinematics
if checkInverseKinematics
    % Compute forward kinematics using the outcome from the inverse kinematics
    [o2, thetaX2, thetaY2, thetaZ2] = ForwardKinematics6DOF(H_06,q1,q2,q3,q4,q5,q6,t);
    
    if (o == o2 && thetaX == thetaX2 && thetaY == thetaY2 && thetaZ == thetaZ2)
        check = true;
        disp('The inverse kinematics check has been verified by computing the corresponding forward kinematics.')
        
    end
    
    % Plot the task space coordinates so that they can be compared to the
    % input of the inverse kinematics operation.
    if displayInverseKinematicsCheck
        figure, clf, hold on
        subplot(6,1,1), x2plot  = plot(t,double(o2(1,:))); title ('x');
        subplot(6,1,2), y2plot  = plot(t,double(o2(2,:))); title ('y');
        subplot(6,1,3), z2plot  = plot(t,double(o2(3,:))); title ('z');
        subplot(6,1,4), tx2plot = plot(t,thetaX2); title ('\theta_x');
        subplot(6,1,5), ty2plot = plot(t,thetaY2); title ('\theta_y');
        subplot(6,1,6), tz2plot = plot(t,thetaZ2); title ('\theta_z');
        sgplot('Forward kinematics computed on joint space received from the inverse kinematics')
        hold off
        
    end
    
end
%% Simulation
if showSimulation
    SimulateMovement(Q,o,t)
end
