function [A_i] = DHmatrix(theta_i,alpha_i,a_i,d_i)
% This function creates a Denavit Hartenberg Transformation matrix A_i by
% using the input rotations theta_i and alpha_i, as well as the input
% displacements a_i and d_i for the i-th Denavit-Hartenberg frame.
    A_i(:,1) = [              cos(theta_i)               sin(theta_i)           0  0];
    A_i(:,2) = [-sin(theta_i)*cos(alpha_i)  cos(theta_i)*cos(alpha_i) sin(alpha_i) 0];
    A_i(:,3) = [ sin(theta_i)*sin(alpha_i) -cos(theta_i)*sin(alpha_i) cos(alpha_i) 0];
    A_i(:,4) = [          a_i*cos(theta_i)           a_i*sin(theta_i)          d_i 1];
end

