function [o,thetaX,thetaY,thetaZ] = ForwardKinematics6DOF(H_06,q1,q2,q3,q4,q5,q6,t)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
% Initilisation of task space variables
thetaX = sym(zeros(1,length(t)));
thetaY = sym(zeros(1,length(t)));
thetaZ = sym(zeros(1,length(t)));

syms t1 t2 t3 t4 t5 t6
T(t1,t2,t3,t4,t5,t6) = H_06(1:3,4);
R(t1,t2,t3,t4,t5,t6) = H_06(1:3,1:3);

% computing the position o = [ox,oy,oz]' of the end-effector 
o = cell2sym(T(q1,q2,q3,q4,q5,q6));

for i = 1 : length(t)
        r = R(q1(i),q2(i),q3(i),q4(i),q5(i),q6(i));
        
        % determine Euler angles from rotation matrix
        
        if (r(1,3) < +1) %xyz
            if (r(1,3) > -1)
                thetaY(i) = asin(r(1,3));
                thetaX(i) = atan2(-r(2,3),r(3,3));
                thetaZ(i) = atan2(-r(1,2),r(1,1));
                
            else % r(1,3) = -1
                % Not a unique solution: psi-phi = atan2(r(2,1),r(2,2))
                thetaY(i) = -pi/2;
                thetaX(i) = -atan2(r(2,1),r(2,2));
                thetaZ(i) = 0;
                
            end
            
        else % r(1,3) = -1
            % Not a unique solution: psi + phi = atan2(r(2,1),r(2,2))
            thetaY(i) = +pi/2;
            thetaX(i) = atan2(r(2,1),r(2,2));
            thetaZ(i) = 0;
            
        end
                
end

end

