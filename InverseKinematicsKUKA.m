function [q1,q2,q3,q4,q5,q6] = InverseKinematicsKUKA(R_03,a,d,o,thetaX,thetaY,thetaZ,t)
% This function performs the inverse kinematics for the KR10 R1100-2
% robotic arm. The transformation matrix R03, the link offsets and the task
% space variables [o_x, o_y, o_z, thetaX, thetaY, thetaZ]' are taken as
% input variables, as well as a vector t that represents the time instances
% that are simulated. The output of the function are the joint space
% variables q1 to q6, which represent the angles theta of the 6 robotic
% joints.
    % Initialise waitbar to keep track of the progress
    f = waitbar(0,'1','Name','Computing the inverse kinematics...',...
    'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');

    setappdata(f,'canceling',0);

    x_c = sym(zeros(1,length(t)));
    y_c = sym(zeros(1,length(t)));
    z_c = sym(zeros(1,length(t)));
    o_c = [x_c; y_c; z_c];

    r   = sym(zeros(1,length(t)));
    s   = sym(zeros(1,length(t)));
    D   = sym(zeros(1,length(t)));
    B   = sym(zeros(1,length(t)));

    q1  = sym(zeros(1,length(t)));
    q2  = sym(zeros(1,length(t)));
    q3  = sym(zeros(1,length(t)));
    q4  = sym(zeros(1,length(t)));
    q5  = sym(zeros(1,length(t)));
    q6  = sym(zeros(1,length(t)));

    syms t1 t2 t3
    f_03(t1,t2,t3) = R_03;

    for j = 1:length(t)
        % Check for clicked Cancel button
        if getappdata(f,'canceling')
            break
        end
        
        % Update waitbar and message
        waitbar(j/length(t),f,...
            sprintf('Computing the inverse kinematics...%i%%',round(j/length(t)*100)))
        
        R = rotx(sym(thetaX(j)))*roty(sym(thetaY(j)))*rotz(sym(thetaZ(j)));
        
        % Determining the x,y,z-coordinates of the wrist center
        o_c(:,j) = o(:,j) - abs(d(6))*R*[0 0 1]';
        x_c(j) = sym(o_c(1,j));
        y_c(j) = sym(o_c(2,j));
        z_c(j) = sym(o_c(3,j));

        % Computing the parameters needed for the inverse position problem
        r(j)     =                                     sqrt(x_c(j)^2 + y_c(j)^2) - a(1);
        s(j)     =                                                        z_c(j) - d(1);
        D(j)     = (r(j)^2 + s(j)^2 - a(2)^2 - d(4)^2 - a(3)^2)/(2*a(2)*sqrt(d(4)^2+a(3)^2));
        B(j)     =                     sqrt((1-D(j)^2)*(d(4)^2 + a(3)^2)./(r(j)^2 + s(j)^2));

        % Inverse Position Solution
        q1(j)   =                                 atan2(-x_c(j),y_c(j));
        q2(j)   = pi/2 -(atan2(B(j),sqrt(1-B(j)^2)) + atan2(s(j),r(j)));
        q3(j)   =    atan2(sqrt(1-D(j)^2),D(j)) - atan2(abs(d(4)),a(3));

        if abs(q1(j)) < eps % if almost zero, make zero
            q1(j) = sym(0);
        end

        if abs(q2(j)) < eps % if almost zero, make zero
            q2(j) = sym(0);
        end

        if abs(q3(j)) < eps % if almost zero, make zero
            q3(j) = sym(0);
        end

        F_03 = f_03(q1(j),q2(j),q3(j));
        R_36 = F_03.'*R;

        % Inverse Orientation solution
        q5(j) = atan2(sqrt(1-R_36(3,3)^2),R_36(3,3));

        if double(abs(q5(j))) < eps % if almost zero, make zero
            q5(j) = sym(0);

        end

        if sin(q5(j)) == 0
            q4(j) = sym(0); % choose q4 arbitrarily to be zero
            q6(j) = atan2(R_36(2,1),R_36(1,1));
        else
            q4(j) = atan2(-R_36(2,3),-R_36(1,3));
            q6(j) = atan2(-R_36(3,2),R_36(3,1));
        end

        if double(abs(q4(j))) < eps % if almost zero, make zero
            q4(j) = sym(0);
        end

        if double(abs(q6(j))) < eps % if almost zero, make zero
            q6(j) = sym(0);
        end

    end
    
    delete(f)
    
end