function [] = SimulateMovement(Q,o,t)
% This function imports the time dependent joint space variables as well as
% the time vector t and shows a visual simulation of the movement of the
% movement of the KUKA KR10 R1100-2 robotic arm. 
    
    addpath('KR_10_R1100-2\urdf')
    robot = importrobot('KR_10_R1100-2.urdf');
    
    config = homeConfiguration(robot);

    for i = 1:length(t)
        for j = 1:6
            config(j).JointPosition = double(Q(j,i));
        end
        
        fig = figure(5);
        if(i == 1) 
            clf(5)
        end
        set(gcf,'color','w');
        
        show(robot, config, 'PreservePlot', false, 'Frames', 'On');
        hold on
        plot3(o(1,i), o(2,i), o(3,i), 'r.', 'MarkerSize', 5)
        legend('Desired', 'Fontsize', 12, 'Position',[0.75 0.75 0.15 0.10])
        axis([-0.5 0.5 -0.5 1.0 -0.2 1.5])
        [caz,cel] = view([-5 3 3])
        drawnow;
        
        frame = getframe(fig);
        im = frame2im(frame);
        [img,map] = rgb2ind(im,256);
        
        if i == 1
            imwrite(img,map,'inversekinematics.gif','gif','LoopCount',Inf,'DelayTime',0.005);
        else
            imwrite(img,map,'inversekinematics.gif','gif','WriteMode','append','DelayTime',0.005);
        end
    end
    
    disp('Simulation:  Done')
    
end

