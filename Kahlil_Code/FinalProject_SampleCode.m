

%% ---> Using triad function
% triad('Matrix',T_sb,'Scale',0.2,'LineWidth',1,'linestyle','-'); % Adjust the options for better representation


%% ---> Importing the robot URDF file
% robot = importrobot('URDF_File_Name.urdf');
% robot.DataFormat = 'column';
% For more information refer to: https://www.mathworks.com/help/robotics/ref/importrobot.html


%% ---> Showing the robot at its home (zero) configuration:
% figure
% Theta_HomeConfiguration = [0 0 0 0 0 0 0].'; % n elements for an n-DOF robot
% show(robot,Theta_HomeConfiguration,'Visuals','on','Frames','on'); % 'on'/'off' options are used to show/hide the CAD STL files and frames
% For more information refer to: https://www.mathworks.com/help/robotics/ref/rigidbodytree.show.html

%% ---> Showing the robot at an arbitrary Theta configuration:
% figure
% Theta = [pi/8 pi/8 pi/8 pi/8 pi/8 pi/8 pi/8].'; % n elements for an n-DOF robot
% show(robot,Theta,'Visuals','on','Frames','on');


%% ---> Robot motion simulation
% figure
% for i=1:N % N: Number of samples
%     Theta_d = [theta_1(i) theta_2(i) theta_3(i) theta_4(i) theta_5(i) theta_6(i) theta_7(i)].';
%     show(robot,Theta_d,'PreservePlot',false,'Visuals','on','Frames','off');
%     drawnow;
% end

% To make the simulation faster for the purpose of checking the code and
% debugging, you may want to keep the number of samples N small and make
% 'Visuals', 'off' and 'Frames', 'on' in the for loop.