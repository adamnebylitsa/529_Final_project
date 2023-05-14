robot = importrobot('Franka_Emika_Panda.urdf');
robot.DataFormat = 'column';
%For more information refer to: https://www.mathworks.com/help/robotics/ref/importrobot.html

figure
Theta_HomeConfiguration = [0 0 0 0 0 0 0].'; % n elements for an n-DOF robot
show(robot,Theta_HomeConfiguration,'Visuals','on','Frames','on'); % 'on'/'off' options are used to show/hide the CAD STL files and frames
