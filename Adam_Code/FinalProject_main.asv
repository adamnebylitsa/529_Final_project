%{
MEC 529
Adam Nebylitsa
Final Project
%}
clear
clc
%% Set UP
robot = importrobot('Franka_Emika_Panda.urdf');
robot.DataFormat = 'column';
%figure
Theta_home = [0 0 0 0 0 0 0].';
%show(robot,Theta_home,'Visuals','on','Frames','on');
figure
show(robot,Theta_home,'Visuals','off','Frames','on');
%% Part 1
S=[0  0    0    0    0 0      0
   0  1    0   -1    0 -1     0
   1  0    1    0    1 0     -1
   0 -.333 0   .649  0 1.033 0
   0  0    0    0    0 0    .088
   0  0    0  -.0825 0 0     0];  
B=[  0    0      0     0      0    0   0
     0   -1      0     1      0    1   0
    -1    0     -1     0     -1    0   1
     0  .593     0   -.277    0  .107  0
   -.088  0   -.088    0    -.088  0   0
    0   .088     0  -.0055    0 -.088  0];
%% Part 2
M=[1  0  0 .088
   0 -1  0  0
   0  0 -1 .926
   0  0  0  1];
theta1=[deg2rad(100),deg2rad(50),deg2rad(-100),deg2rad(-150),0,deg2rad(150),deg2rad(100)]';
Ts=FK_SpaceForm(S,M,theta1);
Tb=FK_BodyForm(B,M,theta1);
%figure
%show(robot,theta1,'Visuals','off','Frames','on');
homeM = [1 0 0 0
         0 1 0 0
         0 0 1 0
         0 0 0 1];
%figure
%view(3)
%triad('Matrix',homeM,'Scale',0.2,'LineWidth',1,'linestyle','-');
%hold on
%triad('Matrix',Ts,'Scale',0.2,'LineWidth',1,'linestyle','-');
%hold off
%% Part 3
lower_angle=[-166,-101,-166,-176,-166,-1,-166]';
upper_angle=[166,101,166,-4,166,215,166]';
num=5;
angles=[linspace(lower_angle(1),upper_angle(1),num);
        linspace(lower_angle(2),upper_angle(2),num);
        linspace(lower_angle(3),upper_angle(3),num);
        linspace(lower_angle(4),upper_angle(4),num);
        linspace(lower_angle(5),upper_angle(5),num);
        linspace(lower_angle(6),upper_angle(6),num);
        linspace(lower_angle(7),upper_angle(7),num)];
points=zeros(3,num^7);
curr =1;
%{
for i1 =1:num
    for i2 =1:num
        for i3 =1:num
            for i4 =1:num
                for i5 =1:num
                    for i6 =1:num
                        for i7 =1:num
                            temp =[angles(1,i1);angles(2,i2);angles(3,i3);angles(4,i4);angles(5,i5);angles(6,i6);angles(7,i7)];
                            T=FK_SpaceForm(S,M,temp);
                            points(:,curr)=T(1:3,4);
                            curr = curr +1;
                        end
                    end
                end
            end
        end
    end
end
figure
scatter3(points(1,:)',points(2,:)',points(3,:)')
%}
%% Part 4
%When joint 2 is 0, 1 and 3 are colinear
singular1 = [pi/2,0,pi/2,-pi/2,pi/2,pi/2,pi/2];
rank1 =rank(J_SpaceForm(S,singular1));
% When joint 2 4 and 6 are coplaner and parallel
singular2=[0,-atan(.0825/.316),0,-(atan(.0825/.316)+atan(.0825/.384)),0,pi/2,pi/2]';
rank2 = rank(J_SpaceForm(S,singular2));
%% Part 5

%% Part 6
theta_0=[pi/2;-pi/4;pi/2;-5*pi/4;0;3*pi/4;2*pi/3];
inv_thetab=IK_BodyForm(Tb,theta_0,[10^-4,10^-4],B,M);
inv_thetas=IK_SpaceForm(Ts,theta_0,[10^-4,10^-4],S,M);
%% Part 7
sym s1 s2 s3
theta_start=[0;0;0;0;0;0;0];
theta_end=[100;150;100;150;50;42;100];
delta_angle = theta_end-theta_start;
max_speed=[150;150;150;150;180;180;180];
max_acc=[850;425;570;700;850;1140;1140];
v_Trap=max_speed./(delta_angle);
a_Trap=max_acc./delta_angle;
T_trap =(delta_angle-(max_speed.^2./max_acc))./max_speed+2*(max_speed./max_acc);
T_trap_acc = max(T_trap);
T_3rd_v=1.5*delta_angle./max_speed;
T_3rd_a=(abs(6*delta_angle./max_acc)).^.5;
T_3rd=max(max(T_3rd_a),max(T_3rd_v));
[p,v,a,t] = trapveltraj([theta_start,theta_end],100,"PeakVelocity",max_speed,"EndTime",T_trap_acc);
%Ts_start = FK_SpaceForm(S,M,theta_start);
%Ts_end = FK_SpaceForm(S,M,theta_end);