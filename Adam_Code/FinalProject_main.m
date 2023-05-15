%{
MEC 529
Adam Nebylitsa
Kahlil Pollack-Hinds
Group 6
Final Project
%}
clear
clc
%% Figures are commented out due to the larger number of figures required, figures can be uncommented to viewed or viewed on the report
%% Set UP
robot = importrobot('Franka_Emika_Panda.urdf');
robot.DataFormat = 'column';
%figure
Theta_home = [0 0 0 0 0 0 0].';
%show(robot,Theta_home,'Visuals','on','Frames','on');
%figure
%show(robot,Theta_home,'Visuals','off','Frames','on');
%axis equal
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
%intialize random angles
theta1=[deg2rad(100),deg2rad(50),deg2rad(-100),deg2rad(-150),0,deg2rad(150),deg2rad(100)]';
theta2=[1.4 -1.5 pi/4 -1 1.2 1.22 2]';
%forward kinematics of those angles
Ts1=FK_SpaceForm(S,M,theta1);
Ts2=FK_SpaceForm(S,M,theta2);
Tb1=FK_BodyForm(B,M,theta1);
Tb2=FK_BodyForm(B,M,theta2);
%figure
%show(robot,theta2,'Visuals','off','Frames','on');
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
%Creates system for upper and lower bounds of angles
lower_angle=deg2rad([-166,-101,-166,-176,-166,-1,-166])';
upper_angle=deg2rad([166,101,166,-4,166,215,166])';
num=4;
% Divides each angle into 4 equally seperated parts
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
%Loops through every possibility for all angles to find the full workspace
of the robot
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
%plots the workspace
figure
scatter3(points(1,:)',points(2,:)',points(3,:)')
%}
%% Part 4
%Define singular points and prove it by showing the jacobian is not full
%rank
%When joint 2 is 0, 1 and 3 are colinear
singular1 = [pi/2,0,pi/2,-pi/2,pi/2,pi/2,pi/2];
J_singular1=J_SpaceForm(S,singular1);
rank1 =rank(J_singular1);
% When joint 2 4 and 6 are coplaner and parallel
singular2=[0,-atan(.0825/.316),0,-(atan(.0825/.316)+atan(.0825/.384)),0,pi/2,pi/2]';
J_singular2=J_SpaceForm(S,singular2);
rank2 =rank(J_singular2);
%% Part 5
%Defines a point near the singularity and finds the geometric jacobain of
%it
near_singular=[0,-atan(.0825/.316)+.001,0,-(atan(.0825/.316)+atan(.0825/.384)),0,pi/2,pi/2];
FK_near = FK_SpaceForm(S,M,near_singular);
j_near_signular=[eye(3) zeros(3); Vector2SSMatrix(FK_near(1:3,4)),eye(3)]*J_SpaceForm(S,near_singular);
%Defines a point away form the singularity and finds the geometric jacobain of
%it
not_singular=[0,-pi/2,0,-(atan(.0825/.316)+atan(.0825/.384)),0,pi/2,pi/2];
FK_not_near = FK_SpaceForm(S,M,not_singular);
j_not_signular=[eye(3) zeros(3); Vector2SSMatrix(FK_not_near(1:3,4)),eye(3)]*J_SpaceForm(S,not_singular);
%seperate the jacobains to the angular and linear parts
j_near_w=j_near_signular(1:3,:);
j_near_v=j_near_signular(4:6,:);
j_not_near_w=j_not_signular(1:3,:);
j_not_near_v=j_not_signular(4:6,:);
%find the eigenvalues and eigenvectors of each part of each jacobian and
%plots the ellipse for it centered at the end effector location
[V,D]=eig(j_near_w*j_near_w');
%sqrt(D)
[X,Y,Z]=ellipsoid(FK_near(1,4),FK_near(2,4),FK_near(3,4),sqrt(D(1,1))/4,sqrt(D(2,2))/4,sqrt(D(3,3))/4);
[vec,ang]=Rot2Vec(V);
%figure()
%s=surf(X,Y,Z);
%hold on
%show(robot,near_singular','Visuals','off','Frames','on');
%rotate(s,vec,ang)
%axis equal
[V,D]=eig(j_near_v*j_near_v');
[X,Y,Z]=ellipsoid(FK_near(1,4),FK_near(2,4),FK_near(3,4),sqrt(D(1,1))/4,sqrt(D(2,2))/4,sqrt(D(3,3))/4);
%sqrt(D)
[vec,ang]=Rot2Vec(V);
%figure()
%s=surf(X,Y,Z);
%hold on
%show(robot,near_singular','Visuals','off','Frames','on');
%rotate(s,vec,ang)
%axis equal
[V,D]=eig(j_not_near_w*j_not_near_w');
%sqrt(D)
[X,Y,Z]=ellipsoid(FK_not_near(1,4),FK_not_near(2,4),FK_not_near(3,4),sqrt(D(1,1))/4,sqrt(D(2,2))/4,sqrt(D(3,3))/4);
[vec,ang]=Rot2Vec(V);
%figure()
%s=surf(X,Y,Z);
%hold on
%show(robot,not_singular','Visuals','off','Frames','on');
%rotate(s,vec,ang)
%axis equal
[V,D]=eig(j_not_near_v*j_not_near_v');
%sqrt(D)
[X,Y,Z]=ellipsoid(FK_not_near(1,4),FK_not_near(2,4),FK_not_near(3,4),sqrt(D(1,1))/4,sqrt(D(2,2))/4,sqrt(D(3,3))/4);
[vec,ang]=Rot2Vec(V);
%figure()
%s=surf(X,Y,Z);
%hold on
%show(robot,not_singular','Visuals','off','Frames','on');
%rotate(s,vec,ang)
%axis equal
%% Part 6
%intial guess for the angle to preform inverse kinematics on it
theta_0=[pi/2;-pi/4;pi/2;-5*pi/4;0;3*pi/4;2*pi/3];
inv_thetab=IK_BodyForm(Tb1,theta_0,[10^-4,10^-4],B,M);
inv_thetas=IK_SpaceForm(Ts1,theta_0,[10^-4,10^-4],S,M);
%% Part 7
%% a
syms s1
%define start and end angles and the max values the joints could handle in
%speed and acceleration
theta_start=deg2rad([-45;90;-35;-80;-20;30;-30]);
theta_end=deg2rad([12;50;23;-10;30;70;20]);
delta_angle = theta_end-theta_start;
trap = theta_start +s1*(theta_end-theta_start);
max_speed=deg2rad([150;150;150;150;180;180;180]);
max_acc=deg2rad([850;425;570;700;850;1140;1140]);
%the values for v, a, and T for trapizod based on the joint limits
v_Trap=min(abs(max_speed./delta_angle));
a_Trap=min(abs(max_acc./delta_angle));
T=(a_Trap+v_Trap^2)/(a_Trap*v_Trap);
s1=[];
t=linspace(0,T,40);
%finding the distribution of s
for i=t
    if i<v_Trap/a_Trap
        s1=[s1,a_Trap*i^2/2];
    elseif i> (T-v_Trap/a_Trap)
        s1=[s1,(2*a_Trap*v_Trap*T-(2*v_Trap^2)-(a_Trap^2*(i-T)^2))/(2*a_Trap)];
    else
        s1=[s1,v_Trap*i-v_Trap^2/(2*a_Trap)];
    end
end
%subing s into the angles
trap = double(subs(trap));
for i=1:length(trap)
    T = FK_SpaceForm(S,M,trap(:,i));
    p(:,i)=T(1:3,4);
end
%Graphing everything for part a
%{
figure
for i=1:40 % N: Number of samples
     Theta_d = [trap(:,i)];
     show(robot,Theta_d,'PreservePlot',false,'Visuals','off','Frames','on');
     drawnow;
end

figure()
plot(t,p)
legend("X","Y","Z")
xlabel("Time(s)")
ylabel("Distance")
figure()
plot(t,rad2deg(trap))
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
xlabel("Time(s)")
ylabel("Angle")
figure()
tda=rad2deg((trap(:,2:end)-trap(:,1:end-1))/(t(2)-t(1)));
plot(t(1:end-1),tda)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
xlabel("Time(s)")
ylabel("Angular Speed")
figure()
tdda=(tda(:,2:end)-tda(:,1:end-1))/(t(2)-t(1));
plot(t(1:end-2),tdda)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
ylabel("Angular Acceleration")
xlabel("Time(s)")
%}
%% B
%
Ts_start = FK_SpaceForm(S,M,theta_start);
Ts_end = FK_SpaceForm(S,M,theta_end);
%total time needed
T_3rd=1.5;
t =linspace(0,T_3rd,40);
%distribution of s
s_3rd=((3*t.^2)/T_3rd^2-(2*t.^3)/T_3rd^3);
[start, angle]=trans2Screw(Ts_start\Ts_end);
ik = inverseKinematics('RigidBodyTree',robot);
%finding the transformation matrix at everypoint
%using build in IK since it is more efficent and finds the solution faster
for i=1:length(s_3rd)
    Ts(:,:,i) = Ts_start*transform(start,angle*s_3rd(i));
    invb(:,i)=ik("b_frame",Ts(:,:,i),[1,1,1,1,1,1],theta_start);
end
%Graphs everything for part B
%{
figure
view(3)
triad('Matrix',Ts_start,'Scale',0.2,'LineWidth',1,'linestyle','-');
hold on
triad('Matrix',Ts_end,'Scale',0.2,'LineWidth',1,'linestyle','-');
hold off
figure
for i=1:40 % N: Number of samples
     Theta_d = [invb(:,i)];
     show(robot,Theta_d,'PreservePlot',false,'Visuals','off','Frames','on');
     drawnow;
     pb(:,i)=Ts(1:3,4,i);
end
figure()
plot(t,pb)
legend("X","Y","Z")
xlabel("Time(s)")
ylabel("Distance")
figure()
plot(t,rad2deg(invb));
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
xlabel("Time(s)")
ylabel("Angle")
tdb=rad2deg((invb(:,2:end)-invb(:,1:end-1))/(t(2)-t(1)));
tddb=(tdb(:,2:end)-tdb(:,1:end-1))/(t(2)-t(1));
figure
plot(t(1:end-1),tdb)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
xlabel("Time(s)")
ylabel("Angular Speed")
figure
plot(t(1:end-2),tddb)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
ylabel("Angular Acceleration")
xlabel("Time(s)")
%}
%% C
T_3rd =4;
t =linspace(0,T_3rd,40);
s_3rd=((3*t.^2)/T_3rd^2-(2*t.^3)/T_3rd^3);
[rotation,angle]=Rot2Vec(Ts_start(1:3,1:3)'*Ts_end(1:3,1:3));
for i=1:length(s_3rd)
    r3(:,:,i)=Ts_start(1:3,1:3)*Rot(rotation,angle*s_3rd(i));
    p3(:,:,i)=Ts_start(1:3,4)+s_3rd(i)*(Ts_end(1:3,4)-Ts_start(1:3,4));
end

Tsc=[r3 p3; zeros(1,3,40) ones(1,1,40)];
for i=1:length(s_3rd)
    invc(:,i)=ik("b_frame",Tsc(:,:,i),[1,1,1,1,1,1],theta_start);
    pc(:,i)=Tsc(1:3,4,i);
end
%{
figure()
for i=1:40 % N: Number of samples
     Theta_d = [invc(:,i)];
     show(robot,Theta_d,'PreservePlot',false,'Visuals','off','Frames','on');
     drawnow;
end
figure()
plot(t,pc)
legend("X","Y","Z")
xlabel("Time(s)")
ylabel("Distance")
figure()
plot(t,rad2deg(invc))
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
xlabel("Time(s)")
ylabel("Angle")
tdc=rad2deg((invc(:,2:end)-invc(:,1:end-1))/(t(2)-t(1)));
tddc=(tdc(:,2:end)-tdc(:,1:end-1))/(t(2)-t(1));
figure()
plot(t(1:end-1),tdc)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
xlabel("Time(s)")
ylabel("Angular Speed")
figure()
plot(t(1:end-2),tddc)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
ylabel("Angular Acceleration")
xlabel("Time(s)")
%}
%% D
T_5th=4;
t=linspace(0,T_5th,40);
s_5th=10*t.^3/T_5th^3-15*t.^4/T_5th^4+6*t.^5/T_5th^5;
%points on the circle and where they start
c=[.4;.4;.2];
p_start=[.4;.2;.2];
alpha=2*pi;
rho = norm(p_start-c);
%path the system takes
p_s(:,1)=c+[rho*cos(alpha*s_5th(1)); rho*sin(alpha*s_5th(1));0];
%Starting to use the angles before as the inital guess for the next angle
%to try to ensure smooth motions
invd(:,1)=ik("b_frame",[[0 0 1; 0 1 0; -1 0 0] p_s(:,1); 0 0 0 1],[1,1,1,1,1,1],theta_start);
for i=2:length(s_5th)
    p_s(:,i)=c+[rho*cos(alpha*s_5th(i)); rho*sin(alpha*s_5th(i));0];
    invd(:,i)=ik("b_frame",[[0 0 1; 0 1 0; -1 0 0] p_s(:,i); 0 0 0 1],[1,1,1,1,1,1],invd(:,i-1));
end
%{
figure()
for i=1:40 % N: Number of samples
     Theta_d = [invd(:,i)];
     show(robot,Theta_d,'PreservePlot',false,'Visuals','off','Frames','on');
     drawnow;
end
tdd=rad2deg((invd(:,2:end)-invd(:,1:end-1))/(t(2)-t(1)));
tddd=(tdd(:,2:end)-tdd(:,1:end-1))/(t(2)-t(1));
figure()
plot(t,p_s)
legend("X","Y","Z")
xlabel("Time(s)")
ylabel("Distance")
figure()
plot(t,rad2deg(invd))
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
xlabel("Time(s)")
ylabel("Angle")
figure()
plot(t(1:end-1),tdd)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
xlabel("Time(s)")
ylabel("Angular Speed")
figure()
plot(t(1:end-2),tddd)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
ylabel("Angular Acceleration")
xlabel("Time(s)")
%}
%% E
%Points to travel through
points=[.04  .3   .7  .6; 
        -.4   0 .2  .3;
        .01  .4 .3   .7;];
%time each point is reached at
time=[0 8 16 24];
%path for the spline with the least jerk
path = minjerkpolytraj(points,time,40);
t=linspace(0,24,40);
Tse(:,:,1)=[eye(3) path(:,1); 0 0 0 1];
inve(:,1)=ik("b_frame",Tse(:,:,1),[1,1,1,1,1,1],theta_start);
for i=2:length(path)
    Tse(:,:,i)=[eye(3) path(:,i); 0 0 0 1];
    inve(:,i)=ik("b_frame",Tse(:,:,i),[1,1,1,1,1,1],inve(:,i-1));
end
%{
figure
view(3)
triad('Matrix',[eye(3) points(:,1); 0 0 0 1],'Scale',0.2,'LineWidth',1,'linestyle','-');
hold on
triad('Matrix',[eye(3) points(:,2); 0 0 0 1],'Scale',0.2,'LineWidth',1,'linestyle','-');
triad('Matrix',[eye(3) points(:,3); 0 0 0 1],'Scale',0.2,'LineWidth',1,'linestyle','-');
triad('Matrix',[eye(3) points(:,4); 0 0 0 1],'Scale',0.2,'LineWidth',1,'linestyle','-');
hold off
figure
for i=1:40 % N: Number of samples
     Theta_d = [inve(:,i)];
     show(robot,Theta_d,'PreservePlot',false,'Visuals','off','Frames','on');
     drawnow;
end
figure()
plot(t,path)
legend("X","Y","Z")
xlabel("Time(s)")
ylabel("Distance")
figure()
plot(t,rad2deg(inve))
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
xlabel("Time(s)")
ylabel("Angle")
tde=rad2deg((inve(:,2:end)-inve(:,1:end-1))/(t(2)-t(1)));
tdde=(tde(:,2:end)-tde(:,1:end-1))/(t(2)-t(1));
figure()
plot(t(1:end-1),tde)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
xlabel("Time(s)")
ylabel("Angular Speed")
figure()
plot(t(1:end-2),tdde)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
ylabel("Angular Acceleration")
xlabel("Time(s)")
%}
%% F
%Making the end effector draw the letter L
%Motion divided into 2 parts based on the 2 segments
end_y=linspace(-.55,0,40);
start_z=linspace(.55,0,40);
T_3rd=4/2;
t=linspace(0,T_3rd,40);
s=((3*t.^2)/T_3rd^2-(2*t.^3)/T_3rd^3);
R=[0 0 1
   0 1 0
   -1 0 0];
end_y=-.55+s*(0+.55);
start_z=.55+s*(0-.55);
Tsf(:,:,1)=[R [.55;-.55;start_z(1)]; 0 0 0 1];
invf(:,1)=ik("b_frame",Tsf(:,:,1),[1,1,1,1,1,1],theta_start);
for i=2:length(start_z)
    Tsf(:,:,i)=[R [.55;-.55;start_z(i)]; 0 0 0 1];
    invf(:,i)=ik("b_frame",Tsf(:,:,i),[1,1,1,1,1,1],invf(:,i-1));
end
for i=1:length(end_y)
    Tsf(:,:,i+40)=[R [.55;end_y(i);0]; 0 0 0 1];
    invf(:,i+40)=ik("b_frame",Tsf(:,:,i+40),[1,1,1,1,1,1],invf(:,i+39));
end
%{
t=linspace(0,T_3rd*2,80);
figure
view(3)
triad('Matrix',[R [.55;-.55;.55]; 0 0 0 1],'Scale',0.2,'LineWidth',1,'linestyle','-');
hold on
triad('Matrix',[R [.55;-.55;0]; 0 0 0 1],'Scale',0.2,'LineWidth',1,'linestyle','-');
triad('Matrix',[R [.55;0;0]; 0 0 0 1],'Scale',0.2,'LineWidth',1,'linestyle','-');
axis equal
hold off
figure()
for i=1:80 % N: Number of samples
     Theta_d = [invf(:,i)];
     show(robot,Theta_d,'PreservePlot',false,'Visuals','off','Frames','on');
     drawnow;
end
pe=[[ones(1,length(start_z))*.55;ones(1,length(start_z))*-.55;start_z],[ones(1,length(end_y))*.55;end_y;zeros(1,length(end_y))]];
figure()
plot(t,pe)
legend("X","Y","Z")
xlabel("Time(s)")
ylabel("Distance")
figure()
plot(t,rad2deg(invf))
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
xlabel("Time(s)")
ylabel("Angle")
tdf=rad2deg((invf(:,2:end)-invf(:,1:end-1)))/(t(2)-t(1));
tddf=(tdf(:,2:end)-tdf(:,1:end-1))/(t(2)-t(1));
figure()
plot(t(1:end-1),tdf)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
xlabel("Time(s)")
ylabel("Angular Speed")
figure()
plot(t(1:end-2),tddf)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
ylabel("Angular Acceleration")
xlabel("Time(s)")
%}