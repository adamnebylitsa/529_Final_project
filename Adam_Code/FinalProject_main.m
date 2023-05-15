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
%figure
%show(robot,Theta_home,'Visuals','off','Frames','on');
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
theta2=[1.4 -1.5 pi/4 -1 1.2 1.22 2]';
Ts=FK_SpaceForm(S,M,theta1);
Tb=FK_BodyForm(B,M,theta1);
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
lower_angle=deg2rad([-166,-101,-166,-176,-166,-1,-166])';
upper_angle=deg2rad([166,101,166,-4,166,215,166])';
num=4;
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
J_singular1=J_SpaceForm(S,singular1);
rank1 =rank(J_singular1);
% When joint 2 4 and 6 are coplaner and parallel
singular2=[0,-atan(.0825/.316),0,-(atan(.0825/.316)+atan(.0825/.384)),0,pi/2,pi/2]';
J_singular2=J_SpaceForm(S,singular2);
rank2 =rank(J_singular2);
%% Part 5
%{
near_singular=[0,-atan(.0825/.316)+.001,0,-(atan(.0825/.316)+atan(.0825/.384)),0,pi/2,pi/2];
FK_near = FK_SpaceForm(S,M,near_singular);
j_near_signular=[eye(3) zeros(3); Vector2SSMatrix(FK_near(1:3,4)),eye(3)]*J_SpaceForm(S,near_singular);
not_singular=[0,-pi/2,0,-(atan(.0825/.316)+atan(.0825/.384)),0,pi/2,pi/2];
FK_not_near = FK_SpaceForm(S,M,not_singular);
j_not_signular=[eye(3) zeros(3); Vector2SSMatrix(FK_not_near(1:3,4)),eye(3)]*J_SpaceForm(S,not_singular);
j_near_w=j_near_signular(1:3,:);
j_near_v=j_near_signular(4:6,:);
j_not_near_w=j_not_signular(1:3,:);
j_not_near_v=j_not_signular(4:6,:);
[V,D]=eig(j_near_w*j_near_w');
[X,Y,Z]=ellipsoid(FK_near(1,4),FK_near(2,4),FK_near(3,4),sqrt(D(1,1))/4,sqrt(D(2,2))/4,sqrt(D(3,3))/4);
[vec,ang]=Rot2Vec(V);
figure()
s=surf(X,Y,Z);
hold on
show(robot,near_singular','Visuals','off','Frames','on');
rotate(s,vec,ang)
axis equal
[V,D]=eig(j_near_v*j_near_v');
[X,Y,Z]=ellipsoid(FK_near(1,4),FK_near(2,4),FK_near(3,4),sqrt(D(1,1))/4,sqrt(D(2,2))/4,sqrt(D(3,3))/4);
[vec,ang]=Rot2Vec(V);
figure()
s=surf(X,Y,Z);
hold on
show(robot,near_singular','Visuals','off','Frames','on');
rotate(s,vec,ang)
axis equal
[V,D]=eig(j_not_near_w*j_not_near_w');
[X,Y,Z]=ellipsoid(FK_not_near(1,4),FK_not_near(2,4),FK_not_near(3,4),sqrt(D(1,1))/4,sqrt(D(2,2))/4,sqrt(D(3,3))/4);
[vec,ang]=Rot2Vec(V);
figure()
s=surf(X,Y,Z);
hold on
show(robot,not_singular','Visuals','off','Frames','on');
rotate(s,vec,ang)
axis equal
[V,D]=eig(j_not_near_v*j_not_near_v');
[X,Y,Z]=ellipsoid(FK_not_near(1,4),FK_not_near(2,4),FK_not_near(3,4),sqrt(D(1,1))/4,sqrt(D(2,2))/4,sqrt(D(3,3))/4);
[vec,ang]=Rot2Vec(V);
figure()
s=surf(X,Y,Z);
hold on
show(robot,not_singular','Visuals','off','Frames','on');
rotate(s,vec,ang)
axis equal
%}
%% Part 6
theta_0=[pi/2;-pi/4;pi/2;-5*pi/4;0;3*pi/4;2*pi/3];
inv_thetab=IK_BodyForm(Tb,theta_0,[10^-4,10^-4],B,M);
inv_thetas=IK_SpaceForm(Ts,theta_0,[10^-4,10^-4],S,M);
%% Part 7
%% a
syms s1
theta_start=deg2rad([-45;90;-35;-80;-20;30;-30]);
theta_end=deg2rad([12;50;23;-10;30;70;20]);
delta_angle = theta_end-theta_start;
trap = theta_start +s1*(theta_end-theta_start);
max_speed=deg2rad([150;150;150;150;180;180;180]);
max_acc=deg2rad([850;425;570;700;850;1140;1140]);
v_Trap=min(abs(max_speed./delta_angle));
a_Trap=min(abs(max_acc./delta_angle));
%T_trap =(delta_angle-(max_speed.^2./max_acc))./max_speed+2*(max_speed./max_acc);
%T_trap_acc = max(abs(T_trap));
T=(a_Trap+v_Trap^2)/(a_Trap*v_Trap);
s1=[];
t=linspace(0,T,40);
for i=t
    if i<v_Trap/a_Trap
        s1=[s1,a_Trap*i^2/2];
    elseif i> (T-v_Trap/a_Trap)
        s1=[s1,(2*a_Trap*v_Trap*T-(2*v_Trap^2)-(a_Trap^2*(i-T)^2))/(2*a_Trap)];
    else
        s1=[s1,v_Trap*i-v_Trap^2/(2*a_Trap)];
    end
end
trap = double(subs(trap));
figure
for i=1:40 % N: Number of samples
     Theta_d = [trap(:,i)];
     show(robot,Theta_d,'PreservePlot',false,'Visuals','off','Frames','on');
     drawnow;
end
figure()
tda=rad2deg((trap(:,2:end)-trap(:,1:end-1))/(t(2)-t(1)));
plot(t(1:end-1),tda)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
figure()
tdda=(tda(:,2:end)-tda(:,1:end-1))/(t(2)-t(1));
plot(t(1:end-2),tdda)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
%% B
Ts_start = FK_SpaceForm(S,M,theta_start);
Ts_end = FK_SpaceForm(S,M,theta_end);
T_3rd=1.5;
t =linspace(0,T_3rd,40);
s_3rd=((3*t.^2)/T_3rd^2-(2*t.^3)/T_3rd^3);
[start, angle]=trans2Screw(Ts_start\Ts_end);
ik = inverseKinematics('RigidBodyTree',robot);
for i=1:length(s_3rd)
    Ts(:,:,i) = Ts_start*transform(start,angle*s_3rd(i));
    invb(:,i)=ik("b_frame",Ts(:,:,i),[1,1,1,1,1,1],theta_start)
    %invb(:,i) = IK_BodyForm(Ts(:,:,i),theta_start+s_3rd(i)*(theta_end-theta_start),[.1,.01],B,M)
end

figure
for i=1:40 % N: Number of samples
     Theta_d = [invb(:,i)];
     show(robot,Theta_d,'PreservePlot',false,'Visuals','off','Frames','on');
     drawnow;
end
pb=Ts(1:3,4,:);
tdb=rad2deg((invb(:,2:end)-invb(:,1:end-1))/(t(2)-t(1)));
tddb=(tdb(:,2:end)-tdb(:,1:end-1))/(t(2)-t(1));
figure
plot(t(1:end-1),tdb)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
figure
plot(t(1:end-2),tddb)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
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
    invc(:,i)=ik("b_frame",Tsc(:,:,i),[1,1,1,1,1,1],theta_start)
    %invc(:,i)=IK_BodyForm(Tsc(:,:,i),theta_start+s_3rd(i)*(theta_end-theta_start),[.1,.01],B,M)
end
figure()
for i=1:40 % N: Number of samples
     Theta_d = [invc(:,i)];
     show(robot,Theta_d,'PreservePlot',false,'Visuals','off','Frames','on');
     drawnow;
end
tdc=rad2deg((invc(:,2:end)-invc(:,1:end-1))/(t(2)-t(1)));
tddc=(tdc(:,2:end)-tdc(:,1:end-1))/(t(2)-t(1));
figure()
plot(t(1:end-1),tdc)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
figure()
plot(t(1:end-2),tddc)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
%% D
T_5th=2;
t=linspace(0,T_5th,40);
s_5th=10*t.^3/T_5th^3-15*t.^4/T_5th^4+6*t.^5/T_5th^5;

%% E
points=[.04  .3   .7  .6; 
        -.4   0 .2  .3;
        .01  .4 .3   .7;];
time=[0 8 16 24];
path = minjerkpolytraj(points,time,40);
t=linspace(0,24,40);
Tse(:,:,1)=[eye(3) path(:,1); 0 0 0 1];
inve(:,1)=ik("b_frame",Tse(:,:,1),[1,1,1,1,1,1],theta_start)
for i=2:length(path)
    Tse(:,:,i)=[eye(3) path(:,i); 0 0 0 1];
    inve(:,i)=ik("b_frame",Tse(:,:,i),[1,1,1,1,1,1],inve(:,i-1))
end
figure
for i=1:40 % N: Number of samples
     Theta_d = [inve(:,i)];
     show(robot,Theta_d,'PreservePlot',false,'Visuals','off','Frames','on');
     drawnow;
end
tde=rad2deg((inve(:,2:end)-inve(:,1:end-1))/(t(2)-t(1)));
tdde=(tde(:,2:end)-tde(:,1:end-1))/(t(2)-t(1));
figure()
plot(t(1:end-1),tde)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
figure()
plot(t(1:end-2),tdde)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
%% F
start =[.2; -.4; .7];
mid = [.2;0;0];
end_p = [.2;.4;.7];
start_y=linspace(-.5,0,40);
end_y=linspace(0,.5,40);
start_z=linspace(.5,0,40);
end_z=linspace(0,.5,40);
R=[0 0 1
   0 1 0
   -1 0 0];
Tsf(:,:,1)=[R [.8;start_y(1);start_z(1)]; 0 0 0 1];
invf(:,1)=ik("b_frame",Tsf(:,:,1),[1,1,1,1,1,1],theta_start)
for i=2:length(start_y)
    Tsf(:,:,i)=[R [.8;start_y(i);start_z(i)]; 0 0 0 1];
    invf(:,i)=ik("b_frame",Tsf(:,:,i),[1,1,1,1,1,1],invf(:,i-1))
end
for i=1:length(end_y)
    Tsf(:,:,i+40)=[R [.8;end_y(i);end_z(i)]; 0 0 0 1];
    invf(:,i+40)=ik("b_frame",Tsf(:,:,i+40),[1,1,1,1,1,1],invf(:,i+39))
end
figure()
for i=1:80 % N: Number of samples
     Theta_d = [invf(:,i)];
     show(robot,Theta_d,'PreservePlot',false,'Visuals','off','Frames','on');
     drawnow;
end
tdf=rad2deg((invf(:,2:end)-invf(:,1:end-1)))/1;
tddf=(tdf(:,2:end)-tdf(:,1:end-1))/1;
figure()
plot(1:length(tdf),tdf)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")
figure()
plot(1:length(tddf),tddf)
legend("Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","Joint 6","Joint 7")