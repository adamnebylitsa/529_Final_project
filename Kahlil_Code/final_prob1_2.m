%home configuartions

%fixed frame
S1 = [0;0;1;   0;0;0];
S2 = [0;1;0;   -0.333;0;0];
S3 = [0;0;1;   0;0;0];
S4 = [0;-1;0;   (.333+.316);0;-0.0825];
S5 = [0;0;1;   0;0;0];
S6 = [0;-1;0;  (.333+.316+.384);0;0];
S7 = [0;0;-1;   0;0.088;0];

%body frame
B1 = [0;0;-1;   0;-0.088;0];
B2 = [0;-1;0;   (.316+.384-.107);0;0.088];
B3 = [0;0;-1;   0;-0.088;0];
B4 = [0;1;0;   -(.384-0.107);0;-0.055];
B5 = [0;0;-1;   0;-0.088;0];
B6 = [0;1;0;   .107;0;-0.088];
B7 = [0;0;1;   0;0;0];

S = [S1, S2, S3, S4, S5, S6, S7];
B = [B1, B2, B3, B4, B5, B6, B7];

Theta_Arb = [1.4 -1.5 pi/4 -1 1.2 1.22 2].'
%get rotation matrix to make M
R = [[1;  0; 0], [0; -1; 0], [0; 0; -1]];
p =  [0.088; 0; .333+.316+.384-.107];

%End effector matrix M:
M = [R, p;
    0, 0, 0, 1];

tsb = FK_SpaceForm(S, M, Theta_Arb)

robot = importrobot('Franka_Emika_Panda.urdf');
robot.DataFormat = 'column';
show(robot,Theta_Arb);


%---> Using triad function
triad('Matrix',tsb,'Scale',0.2,'LineWidth',1,'linestyle','-.')




