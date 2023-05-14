clear all
clc

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
B2 = [0;-1;0;   (.316+.384);0;0.088];
B3 = [0;0;-1;   0;-0.088;0];
B4 = [0;1;0;   -(.384-0.088);0;0];
B5 = [0;0;-1;   0;-0.088;0];
B6 = [0;1;0;   .107;0;-0.088];
B7 = [0;0;1;   0;0;0];

S = [S1, S2, S3, S4, S5, S6, S7];
B = [B1, B2, B3, B4, B5, B6, B7];
%{
Joint limits in radians
j1_lim = [-2.85; 2.85];
j2_lim = [-1.75; 1.75];
j3_lim = [-2.85; 2.85];
j4_lim = [-3; -0.05];
j5_lim = [-2.85; 2.85];
j6_lim = [0; 3.75];
j7_lim = [-2.85; 2.85];
%}

arb_theta1 = [1;1;1.5;-1;1.2;1;1]

R = [[1;  0; 0], [0; -1; 0], [0; 0; -1]];
p =  [0.088; 0; .333+.316+.384-.107];

%End effector matrix M:
M = [R, p;
    0, 0, 0, 1];

tsb = FK_SpaceForm(S, M, arb_theta1)
guess = [.9;.9;1.4;-.9;1.1;.9;.9];
theta_body = IK_BodyForm(tsb, guess,[0.01;0.01],B, M)
theta_space = IK_SpaceForm(tsb, guess, [0.01;0.01], S, M)
