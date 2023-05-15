clear all
clc
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
B2 = [0;-1;0;   (.316+.384);0;0.088];
B3 = [0;0;-1;   0;-0.088;0];
B4 = [0;1;0;   -(.384-0.088);0;0];
B5 = [0;0;-1;   0;-0.088;0];
B6 = [0;1;0;   .107;0;-0.088];
B7 = [0;0;1;   0;0;0];

S = [S1, S2, S3, S4, S5, S6, S7];
B = [B1, B2, B3, B4, B5, B6, B7];

%{
Joint Limits (°):
J1, J3, J5, J7: -2.89/2.89
J2: -1.76278/1.76278
J4: -3.07178/-0.069
J6: -0.0174533/3.75246
%}


theta_start = [1.4 -1.5 pi/4 -1 1.2 1.22 2].';
theta_end = [1.4; -1.5; pi/2; -2; 0; 2; 0];

theta_diff = theta_end - theta_start;

v_lim = deg2rad([150;150;150;150;180;180;180]);
a_lim = deg2rad([850;425;570;700;850;1140;1140]);

v_max = v_lim./theta_diff
a_max = a_lim./theta_diff

v = min(v_max)
a = min(a_max)

if a < v^2
    a = v^2-1
end

q = v^2/a


%get rotation matrix to make M
R = [[1;  0; 0], [0; -1; 0], [0; 0; -1]];
p =  [0.088; 0; .333+.316+.384-.107];

%End effector matrix M:
M = [R, p;
    0, 0, 0, 1];

tsd_start = FK_SpaceForm(S, M, theta_start);
tsd_end = FK_SpaceForm(S, M, theta_end);


for i=1:0.2:
    
end
