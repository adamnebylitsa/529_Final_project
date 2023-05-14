clear all
clc
%{
Joint Limits (°):
J1, J3, J5, J7: -2.89/2.89
J2: -1.76278/1.76278
J4: -3.07178/-0.069
J6: -0.0174533/3.75246
%}

%Joint limits in radians
j1_lim = [-2.85; 2.85];
j2_lim = [-1.75; 1.75];
j3_lim = [-2.85; 2.85];
j4_lim = [-3; -0.05];
j5_lim = [-2.85; 2.85];
j6_lim = [0; 3.75];
j7_lim = [-2.85; 2.85];

S1 = [0;0;1;   0;0;0];
S2 = [0;1;0;   -0.333;0;0];
S3 = [0;0;1;   0;0;0];
S4 = [0;-1;0;   (.333+.316);0;-0.0825];
S5 = [0;0;1;   0;0;0];
S6 = [0;-1;0;  (.333+.316+.384);0;0];
S7 = [0;0;-1;   0;0.088;0];

S = [S1, S2, S3, S4, S5, S6, S7];

%get rotation matrix to make M
R = [[1;  0; 0], [0; -1; 0], [0; 0; -1]];
p =  [0.088; 0; .333+.316+.384-.107];

%End effector matrix M:
M = [R, p;
    0, 0, 0, 1];

x =[];
y =[];
z =[];
inc = 1.25;
for i=j1_lim(1):inc:j1_lim(2)
    for j= j2_lim(1):inc:j2_lim(2)
        for k= j3_lim(1):inc:j3_lim(2)
            for L= j4_lim(1):inc:j4_lim(2)
                for m= j5_lim(1):inc:j5_lim(2)
                    for n= j6_lim(1):inc:j6_lim(2)
                        for o= j7_lim(1):inc:j7_lim(2)
                            q = FK_SpaceForm(S, M, [i; j; k; L; m; n; o]);
                            x = [x, q(1,4)];
                            y = [y, q(2,4)];
                            z = [z, q(3,4)];
                        end
                    end
                end
            end
        end
    end
end



scatter3(x,y,z)
title("Robot Workspace")
axis auto
axis equal




