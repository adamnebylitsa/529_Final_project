%{
    ------checking for redundancy--------
n = 7
r = 6

1 redundant degree of freedom

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
                            if rank(J_SpaceForm(S,[i; j; k; L; m; n; o])) < 6
                                disp('heres one');
                            else
                                disp(rank(J_SpaceForm(S, [i; j; k; L; m; n; o])));
                            end
                        end
                    end
                end
            end
        end
    end
end
