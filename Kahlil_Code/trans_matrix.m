%Q1b of hw 4

function T = trans_matrix(S, theta)

%Unit vector sw are the first three values of 1x6 matrix S
    sw = S(1:3);

 %creating skewed symmetric matrix form
    skewed_sw = [0, -sw(3), sw(2);
                 sw(3), 0, -sw(1);
                 -sw(2), sw(1), 0];
    sv = S(4:6);
    S_matrix = [skewed_sw, sv; 
                0,0,0,0];
    
    %obtaining the transformation matrix using matrix exponent
    T = expm(theta * S_matrix);
end