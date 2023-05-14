function Jb = J_BodyForm(B,theta)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
 matrix_product = eye(4);
    Jb = [];
    
    
    for i = size(B,2):-1:1
        matrix_product = eye(4);
        if i == size(B,2)
            Jb = B(:,size(B,2));
        else
            for j = size(B,2):-1:i+1
                matrix_product = matrix_product * trans_matrix(-(B(:,j)), theta(j));
            end
            p = matrix_product(1:3, 4);
            skewed_p = [0, -p(3), p(2);
                 p(3), 0, -p(1);
                 -p(2), p(1), 0];
            R = matrix_product(1:3,1:3);
            
            z = zeros(3,3);
            adjoint = [R, z;
                       (skewed_p*R), R];
            Jb = [adjoint*B(:,i), Jb];
        end

    end  
    
    Jb;
    return 

end



