function Js = J_SpaceForm(S, theta)
    
    
    matrix_product = eye(4);
    Js = [];
    
    %The following for loop determines each column of the Space Jacobian
    %matrix. The first column is always the same as the first column of the
    %S matrix (first if statement)
    for i = 1:size(S, 2)
     
        matrix_product = eye(4);
        if i == 1
            Js = S(:,1);
        else
            for j = 1: i-1
                matrix_product = matrix_product * trans_matrix(S(:,j), theta(j));
            end

            %getting  from final transformation matrix
            p = matrix_product(1:3, 4);
            skewed_p = [0, -p(3), p(2);
                 p(3), 0, -p(1);
                 -p(2), p(1), 0];
            R = matrix_product(1:3,1:3);
            z = zeros(3,3);

            %Finding Adjoint matrix
            adjoint = [R, z;
                       (skewed_p*R), R];
            
            %appending adjoint * S(i) to Js
            Js = [Js, adjoint*S(:,i)];
        end

    end  

    Js;
     

end