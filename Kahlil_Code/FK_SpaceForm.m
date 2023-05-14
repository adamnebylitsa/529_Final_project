function Tsb = FK_SpaceForm(S, M, theta)
    
    %Initialize matrix product as an identity matrix
    matrix_product = eye(4);

    %trans_matrix function from hw4 utilized to get matrix representation
    %of screw axes
    for i = 1:size(S, 2)
        matrix_product = matrix_product * trans_matrix(S(:,i), theta(i));
    end

    Tsb = matrix_product * M;
end