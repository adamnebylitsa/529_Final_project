function Tbs = FK_BodyForm(B, M, theta)

    %Initialize matrix product as M
    matrix_product = M;

    %trans_matrix function from hw4 utilized to get matrix representation
    %of screw axes
    for i = 1:size(B, 2)
        matrix_product = matrix_product * trans_matrix(B(:,i), theta(i));
    end
    
    Tbs = matrix_product;
end