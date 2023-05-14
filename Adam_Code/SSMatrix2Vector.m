%{
MEC 529
Adam Nebylitsa
HW 3
%}
function v = SSMatrix2Vector(m)
    [s1,s2] = size(m);
    % Makes sure the matrix is skew symmetric if not returns nan.
    if~(issymmetric(m,'skew'))
        fprintf("Matrix is not skew-symmetric.\n")
        v= nan;
        return
    end
    % Makes sure that the matrix is 3 by 3 if not returns nan.
    if~(s1==3&&s2==3)
        fprintf("Matrix is not 3x3.\n")
        v= nan;
        return
    end
    % Gets the vector from the skew symmetric matrix.
    v=[-m(2,3);m(1,3);-m(1,2)];
end