%{
MEC 529
Adam Nebylitsa
HW 3
%}
function m = Vector2SSMatrix(v)
    [s1,s2] = size(v);
    %Tests to make sure that the vector has only 3 elements either a 1 by 3
    %or 3 by 1. If the test fails it returns nana
    if ~((s1==3 && s2 ==1) || (s1==1 &&s2==3))
        fprintf("Input not a valid 3D Vector.\n")
        m=nan;
        return 
    end
    %Uses the vector to create the skew symmetry matrix.
    m = [0,-v(3),v(2);
        v(3),0,-v(1);
        -v(2),v(1),0];
end