%{
MEC 529
Adam Nebylitsa
HW 3
%}
function R = Rot(w, theta)
    % Tests to make sure the norm of the vector is 1. If not returns nan
    if(abs(norm(w)-1)>10^-5&&norm(w)~=0)
        fprintf("Input Vector does not have a length of 1.\n")
        R =nan;
        return
    end
    %Takes the skew symmetry matrix multplies it by the angle and then
    %raises it to the e power to create the rotation matrix.
    R = expm(Vector2SSMatrix(w)*theta);
end