%{
MEC 529
Adam Nebylitsa
Kahlil Pollack-Hinds
Group 6
Final Project
%}
function  a = AD(T)
    %Maps Transformation matrix to adjoint matrix
    R=T(1:3,1:3);
    p =T(1:3,4);
    a =[R zeros(3); Vector2SSMatrix(p)*R R];
end