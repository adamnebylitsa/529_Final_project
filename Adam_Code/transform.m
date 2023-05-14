%{
MEC 529
Adam Nebylitsa
HW 4
%}
function [T] = transform(S,theta)
    [s1,s2] = size(S);
    %Makes sure S is a 6x1 if not it ends and returns nan
    if s1~= 6 || s2~= 1
        fprintf("S is not 6x1 Vector.\n")
        T=nan;
        return
    end
    %Use the S vector and the angle to properly get the transformation
    %matrix
    Sw = S(1:3);
    Sv = S(4:6);
    G=eye(3)*theta+(1-cos(theta))*Vector2SSMatrix(Sw)+(theta-sin(theta))*Vector2SSMatrix(Sw)^2;
    T=[Rot(Sw,theta),G*Sv; 0 0 0 1];
end