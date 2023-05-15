%{
MEC 529
Adam Nebylitsa
Kahlil Pollack-Hinds
Group 6
Final Project
%}
function [S,theta] = trans2Screw(T)
    [s1,s2]= size(T);
    %Makes sure T is a 4x4 if not it ends and returns nan
    if s1~= 4 || s2~= 4
        fprintf("Transformation is not a 4x4 Matrix.\n")
        S=nan;
        theta = nan;
        return
    end
    %If the trace of the Matrix is 4 it is pure translation. 
    if (abs(trace(T)-4)<10^-5)
        Sv = T(1:3,4);
        theta = norm(Sv);
        Sv = Sv/theta;
        S = [0;0;0;Sv];
        return
    end
    %First get the angle and rotation vector
    [Sw, theta]= Rot2Vec(T(1:3,1:3));
    %Use the rtation vector to find the translation vector
    Sw_ = Vector2SSMatrix(Sw);
    G=1/theta *eye(3)-1/2*Sw_+(1/theta-1/2*cot(theta/2))*Sw_^2;
    Sv = G*T(1:3,4);
    S= [Sw;Sv];
end