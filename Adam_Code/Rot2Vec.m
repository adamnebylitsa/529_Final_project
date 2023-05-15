%{
MEC 529
Adam Nebylitsa
Kahlil Pollack-Hinds
Group 6
Final Project
%}
function [w,theta] = Rot2Vec(R)
    %makes sure the matrix is 3 by 3, if not it returns nan.
    [s1,s2]=size(R);
    if~(s1==3&&s2==3)
        fprintf("Matrix is not 3x3.\n")
        w= nan;
        theta=nan;
        return
    end
    %If the trace of the matrix is 3, theta is 0 and w is undified and is
    %returned as nan
    if(abs(trace(R)-3)<10^-5)
        theta =0;
        w = nan;
        return
    end
    %If the trace is -1 the angle is pi, then it goes through the vector
    %possibilities and finds the first vector possibility that is valid.
    if(abs(trace(R)+1)<10^-5)
        theta =pi;
        if(R(1,1)>-1)
            w=[1+R(1,1);R(2,1);R(3,1)]/sqrt(2*(1+R(1,1)));
        elseif R(2,2)>-1
            w=[R(1,2);1+R(2,2);R(3,2)]/sqrt(2*(1+R(2,2)));
        elseif R(3,3)>-1
            w=[R(1,3);R(2,3);R(3,3)]/sqrt(2*(1+R(3,3)));
        end
    else
        %If it is any other angle it would calculate the angle from the
        %trace, then gets the vector from the matrix. 
        theta = acos((trace(R)-1)/2);
        w = SSMatrix2Vector((R-R')/(2*sin(theta)));
    end
end