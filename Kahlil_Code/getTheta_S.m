function [sw, theta] = getTheta_S(T)
    

R = T(1:3, 1:3);
error = 0.000001

tr = trace(R);
p = T(1:3, 4);


if (abs(trace(T))-4) <0.00001
    sv = T(1:3, 4);
    theta = norm(sv);
    sv = sv/theta;
    S = [0;0;0;sv];
    return
end

if abs(tr-3) < error
    sw = [0;0;0];
    sv = p/norm(p);
    theta = 0;
    S = [sw;sv];

elseif abs(tr - -1) < 0.0001  %accounting for matlab error
        theta = pi();
        %testing to ensure that solutions are valid (not zero or imaginary)
        if R(1,1) > -1
            w = (1/sqrt(2*(1+R(1,1)))) * [1 + R(1,1); R(2,1); R(3,1)]
 
            %w_skewed = [0 -w_skewed(3) w_skewed(2); w(3) 0 -w_skewed(1); -w_skewed(2) w_skewed(1) 0]

        elseif R(2,2) > -1
            w = (1/sqrt(2*(1+R(2,2)))) * [R(1,2); 1 + R(2,2); R(3,2)]
            %w_skewed = [0 -w_skewed(3) w_skewed(2); w(3) 0 -w_skewed(1); -w_skewed(2) w_skewed(1) 0]

        elseif R(3,3) > -1
            w = (1/sqrt(2*(1+R(3,3)))) * [R(1,3); 1 + R(2,3); 1+ R(3,3)]
            %w_skewed = [0 -w_skewed(3) w_skewed(2); w(3) 0 -w_skewed(1); -w_skewed(2) w_skewed(1) 0]
        end
        


else

    %obtaining skewed w matrix and reconstructing standard 3x3 matrix
    theta = acos(.5* (tr -1));
    w_skewed = (R - transpose(R))/(2*sin(theta));
    sw = w_skewed;

    G= eye(3)*(1/theta) - .5*sw + (1/theta - (.5*cot(theta/2)))*sw^2;
    sv = G * p;

    w = [w_skewed(3,2); w_skewed(1,3); w_skewed(2,1)];
    sw = w;

    %S = [sw; sv];

end  

end