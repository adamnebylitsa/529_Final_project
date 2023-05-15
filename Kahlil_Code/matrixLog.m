function [S, theta] = matrixLog(T)
    

    R = T(1:3, 1:3);
    error = 0.000001;
    
    tr = trace(R);
    p = T(1:3, 4);


    if (abs(trace(T)-4)< 0.00001)
        sv = T(1:3,4);
        theta = norm(sv);
        sv = Sv/theta;
        s = [0;0;0;Sv];
        return
    end

  
    [sw, theta] = getTheta_S(T);
    sw_skewed = [0 -sw(3) sw(2); sw(3) 0 -sw(1); -sw(2) sw(1) 0];
    G= eye(3)*(1/theta) - .5*sw_skewed + (1/theta - (.5*cot(theta/2)))*sw_skewed^2;
    sv = G * p;
    S = [sw; sv];


    %{

        %obtaining skewed w matrix and reconstructing standard 3x3 matrix
        theta = acos(.5* (tr -1));
        w_skewed = (R - transpose(R))/(2*sin(theta));
        sw = w_skewed;
    
        G= eye(3)*(1/theta) - .5*sw + (1/theta - (.5*cot(theta/2)))*sw^2;
        sv = G * p;
    
        w = [w_skewed(3,2); w_skewed(1,3); w_skewed(2,1)];
        sw = w;
    
        S = [sw; sv];
    %}
    end  

