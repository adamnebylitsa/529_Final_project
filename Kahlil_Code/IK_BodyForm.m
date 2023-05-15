function Theta = IK_BodyForm(Tsd, Theta_0, epsilon, B, M)
    ew = epsilon(1);
    ev = epsilon(2);

    max_iterations = 20;
    i = 0;
    
    %need to do fk on theta_0 initial guess
    %Tbd = inv(M)*Tsd;

    Tsb0 = FK_BodyForm(B,M,Theta_0);
    Tbd = inv(Tsb0)*Tsd;
    
    [S, theta] = matrixLog(Tbd)
    vb = S*theta
    
    v = vb(4:6);
    w = vb(1:3);

    Theta = Theta_0;

    while norm(w) > ew && norm(v) > ev && i < max_iterations
        Theta = Theta + pinv(J_BodyForm(B, Theta))*vb;
        
        Tbd = FK_BodyForm(B,M,Theta)/Tsd;
        [S, theta] = matrixLog(Tbd);
        vb = S*theta;
    
        v = vb(4:6);
        w = vb(1:3);

        i = i+1;
    end

end