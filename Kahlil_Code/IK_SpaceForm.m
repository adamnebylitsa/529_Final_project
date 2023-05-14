function Theta = IK_SpaceForm(Tsd, Theta_0, epsilon, S, M)
    
    ew = epsilon(1);
    ev = epsilon(2);

    max_iterations = 20;
    i = 0;
    
    %finding adjoint--
    p = M(1:3, 4);
    skewed_p = [0, -p(3), p(2);
         p(3), 0, -p(1);
         -p(2), p(1), 0];

    R = M(1:3,1:3);
    
    z = zeros(3,3);
    adjoint = [R, z;
               (skewed_p*R), R];
    %----

    Tbd = M/Tsd;

    logTbd =  matrixLog(Tbd);

    vs_skewed = adjoint * logTbd;

    ws = [vs_skewed(3,2); vs_skewed(1,3); vs_skewed(2,1)];
    vs = vs_skewed(1:3,4);

    Theta = Theta_0;

    while norm(ws) > ew && norm(vs) > ev && i < max_iterations
        Theta = Theta + pinv(J_SpaceForm(S, Theta))*[ws;vs];

        Tbd = FK_SpaceForm(S,M,Theta)/Tsd;
        
        logTbd =  matrixLog(Tbd);

        vs_skewed = adjoint * logTbd;
    
        ws = [vs_skewed(3,2); vs_skewed(1,3); vs_skewed(2,1)];
        vs = vs_skewed(1:3,4);
        
        i = i+1;

    end
    
end