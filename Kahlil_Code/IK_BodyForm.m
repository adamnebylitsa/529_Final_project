function Theta = IK_BodyForm(Tsd, Theta_0, epsilon, B, M)
    ew = epsilon(1);
    ev = epsilon(2);

    max_iterations = 20;
    i = 0;
    
    %Tbd = inv(M)*Tsd;
    Tbd = M/Tsd;

    vb_skewed = matrixLog(Tbd);
    
    wb = [vb_skewed(3,2); vb_skewed(1,3); vb_skewed(2,1)];
    vb = vb_skewed(1:3,4);

    Theta = Theta_0;

    while norm(wb) > ew && norm(vb) > ev && i < max_iterations
        Theta = Theta + pinv(J_BodyForm(B, Theta))*[wb;vb];
        
        Tbd = FK_BodyForm(B,M,Theta)/Tsd;
        vb_skewed = matrixLog(Tbd);
    
        wb = [vb_skewed(3,2); vb_skewed(1,3); vb_skewed(2,1)];
        vb = vb_skewed(1:3,4);

        i = i+1;
    end

end