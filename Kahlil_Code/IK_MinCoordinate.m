function Theta = IK_MinCoordinate(X_d, Theta_0, epsilon)
    max_iterations = 20;
    i = 0;
    Theta = Theta_0;
    e = X_d - FK_2R(Theta);
    while norm(e) > epsilon && i < max_iterations
        %disp(norm(e));
        Theta = Theta + pinv(J_2R(Theta)) * e;
        i = i + 1;
        e = X_d - FK_2R(Theta);
    end
    
end