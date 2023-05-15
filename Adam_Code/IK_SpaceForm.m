%{
MEC 529
Adam Nebylitsa
Final Project
%}
function theta = IK_SpaceForm(T_sd, theta_0, e, S,M)
    f=FK_SpaceForm(S,M,theta_0);
    invf=[f(1:3,1:3)',-f(1:3,1:3)'*f(1:3,4);
        0,0,0,1];
    [s,theta]=trans2Screw(invf*T_sd);
    vs=AD(f)*s*theta;
    error_w=norm(vs(1:3));
    error_v=norm(vs(4:6));
    while ((abs(error_v)>e(2)) || (abs(error_w)>e(1)))
        J=J_SpaceForm(S,theta_0);
        theta_0=theta_0+pinv(J)*vs;
        theta_0=mod(theta_0,2*pi).*sign(theta_0);
        f=FK_SpaceForm(S,M,theta_0);
        invf=[f(1:3,1:3)',-f(1:3,1:3)'*f(1:3,4);
        0,0,0,1];
        [s,theta]=trans2Screw(invf*T_sd);
        vs=AD(f)*s*theta;
        error_w=norm(vs(1:3));
        error_v=norm(vs(4:6));
    end
    theta=theta_0;
end