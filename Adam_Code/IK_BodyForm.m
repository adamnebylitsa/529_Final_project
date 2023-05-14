%{
MEC 529
Adam Nebylitsa
Final Project
%}
function theta = IK_BodyForm(T_sd, theta_0, e, B,M)
    error_w=100;
    error_v=100;
    f=FK_BodyForm(B,M,theta_0);
    R=f(1:3,1:3);
    invf=[transpose(R),-transpose(R)*f(1:3,4);
        0,0,0,1];
    [s,theta]=trans2Screw(invf*T_sd);
    vb=s*theta;

    while ((abs(error_v)>e(2)) || (abs(error_w)>e(1)))
        J=J_BodyForm(B,theta_0);
        theta_0=theta_0+pinv(J)*vb;
        theta_0=mod(theta_0,2*pi).*sign(theta_0);
        f=FK_BodyForm(B,M,theta_0);
        R=f(1:3,1:3);
        invf=[transpose(R),-transpose(R)*f(1:3,4);
        0,0,0,1];
        [s,theta]=trans2Screw(invf*T_sd);
        vb=s*theta;
        error_w=norm(vb(1:3));
        error_v=norm(vb(4:6));
    end
    theta=theta_0;
end