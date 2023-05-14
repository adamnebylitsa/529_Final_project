%{
MEC 529
Adam Nebylitsa
HW 5
%}
function T_sb = FK_BodyForm(B,M, Theta)
    [B1, n1] = size(B);
    [M1,M2] = size(M);
    [n2, T2] = size(Theta);
    %Makes sure B has 6 rows
    if B1~=6 
        fprintf("B does not have 6 rows.\n")
        T_sb=nan;
        return
    end
    %Makes sure M is a 4x4 matirx
    if M1~=4 ||M2~= 4
        fprintf("M is not a 4x4 Matrix.\n")
        T_sb=nan;
        return
    end
    %makes sure the columns in B equal the numbers of angles 
    if ~((n1==n2&&T2==1)||(n1==T2&&n2==1)) 
        fprintf("The number of angles does not equal the number of joints in S.\n")
        T_sb=nan;
        return
    end
    %Starts the transformation matrix as the M matrix
    T_sb = M;
    %goes through all the joints and get the transformation matrix of it
    for i = 1:length(Theta)
        T_sb= T_sb*transform(B(:,i),Theta(i));
    end
end