%{
MEC 529
Adam Nebylitsa
Kahlil Pollack-Hinds
Group 6
Final Project
%}
function J_b =J_BodyForm(B,theta)
    %creates the proper size jacobian
    J_b=zeros(size(B));
    for i =1:length(theta)
        %for every vector it starts as the identity
        m = eye(6);
        for j = length(theta):-1:i+1
            %then for values from the max to 1 more the current value
            %it maps the vector based on the adj matrix.
            m=m*AD(transform(-B(:,j),theta(j)));
        end
        J_b(:,i)=m*B(:,i);
    end
end