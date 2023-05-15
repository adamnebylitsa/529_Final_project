%{
MEC 529
Adam Nebylitsa
Kahlil Pollack-Hinds
Group 6
Final Project
%}
function J_s =J_SpaceForm(S,theta)
    %creates the proper size jacobian
    J_s=zeros(size(S));
    for i =1:length(theta)
        %for every vector it starts as the identity
        m = eye(6);
        %then for values from 1 to 1 under the current value
        for j = 1:i-1
            %it maps the vector based on the adj matrix. 
            m=m*AD(transform(S(:,j),theta(j)));
        end
        J_s(:,i)=m*S(:,i);
    end
end