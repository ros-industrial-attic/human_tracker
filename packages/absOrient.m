function [R,t]=absOrient(P,Pp)
%function [R,t]=absOrient(P,Pp)
%Given Two 3DOF Data:
%P:     3xn data
%Pp:    3xn data
%
%absOrient computes 
%R:     optimal rotation
%t:     optimal translation
%
%such that
%sum||Pi-[R(Ppi)+t]||^2
%
%is  minimized.
%
%Mili Shah

%Centroid of transformations t and that

[m,n]=size(P); 
p = 1/n*sum(P')';
pp = 1/n*sum(Pp')';
for i = 1:n
    P(1:3,i) = P(1:3,i)-p;
    Pp(1:3,i) = Pp(1:3,i)-pp;
end

%Calculates the best rotation
[u,s,v] = svd(Pp*P');
R = v*u';
if det(R)<0
    disp(sprintf('warning: reflection for R'))
    R = v*diag([1,1,-1])*u';
end

%Calculates the best transformation
t = p-R*pp;
