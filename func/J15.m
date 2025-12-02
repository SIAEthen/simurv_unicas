function J = J15(q_min,q_max,q)
%
% Computes the Jacobian for the task function:
%   distance from mechanical joint limits version 1
%
% function  J = J15(q_min,q_max,q)
%
% input:
%       q_min    dim nx1     lower mechnical limit
%       q_max    dim nx1     upper mechnical limit
%       q        dim nx1     joint positions
%
% output:
%       J        dim 1x6+n   Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

q_min = CheckVector(q_min);
q_max = CheckVector(q_max);
q     = CheckVector(q);

n = size(q,1);

q_mean = q_min + .5*(q_max - q_min);

J = [0 0 0 0 0 0];
for i=1:n
    J(6+i) = -(1/n)*(q(i)-q_mean(i))/(q_max(i)-q_min(i));
end
