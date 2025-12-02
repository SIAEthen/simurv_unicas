function J = J16(q_min,q_max,q_offset,q_i,q)
%
% Computes the Jacobian for the task function:
%   distance from mechanical joint limits version 2
%
% function  J = J16(q_min,q_max,q_offset,q_i,q)
%
% input:
%       q_min    dim nx1     lower mechnical limit
%       q_max    dim nx1     upper mechnical limit
%       q_offset dim nx1     target offset from limits
%       q_i      dim nx1     "influence" area around offset
%       q        dim nx1     joint positions
%
% output:
%       J        dim 1x6+n   Jacobian
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

q_min    = CheckVector(q_min);
q_max    = CheckVector(q_max);
q_offset = CheckVector(q_offset);
q_i      = CheckVector(q_i);
q        = CheckVector(q);

n = size(q,1);

J = [0 0 0 0 0 0];
for i=1:n
    if q(i)>=(q_max(i)-q_offset(i)-q_i)
        J(6+i) = -(1/n)*(q(i)-(q_max(i)-q_offset(i)));
    elseif q(i)<=(q_min(i)+q_offset(i)+q_i)
        J(6+i) = -(1/n)*(q(i)-(q_min(i)+q_offset(i)));
    else
        J(6+i) = 0;
    end
end
