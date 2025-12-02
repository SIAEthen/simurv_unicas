function e_out = AlignQuat(e_in,e_out)
%
% Eventually change sign to the quaternion for better data display
%
% function e_out = AlignQuat(e_in,e_out)
%
% input:
%       e_in    dim 4x1      quaternion
%       e_out   dim 4x1      quaternion
%
% output:
%       e_out   dim 4x1      quaternion
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

angle = atan2(norm(cross(e_in(1:3),e_out(1:3))),dot(e_in(1:3),e_out(1:3)));

if (angle>(pi/2))
    %e_out = - e_out;
end