function Phi = Regressor_stupid(R)
%REGRESSOR_STUPID Summary of this function goes here
%   Detailed explanation goes here
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv
z = [0,0,1]';

Phi = [-1*R'* z    zeros(3,3);
       zeros(3,1)  S(R'*z)];
end

