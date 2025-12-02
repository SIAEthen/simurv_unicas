function Phi = Regressor_smart2(R)
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

Phi = [zeros(3,3)     R';
       S(R'*[0;0;1])  zeros(3,3)];