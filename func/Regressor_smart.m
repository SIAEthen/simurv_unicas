function Phi = Regressor_smart(R)
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

Phi = [zeros(3,3)     R'          zeros(3,3);
       S(R'*[0;0;1])  zeros(3,3)  R'];