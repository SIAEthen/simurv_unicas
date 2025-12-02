function U = U_mat(R_b_a, r_ab_a)
%
% Matrix for the force/moment transformation from frame b to frame a
%
% function out = U_mat(R_b_a, r_ab_a)
%
% input:
%       R_b_a   dim 3x3     rotation matrix from frame b to frame a
%       r_ab_a  dim 3x1     vector from origin of frame a to origin of
%                           frame b expressed in frame a
%
% output:
%       U       dim 6x6     output matrix
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

r_ab_a = CheckVector(r_ab_a);

U = [R_b_a            zeros(3,3)
     S(r_ab_a)*R_b_a  R_b_a];