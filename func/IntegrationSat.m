function x_new = IntegrationSat(d_x, x, x_min, x_max, Ts)
%
% Saturated integral of the input
%
% function [x_new, check] = IntegrationSat(d_x, x, x_min, x_max, Ts)
%
% input:
%       d_x   dim mx1      time derivative
%       x     dim mx1      vector to be integrated
%       x_min dim mx1      lower bound
%       x_max dim mx1      upper bound
%       Ts    dim 1x1      sampling time
%
% output:
%       x_new dim mx1      new value for x
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

d_x   = CheckVector(d_x);
x     = CheckVector(x);
x_min = CheckVector(x_min);
x_max = CheckVector(x_max);

m = size(x,1);

x_new = x + Ts*d_x;

for i= 1:m
    if x_new(i) > x_max(i)
        x_new(i) = x_max(i);
    elseif x_new(i) < x_min(i)
        x_new(i) = x_min(i);
    end
end

