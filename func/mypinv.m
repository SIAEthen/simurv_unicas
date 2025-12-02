function out = mypinv(J,W,caller)
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

if nargin==2
    caller = 'x';
end


if any(any(J))
    l = 1e-5;
    [m,~]=size(J);
    if rank(J)==m
        out = W*J'/(J*W*J');
    else
        out = W*J'/(l*eye(m) + J*W*J');
        fprintf('\n WARNING in mypinv.m: task %s singular, damped inverse used',caller);  	
    end
else
    % when the input matrix is null do nothing but transpose it
    out = J';
end

%keyboard