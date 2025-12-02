function DrawSnap(eta,DH,PARAM,depth)
%
% 3D rendering of the UVMS in a given scenario
%
% DrawSnap(eta,DH,PARAM,depth)
%
% input:
%   eta   dim 6x1    vehicle position/orientation
%   DH    dim nx4    DH table (contain joint positions)
%   PARAM struct     system parameters
%   depth dim 1x1    sea bottom depth
%
% output:
%
%   generate figure
%
% G. Antonelli, Simurv 4.0, 2013


% draw UVMS
DrawUVMS(eta,DH,PARAM);
axis tight
v = axis;
dimx = (v(2)-v(1));
dimy = 2*(v(3)-v(4));

% generate sea bottom
DrawTexture([dimx dimy],[v(1)-0*dimx/4 v(4)-dimy/4 depth],0.5,'texture3.jpg')

% generate sea surface (always at -1 m)
DrawTexture([dimx dimy],[v(1)-0*dimx/4 v(4)-dimy/4 -1],0.5,'texture4.jpg')

axis tight


