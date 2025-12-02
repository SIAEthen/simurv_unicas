% dummy_encyclopedia
%
% carica modello phoenix+smart3s: DH + PARAM
%
%

eta = [1 0 1 0 .1 0]';

DH(:,4) = [0 -.7  -1 -.2 -.15 0]';

DrawSnap(eta,DH,PARAM);
axis off
view(158,33)

print('-depsc','fig_encyclopedia.eps')