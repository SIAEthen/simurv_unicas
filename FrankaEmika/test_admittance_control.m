% Create input struct
S.x_r    = [0; 0; 0];
S.dx_r   = [0; 0; 0];
S.ddx_r  = [0; 0; 0];      % will be overwritten
S.xd   = [0; 0; 0];
S.dxd  = [0; 0; 0];
S.dt   = 0.01;
S.Fext = [5; 0; 0];      % 5 N force in X
for i=1:10000
% Run admittance controller
S = admittance3dof(S);

end
disp(S.x_r)
