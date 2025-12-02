function DrawUVMS(eta,DH,PARAM)
%
% 3D rendering of the UVMS in a given configuration
%
% DrawUVMS(eta,DH,PARAM)
%
% input:
%   eta   dim 6x1    vehicle position/orientation
%   DH    dim nx4    DH table (contain joint positions)
%   PARAM struct     system parameters
%
% output:
%
%   generate figure
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

eta=CheckVector(eta);

a1 = PARAM.a1;
a2 = PARAM.a2;
a3 = PARAM.a3;

% homogeneous matrix for the inertial frame
T_i = [eye(3,3), [0 0 0]'
        0 0 0 1];

hold on
grid on

DrawFrame(T_i,1,.35);
DrawVehicle(eta, [a1;a2;a3],PARAM.type);
DrawRobot(eta,DH,PARAM.T_0_B);

T_i_n = DirectKinematics(eta,DH,PARAM.T_0_B);
DrawFrame(T_i_n,0,.35);

view([1 3 .5])
axis('equal')
xlabel('x'),ylabel('y'),zlabel('z')
set(gca,'XDir','reverse','ZDir','reverse');





