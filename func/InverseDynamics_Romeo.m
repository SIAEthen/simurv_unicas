function tau = InverseDynamics_Romeo(e, nu, dnu, rho, g0)
% Calculate Forces and moments acting on the vehicle Romeo
% based on
%  @article{caccia_joe00,
%    title={Modeling and identification of open-frame variable configuration unmanned underwater vehicles},
%    author={M.~Caccia and G.~Indiveri and G.~Veruggio},
%    journal={Oceanic Engineering, IEEE Journal of},
%    volume={25},
%    number={2},
%    pages={227--240},
%    year={2000},
%    publisher={IEEE}
%  }
%
% tau = InverseDynamics_vehicle(e, nu, dnu, rho, g0);
%
% input:
%       e      dim 4x1     vehicle orientation (quaternions)
%       nu     dim 6x1     vehicle velocity
%       dnu    dim 6x1     vehicle acceleration
%       rho    dim 1x1     water density
%       g0     dim 3x1     gravity
%
% output:
%       tau	   dim 6x1     vehicle forces
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

% u = nu(1);  
% v = nu(2);  
% w = nu(3);
% p = nu(4);  
% q = nu(5);  
% r = nu(6);

rv_g = [0 0 0.1];
rv_b = [0 0 0];
m_v   = 450;
if norm(g0)==0
    W = 0;
    B = 0;
else
    W = m_v*norm(g0);  
    B = 4714;
end

M_v = diag([700 900 1000 93 93 93]);
D   = diag([ 50 150  115 24 24 24]);
DD  = diag([320 360  430 31 31 31]);

% restoring forces and moments
xG = rv_g(1);  
yG = rv_g(2);    
zG = rv_g(3);
xB = rv_b(1);  
yB = rv_b(2);    
zB = rv_b(3);

t1 = e(1)^2;
t2 = e(2)^2;
t3 = e(3)^2;
%t4 = e(1)*e(2);
t5 = e(1)*e(3);
t6 = e(2)*e(3);
t7 = e(4)*e(1);
t8 = e(4)*e(2);
%t9 = e(4)*e(3);
t10 = e(4)^2;
temp1 = - t10 + t1 + t2 -t3;
temp2 = 2*(t8 - t5);
temp3 = 2*(t7 + t6);

g(1,1) =  temp2*(W - B);
g(2,1) = -temp3*(W - B);
g(3,1) =  temp1*(W - B);
g(4,1) =  temp1*(yG*W - yB*B) + temp3*(zG*W - zB*B);
g(5,1) = -temp1*(xG*W - xB*B) + temp2*(zG*W - zB*B);
g(6,1) = -temp3*(xG*W - xB*B) - temp2*(yG*W - yB*B);

tau = M_v*dnu + D*nu + DD*(nu.*abs(nu)) + g;

