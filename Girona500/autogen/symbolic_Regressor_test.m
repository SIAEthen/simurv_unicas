syms x y z rx ry rz real
syms q1 q2 q3 q4 real
syms theta [20,1] real
Y_sym = Regressor_Girona500([x,y,z,rx,ry,rz]',[q1,q2,q3,q4]',GironaPARAMS);
answer = Y_sym*theta;
a = simplify(answer);
a(1) % sin(ry)*(theta1 + theta5 + theta9 + theta13 + theta17)
a(2) % -cos(ry)*sin(rx)*(theta1 + theta5 + theta9 + theta13 + theta17)
a(3) % -cos(rx)*cos(ry)*(theta1 + theta5 + theta9 + theta13 + theta17)
a(4)
a(5)
a(6)
subs(a(6),[x, y, z, rx, rz],[0,0,0,0,0])

Y_sym(:,1:4)
Y_sym(:,5:8)
Y_sym(:,9:12)
Y_sym(:,13:16)
Y_sym(:,17:4)

matlabFunction(Y_sym,"File","Girona500/autogen/Y_sym_function","Vars",[x,y,z,rx,ry,rz,q1,q2,q3,q4],"Comments","Version: 1.1");