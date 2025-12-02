Y = [];
for i=1:10000
    x = rand();
    y = rand();
    z = rand();
    rx = rand();
    ry = rand();
    rz = rand();
    q1 = rand();
    q2 = rand();
    q3 = rand();
    q4 = rand();
    Y_i = Y_sym_function(x,y,z,rx,ry,rz,q1,q2,q3,q4);
    Y = [Y; Y_i];
end
rank(Y) %12
[R, pivots] = rref(Y);
disp(pivots) % 1     2     3     4     6     7     9    12    13    15    17    20
rank(Y(:,1:4)) % 4
rank(Y(:,5:8)) % 4
rank(Y(:,9:12)) % 4
rank(Y(:,13:16)) % 4
rank(Y(:,17:20)) % 4
rank(Y(:,5:20)) % 10