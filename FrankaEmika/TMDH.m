function T = TMDH(a,alpha,d,theta)
%TMDH Summary of this function goes here

% return homogeneous transform from i to i-1
% equ 3.6 in Introduction to Robotics Carig

st = sin(theta);
ct = cos(theta);

sa = sin(alpha);
ca = cos(alpha);


R = [ct, -st,0;
    st*ca,ct*ca,-sa;
    st*sa,ct*sa,ca];
p = [a,-sa*d,ca*d]';
T = [R,p;
    0,0,0,1];

end

