function DrawArrow(pa,pb,d1,d2,colr)
%
% DrawArrow(pa,pb)
%
% input:
%   pa   dim 3x1    "starting" point
%   pb   dim 3x1    "ending" point
%   d1   dim 1x1    body diameter
%   d2   dim 1x1    head diameter
%   colr dim 1x1    color
%
% G. Antonelli, Simurv 4.0, 2013
% http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

pa=CheckVector(pa);
pb=CheckVector(pb);

% the arrow is composed of a cylindrical body and a cone head starting
% at 70% of the segment pb-pa
pc = pa + 0.7*(pb-pa);

% ----------
% body
% ----------
% generate points on the cylinder
% aligned with x
[z,y,x] = cylinder(d1*ones(41,1),40);
x = norm(pc-pa)*x;
x2 = (pc-pa)/norm(pc-pa);
if ((x2(1)==-1)||(x2(1)==1))
    z2 = [0 0 1]';
else
    z2 = cross([1;0;0],x2); z2 = z2/norm(z2);
end
y2 = cross(x2,z2);
% rotate and translate points
R = [x2 y2 z2];
for i=1:length(x)
   for j=1:length(x)
      % rotation
      rr=R*[x(i,j) y(i,j) z(i,j)]';
      x(i,j) = rr(1);
      y(i,j) = rr(2); 
      z(i,j) = rr(3);
      % translation
      x(i,j) = x(i,j) + pa(1);
      y(i,j) = y(i,j) + pa(2); 
      z(i,j) = z(i,j) + pa(3);
   end
end
h = surfl(x,y,z);
set(h,'facecolor',colr,'edgecolor','none');

% ----------
% head
% ----------
% generate points on the cone
% aligned with x
[z,y,x] = cylinder(linspace(d2,0,41),40);
x = norm(pb-pc)*x;
x2 = (pb-pc)/norm(pb-pc);
if ((x2(1)==-1)||(x2(1)==1))
    z2 = [0 0 1]';
else
    z2 = cross([1;0;0],x2); z2 = z2/norm(z2);
end
y2 = cross(x2,z2);
% rotate and translate points
R = [x2 y2 z2];
for i=1:length(x)
   for j=1:length(x)
      % rotation
      rr=R*[x(i,j) y(i,j) z(i,j)]';
      x(i,j) = rr(1);
      y(i,j) = rr(2); 
      z(i,j) = rr(3);
      % translation
      x(i,j) = x(i,j) + pc(1);
      y(i,j) = y(i,j) + pc(2); 
      z(i,j) = z(i,j) + pc(3);
   end
end
hold on
h = surfl(x,y,z);
set(h,'facecolor',colr,'edgecolor','none');

