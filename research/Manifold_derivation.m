clear; clc;
syms a1 T p1 p2 v1 v2;
p1=0; v1=0; a1=1;

a2 = -a1;
ta1 = (T - (v2 - v1)/a2)/2;
f = collect(expand(a1*ta1^2 + 2*v1*ta1 + (v2^2-v1^2)/(2*a2) - (p2-p1)),T);
F = matlabFunction(f)

p2 = -1:0.05:1;
v2 = -1:0.05:1;
T = 0:0.05:1;
[X,Y,Z] = meshgrid(p2,v2);
%Z = arrayfun(@F, repmat(p1,size(X)), repmat(v1,size(Y)), X, Y);
