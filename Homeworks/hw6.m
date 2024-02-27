
clear 
clc
rotx = @(t) [1 0 0; 0 cos(t) -sin(t) ; 0 sin(t) cos(t)] ;
roty = @(t) [cos(t) 0 sin(t) ; 0 1 0 ; -sin(t) 0  cos(t)] ;
rotz = @(t) [cos(t) -sin(t) 0 ; sin(t) cos(t) 0 ; 0 0 1] ;

R = @(x,y,z) rotz(x)*roty(y)*rotz(z);

Rsb = R(0,pi/6,0)'*R(pi/4,0,0)

w = logm(Rsb)

Vb = [-w(3,2);w(1,3);-w(1,2)]

[0,pi/2,0]' + [0.5 0 0; 0 1 0; sqrt(3)/2 0 1]*Vb

