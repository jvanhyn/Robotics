addpath('/Users/jvanhyn/Documents/GitHub/Robotics/mr')

Fe = [0,0,0,0,0,-10]';

Tce = [0,0,1,-75;-1/sqrt(2),1/sqrt(2),0,-260/sqrt(2);-1/sqrt(2),-1/sqrt(2),0,160/sqrt(2);0,0,0,1];
Tec = inv(Tce)

Ad_Tce = Adjoint(Tec);

Fc = (Ad_Tce)'*Fe

L0 = 1
L1 = 3
L2 = 2
L3 = 1
theta = [pi/3;3;pi];

M = [-1,0,0,0;0,1,0,L0+L2;0,0,-1,L1-L3;0,0,0,1];

Slist = [[0  0 1 L0 0 0]' [0 0 0 0 1 0]' [0 0 -1 -L0-L2 0 -0.1]']
Blist = [[0 0 1 L2 0 0]' [0 0 0 0 1 0]' [0 0 1 0 0 0.1]']

SPACE = FKinSpace(M,Slist,theta)
BODY = FKinSpace(M,Blist,theta)