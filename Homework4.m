addpath('/Users/jvanhyn/Documents/GitHub/Robotics/mr')

Fe = [0,0,0,0,0,-10]';

Tce = [0,0,1,-75;-1/sqrt(2),1/sqrt(2),0,-260/sqrt(2);-1/sqrt(2),-1/sqrt(2),0,160/sqrt(2);0,0,0,1];
Tec = inv(Tce)

Ad_Tce = Adjoint(Tec);

Fc = (Ad_Tce)'*Fe