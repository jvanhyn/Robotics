addpath("/Users/jvanhyn/Documents/GitHub/Robotics/mr");
syms L theta_1 theta_2 theta_3 real
theta_1 = deg2rad(0);
theta_2 = deg2rad(45);
theta_3 = deg2rad(-45);

S1 = [0,0,1,0,0,0]';
S2 = [0,-1,0,2*L,0,0]';
S3 = [0,-1,0,2*L,0,-L]';

B1 = [0,0,1,-L,2*L,0]';
B2 = [0,-1,0,0,0,-2*L]';
B3 = [0,-1,0,0,0,-L]';

Js1 = S1;
Js2 = Adjoint(expm(VecTose3(S1)*theta_1))*S2;
Js3 = Adjoint(expm(VecTose3(S1)*theta_1)*expm(VecTose3(S2)*theta_2))*S3;

Jb1 = Adjoint(expm(VecTose3(-B3)*theta_1)*expm(VecTose3(-B2)*theta_2))*B1;
Jb2 = Adjoint(expm(VecTose3(-B3)*theta_1))*B2;
Jb3 = B3;

Js = [Js1,Js2,Js3];
Jb = [Jb1,Jb2,Jb3];



function se3mat = VecTose3(V)
    se3mat = [VecToso3(V(1: 3)), V(4: 6); 0, 0, 0, 0];
    end
function so3mat = VecToso3(omg)
   so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
   end
