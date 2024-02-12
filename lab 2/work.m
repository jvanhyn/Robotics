clear  
close all

l1 = 152;
l2 = 120;
l3 = 244;
l4 = 93;
l5 = 213;
l6 = 104;
l7 = 85;
l8 = 92;


q1 = [0, -300,  88]'                    ;
q2 = [0,    0,  l1]'                    ;
q3 = [0,   l2,   0]' + [l3,   0,   0]'  ;
q4 = [0,  -l4,   0]' + [l5,   0,   0]'  ;
q5 = [0,   l6,   0]' + [0,    0, -l7]'  ;
q6 = [0,   l8,   0]'                    ;

sq1 =       q1;
sq2 = sq1 + q2;
sq3 = sq2 + q3;
sq4 = sq3 + q4;
sq5 = sq4 + q5;
sq6 = sq5 + q6;

w1 = [0,0,1]';
w2 = [0,1,0]';
w3 = [0,1,0]';
w4 = [0,1,0]';
w5 = [0,0,1]';
w6 = [0,1,0]';

v1 = cross(-w1,sq1);
v2 = cross(-w2,sq2);
v3 = cross(-w3,sq3);
v4 = cross(-w4,sq4);
v5 = cross(-w5,sq5);
v6 = cross(-w6,sq6);

S1 = [w1;v1];
S2 = [w2;v2];
S3 = [w3;v3];
S4 = [w4;v4];
S5 = [w5;v5];
S6 = [w6;v6];
a = 5;
theta_1 = 0;
theta_2 = 0.1*a;
theta_3 = -0.2*a;
theta_4 = 0;
theta_5 = 0;
theta_6 = 0;

V1 = VecTose3(S1*theta_1);
V2 = VecTose3(S2*theta_2);
V3 = VecTose3(S3*theta_3);
V4 = VecTose3(S4*theta_4);
V5 = VecTose3(S5*theta_5);
V6 = VecTose3(S6*theta_6);

M1 = [eye(3),sq1;0,0,0,1];
M2 = [eye(3),sq2;0,0,0,1];
M3 = [eye(3),sq3;0,0,0,1];
M4 = [eye(3),sq4;0,0,0,1];
M5 = [eye(3),sq5;0,0,0,1];
M6 = [eye(3),sq6;0,0,0,1];

T1 = expm(V1)*M1;
T2 = expm(V1 + V2)*M2;
T3 = expm(V1 + V2+ V3)*M3;
T4 = expm(V1 + V2+ V3 + V4)*M4;
T5 = expm(V1 + V2 + V3+ V4 + V5)*M5;
T6 = expm(V1 + V2 + V3+ V4 + V5 + V6)*M6;

p = [0,0,0,1]';
P1 = T1*p;
P2 = T2*p;
P3 = T3*p;
P4 = T4*p;
P5 = T5*p;
P6 = T6*p;

ax = axes();
xlim(ax, [-600 600]);
ylim(ax, [-600 600]);
zlim(ax, [-600 600]);
view(ax, 3)
hold(ax, 'on')
pbaspect([1 1 1])

grid on

% plot the default configuration
pL1 = quiver3(0,0,0,q1(1),q1(2),q1(3),0);
pL2 = quiver3(sq1(1),sq1(2),sq1(3),q2(1),q2(2),q2(3),0);
pL3 = quiver3(sq2(1),sq2(2),sq2(3),q3(1),q3(2),q3(3),0);
pL4 = quiver3(sq3(1),sq3(2),sq3(3),q4(1),q4(2),q4(3),0);
pL5 = quiver3(sq4(1),sq4(2),sq4(3),q5(1),q5(2),q5(3),0);
pL6 = quiver3(sq5(1),sq5(2),sq5(3),q6(1),q6(2),q6(3),0);

pJ1 = scatter3(sq1(1),sq1(2),sq1(3),"filled");
pJ2 = scatter3(sq2(1),sq2(2),sq2(3),"filled");
pJ3 = scatter3(sq3(1),sq3(2),sq3(3),"filled");
pJ4 = scatter3(sq4(1),sq4(2),sq4(3),"filled");
pJ5 = scatter3(sq5(1),sq5(2),sq5(3),"filled");
pJ6 = scatter3(sq6(1),sq6(2),sq6(3),"filled");
ps0 = scatter3(0,0,0,"filled");

% w_scale = 50;
% pR1 = quiver3(sq1(1),sq1(2),sq1(3),w1(1),w1(2),w1(3),w_scale);
% pR2 = quiver3(sq2(1),sq2(2),sq2(3),w2(1),w2(2),w2(3),w_scale);
% pR3 = quiver3(sq3(1),sq3(2),sq3(3),w3(1),w3(2),w3(3),w_scale);
% pR4 = quiver3(sq4(1),sq4(2),sq4(3),w4(1),w4(2),w4(3),w_scale);
% pR5 = quiver3(sq5(1),sq5(2),sq5(3),w5(1),w5(2),w5(3),w_scale);
% pR6 = quiver3(sq6(1),sq6(2),sq6(3),w6(1),w6(2),w6(3),w_scale);

% plot the transformed robot
pL1_ = quiver3(0    ,0    ,0    ,P1(1)      ,P1(2)      ,P1(3)      ,0);
pL2_ = quiver3(P1(1),P1(2),P1(3),P2(1)-P1(1),P2(2)-P1(2),P2(3)-P1(3),0);
pL3_ = quiver3(P2(1),P2(2),P2(3),P3(1)-P2(1),P3(2)-P2(2),P3(3)-P2(3),0);
pL4_ = quiver3(P3(1),P3(2),P3(3),P4(1)-P3(1),P4(2)-P3(2),P4(3)-P3(3),0);
pL5_ = quiver3(P4(1),P4(2),P4(3),P5(1)-P4(1),P5(2)-P4(2),P5(3)-P4(3),0);
pL6_ = quiver3(P5(1),P5(2),P5(3),P6(1)-P5(1),P6(2)-P5(2),P6(3)-P5(3),0);

pJ1_ = scatter3(P1(1),P1(2),P1(3),"filled");
pJ2_ = scatter3(P2(1),P2(2),P2(3),"filled");
pJ3_ = scatter3(P3(1),P3(2),P3(3),"filled");
pJ4_ = scatter3(P4(1),P4(2),P4(3),"filled");
pJ5_ = scatter3(P5(1),P5(2),P5(3),"filled");
pJ6_ = scatter3(P6(1),P6(2),P6(3),"filled");


basecolor = '[.7 .7 .7]';
jointcolor = '[.7 .7 .7]';
set(pL1,"Color",basecolor)
set(pL2,"Color",basecolor)
set(pL3,"Color",basecolor)
set(pL4,"Color",basecolor)
set(pL5,"Color",basecolor)
set(pL6,"Color",basecolor)
set(pJ1,"MarkerFaceColor",jointcolor)
set(pJ2,"MarkerFaceColor",jointcolor)
set(pJ3,"MarkerFaceColor",jointcolor)
set(pJ4,"MarkerFaceColor",jointcolor)
set(pJ5,"MarkerFaceColor",jointcolor)
set(pJ6,"MarkerFaceColor",jointcolor)
set(ps0,"MarkerFaceColor",jointcolor)
% set(pR1,"Color","#A2142F")
% set(pR2,"Color","#A2142F")
% set(pR3,"Color","#A2142F")
% set(pR4,"Color","#A2142F")
% set(pR5,"Color","#A2142F")
% set(pR6,"Color","#A2142F")


set(pL1_,"Color",basecolor)
set(pL2_,"Color",basecolor)
set(pL3_,"Color",basecolor)
set(pL4_,"Color",basecolor)
set(pL5_,"Color",basecolor)
set(pL6_,"Color",basecolor)
set(pJ1_,"MarkerFaceColor","#77AC30")
set(pJ2_,"MarkerFaceColor","#77AC30")
set(pJ3_,"MarkerFaceColor","#77AC30")
set(pJ4_,"MarkerFaceColor","#77AC30")
set(pJ5_,"MarkerFaceColor","#77AC30")
set(pJ6_,"MarkerFaceColor","red")
set(ps0,"MarkerFaceColor","blue")
hold off









function se3mat = VecTose3(V)
     se3mat = [VecToso3(V(1: 3)), V(4: 6); 0, 0, 0, 0];
     end
function so3mat = VecToso3(omg)
    so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
    end


