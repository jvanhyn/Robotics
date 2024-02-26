clear all 
clc 
close all
addpath("/Users/jvanhyn/Documents/GitHub/Robotics/mr")

% Given link lengths
l1 = 152;
l2 = 120;
l3 = 244;
l4 = 93;
l5 = 213;
l6 = 104;
l7 = 85;
l8 = 92;

% Relative joint positions
q1 = [0, -300,  88]'                    ;
q2 = [0,    0,  l1]'                    ;
q3 = [0,   l2,   0]' + [l3,   0,   0]'  ;
q4 = [0,  -l4,   0]' + [l5,   0,   0]'  ;
q5 = [0,   l6,   0]' + [0,    0, -l7]'  ;
q6 = [0,   l8,   0]'                    ;

% Spacial joint positions
sq1 =       q1;
sq2 = sq1 + q2;
sq3 = sq2 + q3;
sq4 = sq3 + q4;
sq5 = sq4 + q5;
sq6 = sq5 + q6;

% Rotation axes at each joint
w1 = [0,0,1]';
w2 = [0,1,0]';
w3 = [0,1,0]';
w4 = [0,1,0]';
w5 = [0,0,-1]';
w6 = [0,1,0]';

% Linear velocities at each joint
v1 = cross(-w1,sq1);
v2 = cross(-w2,sq2);
v3 = cross(-w3,sq3);
v4 = cross(-w4,sq4);
v5 = cross(-w5,sq5);
v6 = cross(-w6,sq6);

% Screw axes
S1 = [w1;v1];
S2 = [w2;v2];
S3 = [w3;v3];
S4 = [w4;v4];
S5 = [w5;v5];
S6 = [w6;v6];

% Inuput Joint Angles
THETA1 = [-20, -40, 60, 10, 30, 0];
THETA2 = [5, 10, -30, 230, -50, 150];
THETA3 = [-15, -10, -45, 30, 20, 0]; 

% Cell Array to hold theta values for for loop
mat_THETA = {THETA1,THETA2,THETA3};

% Zero Position for each joint
M1 = [eye(3),sq1;0,0,0,1];
M2 = [eye(3),sq2;0,0,0,1];
M3 = [eye(3),sq3;0,0,0,1];
M4 = [eye(3),sq4;0,0,0,1];
M5 = [eye(3),sq5;0,0,0,1];
M6 = [eye(3),sq6;0,0,0,1];

% End Effector Zero position
p_ee = [0,78,0]';
M_ee = M6*[eye(3),p_ee;0,0,0,1];

% Spacial Frame origin 
p = [0,0,0,1]';


for i = 1:length(mat_THETA)
     THETA = mat_THETA{i};
     theta_1 = THETA(1)/180*pi;
     theta_2 = THETA(2)/180*pi;
     theta_3 = THETA(3)/180*pi;
     theta_4 = THETA(4)/180*pi;
     theta_5 = THETA(5)/180*pi;
     theta_6 = THETA(6)/180*pi;

     V1 = VecTose3(S1)*theta_1;
     V2 = VecTose3(S2)*theta_2;
     V3 = VecTose3(S3)*theta_3;
     V4 = VecTose3(S4)*theta_4;
     V5 = VecTose3(S5)*theta_5;
     V6 = VecTose3(S6)*theta_6;

     T1 = expm(V1)*M1;
     T2 = expm(V1)*expm(V2)*M2;
     T3 = expm(V1)*expm(V2)*expm(V3)*M3;
     T4 = expm(V1)*expm(V2)*expm(V3)*expm(V4)*M4;
     T5 = expm(V1)*expm(V2)*expm(V3)*expm(V4)*expm(V5)*M5;   
     T6 = expm(V1)*expm(V2)*expm(V3)*expm(V4)*expm(V5)*expm(V6)*M6;     
     T_ee = expm(V1)*expm(V2)*expm(V3)*expm(V4)*expm(V5)*expm(V6)*M_ee;

     mP1{i} = T1*p;
     mP2{i} = T2*p;
     mP3{i} = T3*p;
     mP4{i} = T4*p;
     mP5{i} = T5*p;
     mP6{i} = T6*p;
     mP_ee{i} = T_ee*p;

     Js1{i} = S1;
     Js2{i} = Adjoint(T1)*S2;
     Js3{i} = Adjoint(T2)*S3;
     Js4{i} = Adjoint(T3)*S4;
     Js5{i} = Adjoint(T4)*S5;
     Js6{i} = Adjoint(T5)*S6;

     Js{i} = [Js1{i} Js2{i} Js3{i} Js4{i} Js5{i} Js6{i}];

     [eigenvects{i},eigenvals{i}] = eig(Js{i}*Js{i}');
     eign{i} = diag(eigenvals{i});
     eigv{i} = eigenvects{i}(4:end,4:end);

end 

for i = 1:3
     P1 = mP1{i}; 
     P2 = mP2{1};
     P3 = mP3{i};
     P4 = mP4{i};
     P5 = mP5{i};
     P6 = mP6{i} ;
     P_ee = mP_ee{i};

     % Plot the configuration 
     figure(i)
     ax = axes();
     xlim(ax, [-600 600]);
     ylim(ax, [-600 600]);
     zlim(ax, [0 1200]);
     view(ax, 3)
     hold(ax, 'on')
     view(-145,35)
     grid on

     % plot the default configuration
     sz = 100;
     quiver3(0,0,0,1,0,0,sz,'LineWidth',3)
     quiver3(0,0,0,0,1,0,sz,'LineWidth',3)
     quiver3(0,0,0,0,0,1,sz,'LineWidth',3)
     ps0 = scatter3(0,0,0,30,"ob","filled");



     % links
     pL2 = quiver3(sq1(1),sq1(2),sq1(3),q2(1),q2(2),q2(3),0);
     pL3 = quiver3(sq2(1),sq2(2),sq2(3),q3(1),q3(2),q3(3),0);
     pL4 = quiver3(sq3(1),sq3(2),sq3(3),q4(1),q4(2),q4(3),0);
     pL5 = quiver3(sq4(1),sq4(2),sq4(3),q5(1),q5(2),q5(3),0);
     pL6 = quiver3(sq5(1),sq5(2),sq5(3),q6(1),q6(2),q6(3),0);
     pLee = quiver3(sq6(1),sq6(2),sq6(3),p_ee(1),p_ee(2),p_ee(3),0);

     % joints 
     pJ1 = scatter3(sq1(1),sq1(2),sq1(3),"filled");
     pJ2 = scatter3(sq2(1),sq2(2),sq2(3),"filled");
     pJ3 = scatter3(sq3(1),sq3(2),sq3(3),"filled");
     pJ4 = scatter3(sq4(1),sq4(2),sq4(3),"filled");
     pJ5 = scatter3(sq5(1),sq5(2),sq5(3),"filled");
     pJ6 = scatter3(sq6(1),sq6(2),sq6(3),"filled");
     pJee = scatter3(sq6(1),sq6(2)+78,sq6(3),"filled");


     % plot the transformed robot
     % links
     pL2_ = quiver3(P1(1),P1(2),P1(3),P2(1)-P1(1),P2(2)-P1(2),P2(3)-P1(3),0);
     pL3_ = quiver3(P2(1),P2(2),P2(3),P3(1)-P2(1),P3(2)-P2(2),P3(3)-P2(3),0);
     pL4_ = quiver3(P3(1),P3(2),P3(3),P4(1)-P3(1),P4(2)-P3(2),P4(3)-P3(3),0);
     pL5_ = quiver3(P4(1),P4(2),P4(3),P5(1)-P4(1),P5(2)-P4(2),P5(3)-P4(3),0);
     pL6_ = quiver3(P5(1),P5(2),P5(3),P6(1)-P5(1),P6(2)-P5(2),P6(3)-P5(3),0);
     pLee_ = quiver3(P6(1),P6(2),P6(3),P_ee(1)-P6(1),P_ee(2)-P6(2),P_ee(3)-P6(3),0);

     % joints
     pJ1_ = scatter3(P1(1),P1(2),P1(3),"filled");
     pJ2_ = scatter3(P2(1),P2(2),P2(3),"filled");
     pJ3_ = scatter3(P3(1),P3(2),P3(3),"filled");
     pJ4_ = scatter3(P4(1),P4(2),P4(3),"filled");
     pJ5_ = scatter3(P5(1),P5(2),P5(3),"filled");
     pJ6_ = scatter3(P6(1),P6(2),P6(3),"filled");
     pJee_ = scatter3(P_ee(1),P_ee(2),P_ee(3),"filled");


     thick = 2;
     basecolor = '[.7 .7 .7]';
     jointcolor = '[.7 .7 .7]';

     pL2.ShowArrowHead = 'off';
     pL3.ShowArrowHead = 'off';
     pL4.ShowArrowHead = 'off';
     pL5.ShowArrowHead = 'off';
     pL6.ShowArrowHead = 'off';
     pLee.ShowArrowHead = 'off';
     set(pL2,"lineWidth",thick)
     set(pL3,"lineWidth",thick)
     set(pL4,"lineWidth",thick)
     set(pL5,"lineWidth",thick)
     set(pL6,"lineWidth",thick)
     set(pLee,"lineWidth",thick)
     set(pJ1,"LineWidth",thick)
     set(pJ2,"LineWidth",thick)
     set(pJ3,"LineWidth",thick)
     set(pJ4,"LineWidth",thick)
     set(pJ5,"LineWidth",thick)
     set(pJ6,"LineWidth",thick)
     set(pJee,"LineWidth",thick)
     set(pL2,"Color",basecolor)
     set(pL3,"Color",basecolor)
     set(pL4,"Color",basecolor)
     set(pL5,"Color",basecolor)
     set(pL6,"Color",basecolor)
     set(pLee,"Color",basecolor)
     set(pJ1,"MarkerFaceColor",jointcolor)
     set(pJ2,"MarkerFaceColor",jointcolor)
     set(pJ3,"MarkerFaceColor",jointcolor)
     set(pJ4,"MarkerFaceColor",jointcolor)
     set(pJ5,"MarkerFaceColor",jointcolor)
     set(pJ6,"MarkerFaceColor",jointcolor)
     set(pJee,"MarkerFaceColor",jointcolor)


     thick_ = 3;
     pL2_.ShowArrowHead = 'off';
     pL3_.ShowArrowHead = 'off';
     pL4_.ShowArrowHead = 'off';
     pL5_.ShowArrowHead = 'off';
     pL6_.ShowArrowHead = 'off';
     set(pL2_,"Color",'k')
     set(pL3_,"Color",'k')
     set(pL4_,"Color",'k')
     set(pL5_,"Color",'k')
     set(pL6_,"Color",'k')
     set(pLee_,"Color",'k')
     set(pJ1_,"MarkerFaceColor","#77AC30")
     set(pJ2_,"MarkerFaceColor","#77AC30")
     set(pJ3_,"MarkerFaceColor","#77AC30")
     set(pJ4_,"MarkerFaceColor","#77AC30")
     set(pJ5_,"MarkerFaceColor","#77AC30")
     set(pJ6_,"MarkerFaceColor","#77AC30")
     set(pJee_,"MarkerFaceColor","red")
     set(pL2_,"lineWidth",thick_)
     set(pL3_,"lineWidth",thick_)
     set(pL4_,"lineWidth",thick_)
     set(pL5_,"lineWidth",thick_)
     set(pL6_,"lineWidth",thick_)
     set(pLee_,"lineWidth",thick_)
     set(pJ1_,"LineWidth",thick_)
     set(pJ2_,"LineWidth",thick_)
     set(pJ3_,"LineWidth",thick_)
     set(pJ4_,"LineWidth",thick_)
     set(pJ5_,"LineWidth",thick_)
     set(pJ6_,"LineWidth",thick_)

     txt = sprintf('X: %9.2f\nY: %9.2f\nZ: %9.2f', P6(1:3));
     xL=xlim;
     yL=ylim;
     zL=zlim;
     offset = 0.95;
     % text(offset*xL(1),offset*yL(1),offset*zL(2),txt,'HorizontalAlignment','right','VerticalAlignment','top')
     % text(-20,0,0,'\{s\}','HorizontalAlignment','left','VerticalAlignment','bottom')
     % text(P_ee(1)+20,P_ee(2),P_ee(3)+20,'\{b\}','HorizontalAlignment','right','VerticalAlignment','bottom')
     % title("Configuration " + string(i))
     % g = 0.0001;
     % [X,Y,Z] = ellipsoid(0,0,0,g,g,g); 

     % for j = 1:21
     %      for k = 1:21
     %           TEMP = eigenvals{i}*eigenvects{i}*[0;0;0;X(j,k);Y(j,k);Z(j,k)];
     %           A{i}(j,k,:) = TEMP(4:end) + mP_ee{i}(1:3);
     %      end
     % end

     % sf = surf(A{i}(:,:,1),A{i}(:,:,2),A{i}(:,:,3),"LineStyle","none");
     % alpha(sf, 0.5);

     hold off

end

function se3mat = VecTose3(V)
     se3mat = [VecToso3(V(1: 3)), V(4: 6); 0, 0, 0, 0];
     end
function so3mat = VecToso3(omg)
    so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
    end


