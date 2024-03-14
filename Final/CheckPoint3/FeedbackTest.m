close all; clear; clc; 
addpath(cd,'../mr')

Xd_i = [...
    0   0   1   0.5   ;
    0   1   0   0     ;
    -1  0   0   0.5   ;
    0   0   0   1     ;
    ];

Xd_f = [...
    0   0   1   0.6   ;
    0   1   0   0     ;
    -1  0   0   0.3   ;
    0   0   0   1     ;
    ];

X = [...
    0.170   0   0.985   0.387   ;
    0       1   0       0       ;
    -0.985  0   0.170   0.570   ;
    0       0   0       1       ;
    ];
dt = 0.01;

Kp = zeros(6,6);
Ki = zeros(6,6);

[Vd,AdjVd,Vb,Xe] = FeedbackControl(X,Xd_i,Xd_f,Kp,Ki,dt);


%% Solutions
du_dtheta_s = [157.2,157.2,157.2,157.2,0,-652.9,1398.6,-745.7,0];

check1 = [0;0;0;20;0;10] == round(Vd,3)
check2 = [0;0;0;21.409;0;6.455] == round(AdjVd,3)
check3 = [0;0;0;21.409;0;6.455] == round(Vb,3)
check4 = [0;0.171;0;0.080;0;0.107] == round(Xe,3)
% check5 = du_dtheta_s == [du,dtheta]

