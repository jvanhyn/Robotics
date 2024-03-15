close all; clear; clc; 
addpath(cd,'../mr')
%% FeedbackControl test script

% Current Desired Robot Position
Xd_i = [...
    0   0   1   0.5   ;
    0   1   0   0     ;
    -1  0   0   0.5   ;
    0   0   0   1     ;
    ];
%

% Next Desired Robot Position
Xd_f = [...
    0   0   1   0.6   ;
    0   1   0   0     ;
    -1  0   0   0.3   ;
    0   0   0   1     ;
    ];
%

% Current Robot Position 
X = [...
    0.170   0   0.985   0.387   ;
    0       1   0       0       ;
    -0.985  0   0.170   0.570   ;
    0       0   0       1       ;
    ];

dt = 0.01;

Kp = zeros(6);
Ki = zeros(6);

q = [0, 0, 0, 0, 0, 0.2, -1.6, 0];
[Vb,du,dtheta] = FeedbackControl(q,X,Xd_i,Xd_f,Kp,Ki,dt);

