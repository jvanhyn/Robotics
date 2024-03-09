clear
clc

% Dimentions (M)
W_1 = 109e-3;
W_2 = 82e-3;
L_1 = 425e-3;
L_2 = 392e-3;
H_1 = 89e-3;
H_2 = 95e-3;

% Joint Locations in Body Frame
qb1 = [L_1+L_2,0,-W_1-W_2]';
qb2 = [L_1+L_2,H_2,0]';
qb3 = [L_2,H_2,0]';
qb4 = [0,H_2,0]';
qb5 = [0,0,-W_2]';
qb6 = [0,0,0]';

qb_list = [qb1,qb2,qb3,qb4,qb5,qb6];

% Rotation Axes in Body Frame
wb1 = [ 0,  1,  0]';
wb2 = [ 0,  0,  1]';
wb3 = [ 0,  0,  1]';
wb4 = [ 0,  0,  1]';
wb5 = [ 0, -1,  0]';
wb6 = [ 0,  0,  1]';

wb_list = [wb1,wb2,wb3,wb4,wb5,wb6];

Blist = R_screw(wb_list,qb_list);

M = [-1,0,0,L_1+L_2;
      0,0,1,W_1+W_2;
      0,1,0,H_1-H_2;
      0,0,0,1];

T = [ 0, 1, 0,-0.5;
      0, 0,-1,-0.1;
      -1, 0, 0,-0.1;
       0, 0, 0,   1];

eomg = 0.001;
ev =  0.001;


thetalist0 = [0.2560,1.0865,1.8352,1.3615,-2.8856,-1.5707]';
[thetalist, success] = IKinBodyIterations(Blist, M, T, thetalist0, eomg, ev);

% Quick fucntion for getting the screw axis from omega and q
function S_list = R_screw(w,q) 
    n = length(w);
    v = zeros(3,n);
    S_list = zeros(6,n);

    for i = 1:n
        v(:,i) = cross(-w(:,i),q(:,i));
        S_list(:,i) = [w(:,i);v(:,i)];
    end
end 

% Requested fucntion for getting inverse kinematics and itteration reports
function [thetalist, success] = IKinBodyIterations(Blist, M, T, thetalist0, eomg, ev)

    thetalist = thetalist0;
    i = 0;
    maxiterations = 20;
    Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
    err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    Tsb = FKinBody(M, Blist, thetalist);
    
    disp("<>= Inverse Kinematics Itteration Report =<>") 
    disp("---------------------------------------------")

    % Print the initial guess before any interations
    disp("Iteration:")
    disp(i)
    disp("Configuration:")
    disp(thetalist)
    disp("Twist:")
    disp(Vb)
    disp("Position:")
    disp(Tsb)
    disp("Rotation Error:")
    disp(norm(Vb(1: 3)))
    disp("Position Error:")
    disp(norm(Vb(4: 6)))
    % save theta
    thetaitterations(:,i+1) = thetalist;

    while err && i < maxiterations
        thetalist = wrapToPi(thetalist);
        thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;
        i = i + 1;
        Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
        err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;

        Tsb = FKinBody(M, Blist, thetalist)
         
        % save theta
        thetaitterations(:,i+1) = thetalist;
        % Print the itteration report
        disp("---------------------------------------------")
        disp(" ")
        disp("Iteration:")
        disp(i)
        disp("Configuration:")
        disp(thetalist)
        disp("Twist:")
        disp(Vb)
        disp("Position:")
        disp(Tsb)
        disp("Rotation Error:")
        disp(norm(Vb(1: 3)))
        disp("Position Error:")
        disp(norm(Vb(4: 6)))
        
    end
    disp("-------------End of Report-----------------")
    writematrix(thetaitterations',"thetaitterations.csv")
    success = ~ err;
end