function [thetalist, success] = IKinBodyIterations(Blist, M, T, thetalist0, eomg, ev)

    thetalist = thetalist0;
    i = 0;
    maxiterations = 20;
    Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
    err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    Tsb = FKinBody(M, Blist, thetalist)

    disp("Iteration:")
    disp(i)
    disp("Configuration:")
    disp(thetalist)
    disp("Twist")
    disp(Vb)
    disp("Position")
    disp(Tsb)
    disp("Rotation Error")
    disp(norm(Vb(1: 3)))
    disp("Position Error")
    disp(norm(Vb(4: 6)))
    thetaitterations(:,i+1) = thetalist;

    while err && i < maxiterations
        thetalist = wrapToPi(thetalist);
        thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;
        i = i + 1;
        Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
        err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;

        Tsb = FKinBody(M, Blist, thetalist)

        thetaitterations(:,i+1) = thetalist;
        disp("Iteration:")
        disp(i)
        disp("Configuration:")
        disp(thetalist)
        disp("Twist")
        disp(Vb)
        disp("Position")
        disp(Tsb)
        disp("Rotation Error")
        disp(norm(Vb(1: 3)))
        disp("Position Error")
        disp(norm(Vb(4: 6)))

    end
    
    writematrix(thetaitterations',"thetaitterations.csv")
    success = ~ err;
end