function [thetalist, success] = IKinBody(Blist, M, T, thetalist0, eomg, ev)

    thetalist = thetalist0;
    i = 0;
    maxiterations = 20;
    Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
    err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    while err && i < maxiterations
        thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;
        i = i + 1;
        Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
        err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    end
    success = ~ err;
    end