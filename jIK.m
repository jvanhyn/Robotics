function varargout = jIK(S,M,THETA0, XYZd,thresh)
addpath("/Users/jvanhyn/Documents/GitHub/Robotics/mr");
n = length(S);

[XYZc, T, V] = jFK(S,M,THETA0)

Tsb = T{n};
Tbs = inv(Tsb);
Vb = Adjoint(Tbs)*V{n}

e = (XYZc - XYZd);

varargout = {e};

    function se3mat = VecTose3(V)
        se3mat = [VecToso3(V(1: 3)), V(4: 6); 0, 0, 0, 0];
    end
    function so3mat = VecToso3(omg)
        so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
    end
end