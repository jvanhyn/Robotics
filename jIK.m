function varargout = jIK(S,M,THETA0, XYZd,thresh)
addpath("/Users/jvanhyn/Documents/GitHub/Robotics/mr");
n = length(S);

THETA = THETA0;

T = eye(4);
J = zeros(6,n);

for i = 1:n
    J(:,i) = Adjoint(T)*S{i};
    V = VecTose3(S{i})*THETA(i);
    T = T*expm(V);
end
T_FK = T*M;

% XYZc = T_FK*[0,0,0,1]';
% e = XYZc(1:3) - XYZd;

% while norm(e) > thresh
%     for i = 1:n
%         Js(:,i) = Adjoint(T_FK)*S{i};
%         V = VecTose3(S{i})*THETA(i);
%         T = T*expm(V);
%         T_FK = T*M{i};
%     end
    
% end

varargout = {J};

    function se3mat = VecTose3(V)
        se3mat = [VecToso3(V(1: 3)), V(4: 6); 0, 0, 0, 0];
    end
    function so3mat = VecToso3(omg)
        so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
    end
end