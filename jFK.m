function varargout = jFK(S,M,THETA)

n = length(S);
V = cell(n,1);
T = cell(n,1);
T_FK = cell(n,1);
P = cell(n,1);

V{1} = VecTose3(S{1})*THETA(1);
T{1} = expm(V{1});
T_FK{1} = T{1}*M{1};
P{1} = T_FK{1}*[0,0,0,1]';

for i = 2:n
    V{i} = VecTose3(S{i})*THETA(i);
    T{i} = T{i-1}*expm(V{i});
    T_FK{i} = T{i}*M{i};
    P{i} = T_FK{i}*[0,0,0,1]';
end


varargout = {P,T_FK,V};

    function se3mat = VecTose3(V)
        se3mat = [VecToso3(V(1: 3)), V(4: 6); 0, 0, 0, 0];
    end
    function so3mat = VecToso3(omg)
        so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
    end
end

