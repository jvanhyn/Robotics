% Joint Locatioins
q1 = [1,0,0]';
q2 = [1,0,0]';
q3 = [1,0,0]';
q4 = [1,0,0]';
q5 = [1,0,0]';
q6 = [1,0,0]';

q_list = [q1,q2,q3,q4,q5,q6];

% Rotation axes at each joint
w1 = [ 0, 0,  1]';
w2 = [ 0, 1,  0]';
w3 = [ 0, 1,  0]';
w4 = [ 0, 1,  0]';
w5 = [ 0, 0, -1]';
w6 = [ 0, 1,  0]';

w_list = [w1,w2,w3,w4,w5,w6];

Slist = R_screw(w_list,q_list)

function S_list = R_screw(w,q)
    n = length(w);
    v = zeros(3,n);
    S_list = zeros(6,n);

    for i = 1:n
        v(:,i) = cross(-w(:,i),q(:,i));
        S_list(:,i) = [w(:,i);v(:,i)];
    end
end 