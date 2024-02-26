function S_list = R_screw(w,q)
    n = length(w);
    v = zeros(3,n);
    S_list = zeros(6,n);

    for i = 1:n
        v(:,i) = cross(-w(:,i),q(:,i));
        S_list(:,i) = [w(:,i);v(:,i)];
    end
end 