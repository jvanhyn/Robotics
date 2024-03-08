function [q,theta,u] = NextState(q0,u0,theta0,du,dtheta,dt,du_max)
    for i = 1:4
        if(abs(du(i)) > du_max)
            du(i) = du_max;
        end
    end
    
    u = u0 + du.*dt;
    theta = theta0 + dtheta.*dt;

    l = 1;
    w = 1;
    r = 1;

    F = r/4 * [-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1];
    V_b =  F*du;

    wzb = V_b(1);
    vxb = V_b(2);
    vyb = V_b(3);

    if wzb == 0 
        dqb = [wzb;vxb;vyb];
    else 
        dqb = [wzb;
              (vxb*sin(wzb)+vyb*(cos(wzb)-1))/wzb;
              (vyb*sin(wzb)+vxb*(1-cos(wzb)))/wzb];
    end

    phi = q0(1);
    dqs = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)]*dqb;

    q = q0 + dqs * dt;
end