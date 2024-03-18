function [q,theta,u] = NextState(q0,u0,theta0,du,dtheta,dt,speed_max)

% NextState uses the velocity kinematics of a 5R mobile robot and the Euler method
% of approximation to predict how the robot will move in a small timestep
% given its current configuration and velocity.

% INPUTS:
% q0: current location of the chassis (in the set of R^3)
% u0: current chassis wheel angles (in the set of R^4)
% theta0: current joint angles (in the set of R^5)

% du: joint velocities (in the set of R^5)
% dtheta: wheel velocities (in the set of R^4)

% dt: timestep
% speed_max: maximum motor speed vector (dtheta1_max,dtheta2_max,dtheta3_max,dtheta4_max,dtheta5_max,du_max)

% OUTPUTS: configuration of the next state
% q: next location of the chassis (in the set of R^3)
% u: next chassis wheel angles (in the set of R^4)
% theta: next joint angles (in the set of R^5)

    % ensure that wheel speeds are under the speed threshold
    for i = 1:4
        if(abs(du(i)) > speed_max(6))
            du(i) = speed_max;
        end
    end

    % ensure that joint speeds are under the speed threshold
    for i = 1:5
        if(abs(dtheta(i)) > speed_max(i))
            dtheta(i) = speedmax;
        end
    end
    
    % Use Euler's method to approximate new wheel and joint angles
    u = u0 + du.*dt;
    theta = theta0 + dtheta.*dt;

    % Calculate the next state of the chassis configuration using odometry
   % AT: what are l,w,r?
   % JVH: w,l, and r are the dimentions of the mechanum wheels (width,length, radius). 
   l = 0.47/2;
   w = 0.3/2;
   r = 0.0475;
    
    % Calculate the body frame twist of the chassis
    F = r/4 * [-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1]; % F for a 4-mecanum-wheel chassis (see eqn 13.33 from mr)
    V_b =  F*du;

    wzb = V_b(1);
    vxb = V_b(2);
    vyb = V_b(3);
    
    % Extract change in coordinates relative to the chassis body frame
    if wzb == 0 
        dqb = [wzb;vxb;vyb];
    else 
        dqb = [wzb;
              (vxb*sin(wzb)+vyb*(cos(wzb)-1))/wzb;
              (vyb*sin(wzb)+vxb*(1-cos(wzb)))/wzb];
    end

    phi = q0(1); % current chassis angle
    dqs = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)]*dqb; %transform dq from frame {b} to frame {s}

    % Estimate new chassis configuration
    q = q0 + dqs * dt;
end