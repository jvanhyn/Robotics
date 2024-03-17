function [thetafinal] = IKinBodyIterations_1(Blist, M, T, thetalist0, eomg, ev)
% *** CHAPTER 6: INVERSE KINEMATICS ***
% Takes Blist: The joint screw axes in the end-effector frame when the
% manipulator is at the home position, in the format of a
% matrix with the screw axes as the columns,
% M: The home configuration of the end-effector,
% T: The desired end-effector configuration Tsd,
% thetalist0: An initial guess of joint angles that are close to
% satisfying Tsd,
% eomg: A small positive tolerance on the end-effector orientation
% error. The returned joint angles must give an end-effector
% orientation error less than eomg,
% ev: A small positive tolerance on the end-effector linear position
% error. The returned joint angles must give an end-effector
% position error less than ev.
thetalist(:,1) = thetalist0;
thetalist_deg(:,1) = rad2deg(thetalist0);
theta = char(hex2dec('03b8'));
i = 0;
T_sb = M;
maxiterations = 200;
Vb(:,i+1) = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist,thetalist(:,i+1))) * T));
w_mod(1,1) = norm(Vb(1:3,1));
v_mod(1,1) = norm(Vb(4:6,1));
err = norm(Vb(1: 3,i+1)) > eomg || norm(Vb(4: 6,i+1)) > ev;
fprintf('--------------------------------------------------------------------\n')
while err && i < maxiterations
 thetalist(:,i+2) = thetalist(:,i+1) + pinv(JacobianBody(Blist, thetalist(:,i+1))) * Vb(:,i+1);
 % joint limits check
 for jj = 1:length(thetalist0)
 if thetalist(jj,i+2) > pi || thetalist(jj,i+2) < -pi
 thetalist(jj,i+2) = wrapToPi(thetalist(jj,i+2));
 end
 end
 thetalist_deg(:,i+2) = rad2deg(thetalist(:,i+2));
 i = i + 1;
 T_sb = FKinBody(M, Blist, thetalist(:,i+1));
 Vb(:,i+1) = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist(:,i+1))) * T));
 err = norm(Vb(1: 3,i+1)) > eomg || norm(Vb(4: 6,i+1)) > ev;
 w_mod(1,i+1) = norm(Vb(1:3,i+1));
 v_mod(1,i+1) = norm(Vb(4:6,i+1));

 % Print Theta values in radians
 fprintf(['It %g: \n' theta '%g = '],i-1,i-1)
 for k = 1:length(thetalist(:,i))
 if k == 1
 fprintf('[%.3f, ',deg2rad(thetalist_deg(k,i)))
 elseif k > 1 && k < length(thetalist(:,i))
 fprintf('%.3f, ',deg2rad(thetalist_deg(k,i)))
 elseif k == length(thetalist(:,i))
 fprintf('%.3f] (rad)\n',deg2rad(thetalist_deg(k,i)))
 end
 end
 % Print Theta values in degrees
 fprintf(['' theta '%g = '],i-1)
 for k = 1:length(thetalist(:,i))
 if k == 1
 fprintf('[%.2f, ',thetalist_deg(k,i))
 elseif k > 1 && k < length(thetalist(:,i))
 fprintf('%.2f, ',thetalist_deg(k,i))
 elseif k == length(thetalist(:,i))
 fprintf('%.2f] (deg)\n',thetalist_deg(k,i))
 end
 end
 fprintf('Error twist Vb = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n',Vb(:,i))
 fprintf('Angular error magnitude, ||wb|| = %.3f (rad/s); ',w_mod(:,i))
 fprintf('Linear error magnitude, ||vb|| = %.3f (mm/s)\n',v_mod(:,i))
 fprintf('Current end-effector configuration: \n')
 fprintf('Tsb(%g) = \n',i-1)
 disp(FKinBody(M, Blist, thetalist(:,i)))

fprintf('--------------------------------------------------------------------\n')
end
% Print Theta values in radians
fprintf(['It %g: \n' theta '%g = '],i,i)
for k = 1:length(thetalist(:,i))
 if k == 1
 fprintf('[%.3f, ',deg2rad(thetalist_deg(k,i)))
 elseif k > 1 && k < length(thetalist(:,i))
 fprintf('%.3f, ',deg2rad(thetalist_deg(k,i)))
 elseif k == length(thetalist(:,i))
 fprintf('%.3f] (rad)\n',deg2rad(thetalist_deg(k,i)))
 end
end
% Print Theta values in degrees
fprintf(['' theta '%g = '],i)
for k = 1:length(thetalist(:,i))
 if k == 1
 fprintf('[%.2f, ',thetalist_deg(k,i))
 elseif k > 1 && k < length(thetalist(:,i))
 fprintf('%.2f, ',thetalist_deg(k,i))
 elseif k == length(thetalist(:,i))
 fprintf('%.2f] (deg)\n',thetalist_deg(k,i))
 end
end
fprintf('Error twist Vb = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n',Vb(:,i+1))
fprintf('Angular error magnitude, ||wb|| = %.3f (rad/s); ',w_mod(:,i+1))
fprintf('Linear error magnitude, ||vb|| = %.3f (mm/s)\n',v_mod(:,i+1))
fprintf('Current end-effector configuration: \n')
fprintf('Tsb(%g) = \n',i)
disp(FKinBody(M, Blist, thetalist(:,i+1)))
fprintf('--------------------------------------------------------------------\n')
% Export Thetalist to cvs file
csvwrite('Jointvector_iteration.csv',thetalist')
% Plot angular and linear velocity errors
figure(1)
plot(0:1:i,w_mod)
xlabel('Iteration number')
ylabel('Angular error magnitude (rad/s)')
figure(2)
plot(0:1:i,v_mod)
xlabel('Iteration number')
ylabel('Linear error magnitude (mm/s)')
end

