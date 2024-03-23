%% Plot Error
figure()
set(gcf,"Position",[10 10 800 400])
tgl = tiledlayout(1,2);

nexttile(tgl)
plot(t,Xs(1:3,:)',"LineWidth",2)
title("Angular Velocity Error","Interpreter","Latex")
xlabel("Time (s)","Interpreter","Latex")
ylabel("Angular Velocity (rad/s)","Interpreter","Latex")
xlim([0 6])
ylim([-1 1])
xline(t(2*k/dt+63),'--',"Cube Grabbed","Interpreter","Latex");
xline(t(5*k/dt+2*63),'--',"Cube Placed","Interpreter","Latex");
legend("$\bf{w_x}$","$\bf{w_y}$","$\bf{w_z}$",'Location','southeast',"Interpreter","Latex")
grid on

nexttile(tgl)
plot(t,Xs(4:6,:)',"LineWidth",2)
title("Linear Velocity Error","Interpreter","Latex")
xlabel("Time (s)","Interpreter","Latex")
ylabel("Velocity (m/s)","Interpreter","Latex")
xlim([0 6])
ylim(3e-1*[-1 1])
xline(t(2*k/dt+63),'--',"Cube Grabbed","Interpreter","Latex");
xline(t(5*k/dt+2*63),'--',"Cube Placed","Interpreter","Latex");
legend("$\bf{v_x}$","$\bf{v_y}$","$\bf{v_z}$",'Location','southeast',"Interpreter","Latex")
grid on

title(tgl,"\bf{Error Twist vs. Time}","Interpreter","Latex")
subtitle(tgl,{"$\bf{K_{p} =}$ " + Kp(1),"$\bf{K_{i} =}$ " + Ki(1)},"Interpreter","Latex")