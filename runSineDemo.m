checkDependency('lcm');
javaaddpath('LCMTypes/acrobot_types.jar')

q0 = [0;0];
u0 = 0;

x0 = [q0;0;0];


%%
p = AcrobotPlantSmooth;

t_s = linspace(0,10,1e4);
u_s = 3*sin(2*t_s);

u_traj = PPTrajectory(foh(t_s,u_s));
u_traj = u_traj.setOutputFrame(p.getInputFrame);

sys_ol = u_traj.cascade(p);
x_traj = sys_ol.simulate([0 t_s(end)],x0);

Q = diag([10;10;1;1]);
R = 5;
Qf = Q;

[ltvsys,V] = p.tvlqr(x_traj,u_traj,Q,R,Qf);

K_traj = -ltvsys.D;

%%
uc = AcrobotFeedbackController(u_traj,K_traj,x_traj);
runLCM(uc,[],[]);