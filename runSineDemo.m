checkDependency('lcm');
javaaddpath('LCMTypes/acrobot_types.jar')

q0 = [0;0];
u0 = 0;

x0 = [q0;0;0];
% Calibrate first
x_offset = calibrateAcrobot(q0);
q_offset = x_offset(1:2);

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
uc = AcrobotFeedbackController(p,u_traj,K_traj,x_traj);
R_ekf = diag([1e-4;3e-4]); % measurement covariance, from tick resolution
Q_ekf = diag([1e-7;1e-7;.01;.01;1e-4;1e-4]); %process noise covariance
estimator = AcrobotBEKFEstimator(p,R_ekf,Q_ekf,q_offset);
sys = estimator.cascade(uc);
P0 = diag([1;1;1;1;.05;.05]);  % initial covariance
init_state = estimator.wrapState(0,[x0;0;0],P0);
display('Starting sine demo commands now')
runLCM(sys,init_state,[]);