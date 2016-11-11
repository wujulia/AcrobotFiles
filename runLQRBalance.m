checkDependency('lcm');

q0 = [0;0];
u0 = 0;

%%
p = AcrobotPlantSmooth;


x0 = [q0;0;0];
[A,B] = p.linearize(0,x0,u0);

Q = diag([10;10;1;1]);
R = .1;
[K,S] = lqr(A,B,Q,R);

u0_traj = ConstantTrajectory(u0);
x0_traj = ConstantTrajectory(x0);
K_traj = ConstantTrajectory(K);


c = AcrobotFeedbackController(u0_traj,K_traj,x0_traj);

disp(sprintf('Starting feedback controller about point (%f,%f). Sending commands now...',q0(1),q0(2)))
runLCM(uc,[],[]);