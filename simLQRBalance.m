checkDependency('lcm');
javaaddpath('LCMTypes/acrobot_types.jar')

q0 = [pi;0];
u0 = 0;

%%
p = AcrobotPlantSmooth;


x0 = [q0;0;0];
[A,B] = p.linearize(0,x0,u0);
% xdot ~ A*(x-x0) + B*(u-u0)

Q = diag([1;1;1;1]);
R = .10;
[K,S] = lqr(A,B,Q,R);

% K = [0; -1; 0; -.1]';
K
u0_traj = ConstantTrajectory(u0);
x0_traj = ConstantTrajectory(x0);
K_traj = ConstantTrajectory(K);


uc = AcrobotFeedbackController(p,u0_traj,K_traj,x0_traj,false);
sys = p.feedback(uc);

xtraj = sys.simulate([0 5],x0-.05*randn(4,1));

%%
figure(1); 
t = xtraj.pp.breaks;
x = xtraj.eval(t); 
plot(t,x(1:2,:))