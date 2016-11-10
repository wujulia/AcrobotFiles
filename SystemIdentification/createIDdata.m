% Load data and convert to idddata (needed for system identification)
% Add path to data
% addpath('../Data/');
% Load data
%load dataTraj.mat

% Subsample and get theta derivatives
dt = 0.005; % CHANGE BOTTOM TOO IF YOU CHANGE THIS
t_sample = t(1):dt:t(end);
dt = diff(t_sample(1:2));
theta1_spl = spline(t,theta1);
theta2_spl = spline(t,theta2);
torque_spl = spline(t,torque);
theta1_spl_dot = fnder(theta1_spl);
theta2_spl_dot = fnder(theta2_spl);
theta1_dot = ppval(theta1_spl_dot,t_sample);
theta2_dot = ppval(theta2_spl_dot,t_sample);
theta1 = ppval(theta1_spl,t_sample);
theta2 = ppval(theta2_spl,t_sample);
torque = ppval(torque_spl,t_sample);

outputs_full = [theta1;theta2;theta1_dot;theta2_dot];
outputs = [theta1;theta2];

zTraj = iddata(outputs',torque',0.005);
