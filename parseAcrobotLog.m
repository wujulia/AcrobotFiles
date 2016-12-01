checkDependency('lcm');
javaaddpath('LCMTypes/acrobot_types.jar')

% filename = 'AcrobotLogs/11-11-2016/lcmlog-2016-11-11.00';
filename = 'AcrobotLogs/11-30-2016/successfulSine.mod';
channels = {'acrobot_y','acrobot_xhat','acrobot_u','acrobot_out'};
coders = {AcrobotYCoder(),AcrobotStateCoder(),AcrobotInputCoder(),AcrobotOutCoder()};
data = readLog(filename,channels,coders,.5,.77);

%%
plant = AcrobotPlantSmooth;

t = unique(data{1}.t);

[~,Ix] = unique(data{2}.t);
x = data{2}.data(:,Ix);

[~,Iu] = unique(data{3}.t);
u = data{3}.data(:,Iu);

[~,Iy] = unique(data{1}.t);
y = data{1}.data(:,Iy);


y = interp1(data{1}.t(Iy),y',t,'linear','extrap')';
x = interp1(data{2}.t(Ix),x',t,'linear','extrap')';
u = interp1(data{3}.t(Iu),u,t,'linear','extrap');


y(1:2,:) = y(1:2,:) + repmat(x(1:2,1) - y(1:2,1),1,length(t));

t = t-t(1);
%%

xdot = x*0;
for i=1:length(t),
  xdot(:,i) = plant.dynamics(t(i),x(:,i),u(i));
end

q = x(1:2,:);
v = x(3:4,:);
v_diff = [diff(q(1,:))./diff(t);diff(q(2,:))./diff(t)];
v_diff = [zeros(2,1) v_diff];

vdot_diff = [diff(v(1,:))./diff(t);diff(v(2,:))./diff(t)];
vdot_diff = [zeros(2,1) vdot_diff];

vdot_diff_filt = [filter(hamming(5),sum(hamming(5)),vdot_diff(1,:)); filter(hamming(5),sum(hamming(5)),vdot_diff(2,:))];

%% mimic state estimate
x_est = x*0;
x_est(:,1) = x(:,1);
L = diag([1;1;.25;.25]);

v_filt = filter(hamming(5),sum(hamming(5)),v_diff(1,:));

for i=2:length(t),
  x_meas = [y(1:2,i);(y(1:2,i) - y(1:2,i-1))/(t(i) - t(i-1))];
  xdot(:,i) = plant.dynamics(0,x_est(:,i-1),u(:,i-1));
  x_pred = x_est(:,i-1) + (t(i) - t(i-1))*xdot(:,i);
  x_est(:,i) = x_pred  + L*(x_meas-x_est(:,i-1));
end

xdot_filt = filter(hamming(5),sum(hamming(5)),xdot(3,:));

%% EKF state estimate
C = [eye(2) zeros(2)]; % measurement model
x_ekf = x*0;
xdot_ekf = x*0;
xdot_test = x*0;
x_ekf(:,1) = x(:,1);
P = zeros(4,4,length(t));


P(:,:,1) = eye(4);  % initial covariance
Q = diag([1e-7;1e-7;.01;.01]); %process noise covariance
R = diag([1e-4;3e-4]); % measurement covariance, from tick resolution

for i=2:length(t),
  [xdot_ekf(:,i),dxdot] = plant.dynamics(0,x_ekf(:,i-1),u(:,i-1));
  [xdot_test(:,i)] = plant.dynamics(0,x_ekf(:,i-1),u(:,i-1));
  F = eye(4) + (t(i) - t(i-1))*dxdot(:,2:5);
  
  % predict
  x_pred = x_ekf(:,i-1) + (t(i) - t(i-1))*xdot_ekf(:,i);
  P_pred = F*P(:,:,i-1)*F' + Q;
  
  %update
  y_resid = y(1:2,i) - C*x_pred;
  S = C*P_pred*C' + R;
  K = P_pred*C'/S;
  
  x_ekf(:,i) = x_pred + K*y_resid;
  P(:,:,i) = (eye(4) - K*C)*P_pred;
end

figure(1)
subplot(3,1,1)
plot(t,x_ekf(3,:),t,x(3,:),t,x(3,:),t,v_diff(1,:)); legend('ekf','est')
subplot(3,1,2)
plot(t,x_ekf(4,:),t,x(4,:),t,x(4,:),t,v_diff(2,:)); legend('ekf','est')
subplot(3,1,3)
plot(t,xdot_ekf(3:4,:))

% figure(2)
% plot(t,xdot_test(3:4,:))

figure(3)
subplot(2,1,1)
plot(t,vdot_diff(1,:),t,xdot(3,:))
subplot(2,1,2)
plot(t,vdot_diff(2,:),t,xdot(4,:))

%% Bias EKF state estimate
C = [eye(2) zeros(2,4)]; % measurement model
x_bekf = zeros(6,length(t));
xdot_bekf = zeros(6,length(t));
x_bekf(:,1) = [x(:,1);0;0];
P = zeros(6,6,length(t));

T = [eye(4) [eye(2);zeros(2)]]; % transform x to [q;v]

P(:,:,1) = diag([1;1;1;1;.05;.05]);  % initial covariance
Q = diag([1e-7;1e-7;.01;.01;1e-4;1e-4]); %process noise covariance
R = diag([1e-4;3e-4]); % measurement covariance, from tick resolution

for i=2:length(t),
  [xdot_bekf(1:4,i),dxdot] = plant.dynamics(0,T*x_bekf(:,i-1),u(:,i-1));
  xdot_bekf(5:6,i) = zeros(2,1);
  F = eye(6) + (t(i) - t(i-1))*[dxdot(:,2:5)*T; zeros(2,6)];
  
  % predict
  x_pred = x_bekf(:,i-1) + (t(i) - t(i-1))*xdot_bekf(:,i);
  P_pred = F*P(:,:,i-1)*F' + Q;
  
  %update
  y_resid = y(1:2,i) - C*x_pred;
  S = C*P_pred*C' + R;
  K = P_pred*C'/S;
  
  x_bekf(:,i) = x_pred + K*y_resid;
  P(:,:,i) = (eye(6) - K*C)*P_pred;
end

%%
figure(4)
subplot(2,1,1)
plot(t,vdot_diff(1,:),t,xdot_bekf(3,:))
subplot(2,1,2)
plot(t,vdot_diff(2,:),t,xdot_bekf(4,:))

figure(5)
plot(t,x_bekf(5:6,:))