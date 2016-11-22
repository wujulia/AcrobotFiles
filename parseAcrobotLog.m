checkDependency('lcm');
javaaddpath('LCMTypes/acrobot_types.jar')

filename = '/data/mposa/Dropbox (MIT)/AcrobotLogs/11-11-2016/lcmlog-2016-11-11.00';
channels = {'acrobot_y','acrobot_xhat','acrobot_u','acrobot_out'};
coders = {AcrobotYCoder(),AcrobotStateCoder(),AcrobotInputCoder(),AcrobotOutCoder()};
data = readLog(filename,channels,coders);

%%
plant = AcrobotPlantSmooth;

t = data{2}.t;
x = data{2}.data;
u = data{3}.data;
y = data{1}.data;

u = interp1(data{3}.t,u,t,'spline','extrap');
y = interp1(data{1}.t,y',t,'spline','extrap')';

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

%% mimic state estimate
x_est = x*0;
x_est(:,1) = x(:,1);
L = diag([1;1;.25;.25]);

v_filt = filter(hamming(5),sum(hamming(5)),v_diff(1,:));

for i=2:length(t),
  x_meas = [y(1:2,i);(y(1:2,i) - y(1:2,i-1))/(t(i) - t(i-1))];
  xdot(:,i) = plant.dynamics(0,x_est(:,i-1),u(:,i-1));
  x_pred = x_est(:,i-1) + (t(i) - t(i-1))*xdot_pred;
  x_est(:,i) = x_pred  + L*(x_meas-x_est(:,i-1));
end

xdot_filt = filter(hamming(5),sum(hamming(5)),xdot(3,:));